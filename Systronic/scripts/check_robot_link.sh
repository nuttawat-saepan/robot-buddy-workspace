#!/usr/bin/env bash
# Check whether the Unitree board and the ground station can actually run ROS 2
# between them, before anything depends on it.
#
# The failure this exists for looks the same whatever caused it: `ros2 topic
# list` comes back empty. That single symptom has at least six causes - the AP
# isolating its clients, the AP dropping multicast, mismatched ROS_DOMAIN_ID,
# mismatched RMW implementation, a CycloneDDS interface binding that excludes
# the wireless card, or a firewall - and working through them one at a time in
# the field costs hours. This narrows it down in about two minutes.
#
# It is also the missing baseline for the one network measurement this project
# has: 33% loss and 395-1453 ms RTT, recorded on 2026-08-19 while a point cloud
# was being streamed. Nobody ever measured the link idle, so it is still not
# known whether the Wi-Fi itself is usable. Run step 3 before drawing any
# conclusion about the deployment architecture.
#
# Usage, on both machines, ground station first:
#
#     ./check_robot_link.sh listen                  # ground station
#     ./check_robot_link.sh check <ground-station-ip>   # robot
#
# Read-only. Sends test packets and nothing else; touches no robot topic.

set -u

ROLE="${1:-}"
PEER="${2:-}"
PING_COUNT="${PING_COUNT:-50}"

c_ok()   { printf '  \033[32mOK\033[0m   %s\n' "$1"; }
c_bad()  { printf '  \033[31mFAIL\033[0m %s\n' "$1"; }
c_warn() { printf '  \033[33mWARN\033[0m %s\n' "$1"; }
hdr()    { printf '\n== %s ==\n' "$1"; }

usage() {
    cat <<'USAGE'
usage:
  check_robot_link.sh listen              run this on the ground station first
  check_robot_link.sh check <peer-ip>     then this on the robot

  PING_COUNT=200 check_robot_link.sh check 192.168.1.5     longer jitter sample
USAGE
    exit 2
}

# ---------------------------------------------------------------- local state
report_local() {
    hdr "This machine"
    echo "  hostname         $(hostname)"
    echo "  interfaces"
    ip -o -4 addr show scope global 2>/dev/null \
        | awk '{printf "                   %-10s %s\n", $2, $4}'

    echo "  ROS_DOMAIN_ID    ${ROS_DOMAIN_ID:-unset (= 0)}"
    echo "  RMW              ${RMW_IMPLEMENTATION:-unset (= rmw_fastrtps_cpp)}"
    if [ -n "${CYCLONEDDS_URI:-}" ]; then
        # A CycloneDDS interface binding that names only the wired card is the
        # quiet reason a wireless link never discovers anything. Unitree's
        # setup.sh ships exactly that.
        local ifs
        ifs=$(printf '%s' "$CYCLONEDDS_URI" \
              | grep -oE 'NetworkInterface name="[^"]+"' \
              | cut -d'"' -f2 | paste -sd, -)
        echo "  CYCLONEDDS_URI   binds to: ${ifs:-<could not parse>}"
        c_warn "these must include the interface carrying the link between the two machines"
    else
        echo "  CYCLONEDDS_URI   unset"
    fi
    echo
    echo "  ROS_DOMAIN_ID and RMW must be IDENTICAL on both machines."
    echo "  FastRTPS and CycloneDDS cannot talk to each other at all."
}

# --------------------------------------------------------------------- checks
check_reachable() {
    hdr "1. Can the two machines reach each other"
    if ping -c 3 -W 2 "$PEER" >/dev/null 2>&1; then
        c_ok "$PEER responds to ping"
        return 0
    fi

    c_bad "$PEER does not respond to ping"
    local gw
    gw=$(ip route | awk '/^default/{print $3; exit}')
    if [ -n "$gw" ] && ping -c 2 -W 2 "$gw" >/dev/null 2>&1; then
        c_bad "but the access point at $gw does respond"
        echo
        echo "  That combination is client isolation: the AP lets each device"
        echo "  reach the internet but not each other. Shared and guest"
        echo "  networks enable it by default and it cannot be worked around"
        echo "  from this end. Use the Go2's own access point, or a small"
        echo "  dedicated router that only these two machines join."
    else
        echo "  The access point is not reachable either - check that both"
        echo "  machines are on the same network before anything else."
    fi
    return 1
}

check_latency() {
    hdr "2. Latency and jitter, idle"
    local out
    out=$(ping -c "$PING_COUNT" -i 0.2 -W 2 "$PEER" 2>/dev/null)
    local loss rtt
    loss=$(printf '%s' "$out" | grep -oE '[0-9.]+% packet loss' | head -1)
    rtt=$(printf '%s' "$out" | awk -F'= ' '/rtt|round-trip/{print $2}')
    echo "  packets          $PING_COUNT over $(echo "$PING_COUNT * 0.2" | bc 2>/dev/null || echo '?')s"
    echo "  loss             ${loss:-unknown}"
    echo "  rtt min/avg/max/mdev  ${rtt:-unknown}"
    echo
    echo "  mdev is jitter, and it matters more than avg: Nav2 sends velocity"
    echo "  at 5 Hz, so a link that is usually fast but stalls for 300 ms"
    echo "  leaves the robot executing a stale command. The robot-side"
    echo "  watchdog stops it after 0.5 s, which is safe but useless for"
    echo "  driving."
    echo
    echo "  Rough guide for this system:"
    echo "    avg under  20 ms, mdev under 10 ms, loss 0%   comfortable"
    echo "    avg under 100 ms, mdev under 50 ms, loss <1%  usable"
    echo "    anything worse                                do not drive over it"
}

check_multicast() {
    hdr "3. Multicast, which ROS 2 discovery depends on"
    echo "  Listening for 10 s. The peer must be running 'listen' mode,"
    echo "  which sends on a loop."
    local out
    out=$(timeout 10 ros2 multicast receive 2>&1)
    # Case-insensitive on purpose: ros2 multicast receive prints
    # "Received from ...", and matching the lowercase form found nothing while
    # the datagram was arriving perfectly well.
    if printf '%s' "$out" | grep -qi "received from"; then
        c_ok "multicast arrives"
        printf '%s\n' "$out" | head -2 | sed 's/^/       /'
    else
        c_bad "no multicast received in 10 s"
        echo
        echo "  If step 1 passed but this failed, the AP is filtering"
        echo "  multicast - common on shared networks, where it is dropped to"
        echo "  save airtime. ROS 2 discovery will not work: ping succeeds,"
        echo "  'ros2 topic list' stays empty, and nothing explains why."
        echo
        echo "  Fixes, in order of preference: use the Go2's own AP or a"
        echo "  dedicated router; or configure a DDS discovery server so"
        echo "  discovery no longer needs multicast."
    fi
}

check_throughput() {
    hdr "4. Throughput"
    if ! command -v iperf3 >/dev/null 2>&1; then
        c_warn "iperf3 not installed, skipping"
        echo "  This step is the least important of the four. The deployment"
        echo "  needs about 0.33 Mbps: /scan at 3 KB x 10 Hz, plus odometry,"
        echo "  TF and velocity. Any link that passes steps 1 to 3 has orders"
        echo "  of magnitude more than that."
        echo
        echo "  What is NOT sent over the link, and must never be, is the point"
        echo "  cloud: 41.6 Mbps fragmented into hundreds of UDP packets per"
        echo "  frame, which collapses a wireless link rather than slowing it."
        echo "  Install iperf3 on both machines if you want a number here."
        return
    fi
    echo "  Run 'iperf3 -s' on the peer, then re-run this step."
    iperf3 -c "$PEER" -t 5 2>&1 | tail -5 | sed 's/^/  /'
}

# ----------------------------------------------------------------------- main
case "$ROLE" in
    listen)
        report_local
        hdr "Multicast sender"
        echo "  Sending on a loop. Run the 'check' side on the other machine,"
        echo "  then stop this with Ctrl-C."
        echo
        while true; do
            ros2 multicast send >/dev/null 2>&1
            sleep 1
        done
        ;;
    check)
        [ -n "$PEER" ] || usage
        report_local
        if check_reachable; then
            check_latency
            check_multicast
            check_throughput
        else
            hdr "Stopping here"
            echo "  The machines cannot reach each other, so the remaining"
            echo "  checks would only report the same fault again."
        fi
        hdr "Done"
        ;;
    *)
        usage
        ;;
esac
