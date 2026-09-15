# Sourced by setup_robot_env.sh and setup_ground_env.sh. Warns when the IPs in
# onsite.env are not on the network this machine is actually on.
#
# Every site gives a different subnet, and nothing fails when onsite.env still
# holds the last one: the stack starts, the robot publishes to a broker that
# is not there, and the web shows no map and no robot with no error anywhere.
# On 2026-09-14 the board's onsite.env carried two subnets from earlier sites
# at once. This compares the first three octets and says so, loudly.
#
#     _check_site_ips <interface> VAR [VAR ...]
_check_site_ips() {
    local iface="$1"; shift
    local mine
    mine="$(ip -4 -o addr show "$iface" 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -1)"
    if [ -z "$mine" ]; then
        echo "WARNING: $iface has no IPv4 address - not connected to the site network?"
        return
    fi
    local net="${mine%.*}"
    local bad=0 var val
    for var in "$@"; do
        val="${!var}"
        [ -z "$val" ] && continue
        if [ "${val%.*}" != "$net" ]; then
            [ $bad -eq 0 ] && echo && echo "!!! onsite.env does not match this network ($iface is $mine)"
            echo "!!!   $var=$val   expected ${net}.x"
            bad=1
        fi
    done
    if [ $bad -eq 1 ]; then
        echo "!!! edit scripts/onsite.env on this machine before launching - the web will"
        echo "!!! show nothing and nothing will say why. See CHEATSHEET.md, 'new site'."
        echo
    fi
}
