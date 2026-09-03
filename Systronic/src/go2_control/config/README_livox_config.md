# Livox Mid-360 driver config

`livox_mid360_field.json` is the config that actually brought this sensor up on
2026-08-19, copied out of `~/ws_livox/src/livox_ros_driver2/config/` so it is
not the one file the whole sensor path depends on living outside version
control.

## The device variant matters

This unit reports `dev_type = 35`, despite being labelled MID-360. That variant
needs the JSON root key **`Mid360s`**, not `MID360`, and `host_net_info` as an
**array** of objects carrying `host_ip` - not a single object. With the wrong
schema the driver starts, prints `Init lds lidar success!`, and then publishes
nothing at all.

The diagnostic: a working start logs a `GetFreeIndex` line. A hung one does not.

## IPs are site-specific

```text
lidar ip    192.168.123.20
host_ip     192.168.123.18
```

The host address is whichever machine the sensor is cabled to. Note that the
### 192.168.123.x is the Go2's own network, not a LiDAR-only link

Discovered on site, 2026-08-25, and worth knowing before planning any
deployment. Probing the subnet from the MiniPC's `enp3s0` finds two devices:

```text
192.168.123.20    the Mid-360
192.168.123.161   the Unitree board's wired side
```

The board answers on that wired address and on `192.168.68.70` over the site
Wi-Fi - one machine, two interfaces. The cable is the more dependable of the
two: on 2026-09-01 the Wi-Fi address accepted a deploy and then disappeared
from the network minutes later, while the wired address kept accepting SSH.

Log in as the `unitree` account; `sys20` is the MiniPC's user and does not
exist on the board:

```bash
ssh unitree@192.168.123.161
```

The board does not answer ICMP on either address, so `ping` failing proves
nothing about whether it is up. Check the SSH port instead, with
`nc -zv -w4 192.168.123.161 22`.

So the cable from the sensor does not run to a private point-to-point link with
the MiniPC - it joins the robot's internal network, and the MiniPC joins the
same one. That is why the arrangement can be remembered as "the driver went
through the board": physically it does. The driver process, however, has always
run on the MiniPC, which is what `MID360_minipc.json` and the `192.168.123.18`
host address record.

The practical consequence is larger than the naming. **The board can be
deployed to and built on from the MiniPC today** - see
`scripts/deploy_to_board.sh`, which has been run against
the board. The board build has been the single blocker on the
two-machine deployment, and nothing else has to be settled before it starts.

The Unitree board's `eth0` also uses 192.168.123.18, so the MiniPC profile has to
come down before the sensor is moved onto the board:

```bash
sudo nmcli con down livox-direct
```

On the MiniPC that address comes from a persistent NetworkManager profile,
created deliberately with no gateway so the LiDAR subnet cannot take over the
default route:

```bash
sudo nmcli con add type ethernet ifname enp3s0 con-name livox-direct \
  ipv4.method manual ipv4.addresses 192.168.123.18/24 ipv6.method ignore
```

## Using it

```bash
ros2 launch go2_control livox_mid360_driver.launch.py \
  user_config_path:=<this file>
```

`livox_mid360_field.example.json` next to it is the generic MID360-schema
template and is kept only for reference. Use this file.
