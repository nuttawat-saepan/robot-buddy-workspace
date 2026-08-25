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
Unitree board's `eth0` also uses 192.168.123.18, so the MiniPC profile has to
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
