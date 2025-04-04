# Avetics Integration

## Installing dependencies
- Follow README.md at top directory

## Hardware

User: nvidia, pass: nvidia

- Onboard computer: Nvidia Orin NX
- Flight Controller Unit:
- Network adaptor: Viumesh
- 


## Networking

### Accesing router
User: admin, pass: admin

### Network manager settings

For GCS computer:

<img src="images/gcs_network_settings.png" alt="GCS settings" style="width: 500px;"/>

For Drone 0:
- We use host address as default gateway so we can access the internet

<img src="images/drone0_network_settings.png" alt="Gestelt Architecture" style="width: 500px;"/>




GCS: 192.168.17.100
GCS-Viumesh: 192.168.17.1

Drone0-Viumesh: 192.168.17.11
Drone0: 192.168.17.10

### Set up a network bridge on the GCS computer
A network bridge would allow us to forward packets between network interfaces. In this case, we want to have internet access on the drones and therefore we would forward packets within the wireless interface (e.g. wlp5s0) on the GCS and the wired interface (e.g. enp2s0) connected to the GCS viumesh.

```bash
# Use ip link to get the name of the network interfaces you wish to connect
# In this example, we use the wired interface 'enp2s0' and the wireless interface 'wlp5s0'
ip link
# Create bridge named br0
sudo ip link add br0 type bridge
# Show bridge details
ip -d link show br0
# Show bridge details in a pretty JSON format (which is a good way to get bridge key-value pairs):
ip -j -p -d link show br0

# Enable 4addr on wireless interface
sudo iw dev wlp5s0 set 4addr on

# Add interfaces to the bridge
sudo ip link set enp2s0 master br0
sudo ip link set wlp5s0 master br0


# Bring down bridge
sudo ip link set br0 down
# Delete bridge
sudo ip link delete br0 type bridge
```

References:
1. https://developers.redhat.com/articles/2022/04/06/introduction-linux-bridging-commands-and-features#spanning_tree_protocol
2. https://serverfault.com/questions/152363/bridging-wlan0-to-eth0

### Set up NAT on GCS for internet access on drones
```bash
# Use ip link to get the name of the network interfaces you wish to connect
# In this example, we use the wired interface 'enp2s0' and the wireless interface 'wlp5s0'
ip link

echo 1 > /proc/sys/net/ipv4/ip_forward

# Refer to https://linux.die.net/man/8/iptables for flag usage
iptables -t nat -A POSTROUTING -o wlp5s0 -j MASQUERADE

# Assign IP address to yourself:
ifconfig enp2s0 192.168.17.100 netmask 255.255.255.0 up

# Update /etc/dhcp/dhcpd.conf
vim /etc/dhcp/dhcpd.conf
subnet 192.168.17.100 netmask 255.255.255.0 {
    range 192.168.17.0 192.168.17.120;
    option routers 192.168.17.100;
    option domain-name-servers 127.0.0.53;
}

# Start dhcp server
sudo /etc/init.d/isc-dhcp-server start

# Check status of dhcp server
sudo systemctl status isc-dhcp-server

# Check Iptable rules
sudo iptables --table nat --list
```

References:
1. https://serverfault.com/questions/152363/bridging-wlan0-to-eth0
2. https://medium.com/@sydasif78/setting-up-a-dhcp-server-on-ubuntu-a-guide-for-network-engineer-d620c5d7afb2