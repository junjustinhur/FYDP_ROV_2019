### This will enable static IP by removing the appropriate '#'s from the interfaces file
### NOTE that this means that interfaces file MUST already be setup for static IP beforehand

#!/bin/sh
sudo sed -i -e "s/#auto/auto/" -e "s/#iface/iface/" -e "s/#	/	/" /etc/network/interfaces
sudo systemctl disable dhcpcd
sudo systemctl enable networking
sudo /etc/init.d/networking restart