### This will disable static IP by commenting out the interfaces file
### NOTE that this means that interfaces file MUST already be setup for static IP beforehand

#!/bin/sh
sudo sed -i -e "s/auto/#auto/" -e "s/iface/#iface/" -e "s/	/#	/" /etc/network/interfaces
sudo systemctl enable dhcpcd
sudo systemctl disable networking
sudo /etc/init.d/networking restart