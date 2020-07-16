echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout",  SYMLINK+="huanyu_base"' >/etc/udev/rules.d/huanyu_base.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout",  SYMLINK+="huanyu_laser"' >/etc/udev/rules.d/huanyu_laser.rules
service udev reload
sleep 2
service udev restart


