@echo -off
echo Running startup.nsh - The Auto-Start UEFI Shell Script
@echo -off
echo Running startup.nsh - The Auto-Start UEFI Shell Script
@echo -on
ifconfig -s eth0 static 192.168.0.9 255.255.255.0 192.168.0.1
ifconfig -l eth0
ping 192.168.0.1 -n 2
@echo .
fs1:
ls
tftp -r 70 192.168.0.1 vxWorks
runaxf vxWorks



