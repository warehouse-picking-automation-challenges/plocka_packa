How to fix time synchronization issues in ROS. This is solves a lot of TF issues.  

```
sudo apt-get install ntp  
sudo gedit /etc/ntp.conf
```

Comment all the  server lines and add the UTA ntp servers:
```
server 129.107.24.1 iburst  
server 129.107.57.1
```

Restart ntp:
```
sudo /etc/init.d/ntp restart
```

The system clock should be updated in about 10-15 seconds (or up to 15-20 minutes without 'iburst').


PR2
----
```
sudo nano /etc/chrony/chrony.conf.pr2-basestation  
server 129.107.24.1  
server 129.107.57.1  
sudo invoke-rc.d chrony restart
```
