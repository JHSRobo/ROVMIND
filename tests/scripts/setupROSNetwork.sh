#! /bin/bash

if ! grep "192.168.1.100 master" /etc/hosts ; then
   sudo -- sh -c "echo 192.168.1.100   master >> /etc/hosts"
fi

if ! grep "192.168.1.111 bottomside" /etc/hosts ; then
   sudo -- sh -c "echo 192.168.1.111   bottomside >> /etc/hosts"
fi

if ! grep "192.168.1.101 control" /etc/hosts ; then
   sudo -- sh -c "echo 192.168.1.101   control >> /etc/hosts"
fi


echo -n "What computer are you on? 'topside', 'bottomside', or 'control' (tcu touchscreen)?"
read response

if ! grep "export ROS_MASTER_URI=http://master:11311" ~/.bashrc ; then
  echo "export ROS_MASTER_URI=http://master:11311" >> ~/.bashrc
fi

if [ "$response" = "topside" ]; then
  if ! grep "export ROS_HOSTNAME=master" ~/.bashrc ; then
   echo -e "\nexport ROS_HOSTNAME=master" >> ~/.bashrc
  fi
  echo -e "\n\nYou are now configured as master!"

elif [ "$response" == "bottomside" ]; then
  if ! grep "export ROS_HOSTNAME=bottomside" ~/.bashrc ; then
   echo -e "\nexport ROS_HOSTNAME=bottomside" >> ~/.bashrc
  fi
  echo -e "\n\nYou are now configured as bottomside!"

elif [ "$response" == "control" ]; then
  if ! grep "export ROS_HOSTNAME=control" ~/.bashrc ; then
   echo -e "\nexport ROS_HOSTNAME=control" >> ~/.bashrc
  fi
  echo -e "\n\nYou are now configured as control!"

else
  echo "Response not found"

fi

echo -e "\n\nTo finish setup, go to the DHCP reservations list on your router, and assign the MAC Address to the following: "
echo "Topside: 192.168.1.100"
echo "Bottomside: 192.168.1.111"
echo "Control (TCU Touchscreen): 192.168.1.101"
