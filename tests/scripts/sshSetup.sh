#! /bin/bash

#https://www.maketecheasier.com/setup-enable-ssh-ubuntu/
#https://debian-administration.org/article/530/SSH_with_authentication_key_instead_of_password

sudo apt-get install openssh-server

sudo cp /etc/ssh/sshd_config /etc/ssh/sshd_config.factory-defaults #Backup conifg file

sudo chmod a-w /etc/ssh/sshd_config.factory-defaults

sudo restart ssh

#setting up keys
mkdir ~/.ssh
chmod 700 ~/.ssh
ssh-keygen -t rsa
roboip="$(hostname -I | sed -e 's/ //g')" #get ip address and remove whitespace

if [ "$roboip" = "192.168.1.100" ]; then
   ssh-copy-id -i ~/.ssh/id_rsa.pub bottomside
elif [ "$roboip" = "192.168.1.111" ]; then
   ssh-copy-id -i ~/.ssh/id_rsa.pub master
else
   echo "Error setting up ssh keys for ROS! "
fi


echo "ssh setup complete!"
