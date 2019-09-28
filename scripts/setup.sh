#! /bin/bash

BASEDIR=$(dirname "$0")

cp "${BASEDIR}/CustomCommands.sh" ~/
chmod +x ~/CustomCommands.sh

echo -e "\n#From rovotics setup.sh" >> ~/.bashrc

if ! grep -q -x -F "source ~/CustomCommands.sh" ~/.bashrc; then
   echo "source ~/CustomCommands.sh" >> ~/.bashrc
else
   echo "source ~/CustomCommands.sh already in bashrc"
fi


echo "Setup finished! "

exec bash #start new bash terminal in order to source the bashrc
