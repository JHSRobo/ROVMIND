#! /bin/zsh

BASEDIR=$(dirname "$0")

cp "${BASEDIR}/CustomZshCommands.zsh" ~/
chmod +x ~/CustomZshCommands.zsh

echo -e "\n#From rovotics setup.zsh" >> ~/.zshrc

if ! grep -q -x -F "source ~/CustomZshCommands.zsh" ~/.zshrc; then
   echo "source ~/CustomZshCommands.zsh" >> ~/.zshrc
else
   echo "source ~/CustomZshCommands.zsh already in zshrc"
fi


echo "Setup finished! "

exec zsh #start new bash terminal in order to source the bashrc
