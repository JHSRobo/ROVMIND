#!/bin/bash
read -p "Your name: " name
read -p "Your email associated with GIT: " email
git config --global user.name "$name"
git config --global user.email "$email"
git config --global color.ui auto
git config --global branch.autosetupmerge true
git config --global push.default tracking
git config credential.helper store
git pull
