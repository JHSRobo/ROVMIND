#! /bin/zsh

function clion(){
   cmdpath=$(find / -name "clion.sh")
   ROSWorkspace=$(find /home -type d -name "ros_workspace")
   continue="f"
   while [ $continue != "t" ];
   do
      if [ ! -z $(sudo find "${ROSWorkspace}" -name "devel")  ]; then
         source "${ROSWorkspace}/devel/setup.zsh"
         echo "${ROSWorkspace}/devel/setup.zsh successfully sourced! "
         continue="t"
      else
         read -p "Source of ${ROSWorkspace}/devel/setup.zsh failed do you wish to continue (t/f)? " continue
         if [ $continue != "t" ]; then
            read -p "Enter the ROS package you would like to source: " ROSWorkspace
         fi
      fi

    done

    sudo "${cmdpath}"
}

function webstorm(){
   cmdpath=$(find / -name "webstorm.sh")
   ROSWorkspace=$(find /home -type d -name "ros_workspace")
   continue="f"
   while [ $continue != "t" ];
   do
      if [ ! -z $(sudo find "${ROSWorkspace}" -name "devel")  ]; then
         source "${ROSWorkspace}/devel/setup.zsh"
         echo "${ROSWorkspace}/devel/setup.zsh successfully sourced! "
         continue="t"
      else
         read -p "Source of ${ROSWorkspace}/devel/setup.zsh failed do you wish to continue (t/f)? " continue
         if [ $continue != "t" ]; then
            read -p "Enter the ROS package you would like to source: " ROSWorkspace
         fi
      fi

    done

    sudo "${cmdpath}"
}

function pycharm(){
   cmdpath=$(find / -name "pycharm.sh")
   ROSWorkspace=$(find /home -type d -name "ros_workspace")
   continue="f"
   while [ $continue != "t" ];
   do
      if [ ! -z $(sudo find "${ROSWorkspace}" -name "devel")  ]; then
         source "${ROSWorkspace}/devel/setup.zsh"
         echo "${ROSWorkspace}/devel/setup.zsh successfully sourced! "
         continue="t"
      else
         read -p "Source of ${ROSWorkspace}/devel/setup.zsh failed do you wish to continue (t/f)? " continue
         if [ $continue != "t" ]; then
            read -p "Enter the ROS package you would like to source: " ROSWorkspace
         fi
      fi

    done

    sudo "${cmdpath}"
}

function arduino(){
   cmdpath=$(find / -type f -name arduino)
   
   sudo "${cmdpath}"
}

