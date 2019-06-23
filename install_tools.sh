#sudo add-apt-repository ppa:git-core/ppa
#sudo apt-get update

#sudo apt-get install git
#git config core.hooksPath .githooks
#sudo apt install clang-format

#chmod +x .githooks/apply-format
#chmod +x .githooks/pre-commit

toInstall=("ros-kinetic-desktop-full" "python-rosinstall" 
    "python-rosinstall-generator" "python-wstool" "build-essential" 
    "python-catkin-tools")

notInstalled=()
installed=()

softwareCount=0

while [ "$softwareCount" -lt "6" ]
do
dpkg -s "${toInstall[softwareCount]}" &> /dev/null 
    if [ $? -ne 0 ]; then
       notInstalled[softwareCount]="${toInstall[softwareCount]}"
    else
       installed[softwareCount]="${toInstall[softwareCount]}"
    fi
softwareCount=$[softwareCount+1]
done

if [ "${#installed[@]}" -gt "0" ]; then
	echo "The installed are ${installed[@]}"
else
	echo "None of the required packages have been installed"
fi

if [ "${#notInstalled[@]}" -gt "0" ]; then
	echo "The package(s) that need to be installed are ${notInstalled[@]}"
	read -p "Do you want to install them now (y/n)? " -n 1 -r 
	echo
	if [[ ! "$Reply" =~ ^[Yy]$ ]]; then
    		sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
		sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
		sudo apt-get update
		sudo apt-get install ros-kinetic-desktop-full
		sudo rosdep init
		rosdep update
		echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
		source ~/.bashrc
		sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
		sudo apt install python-catkin-tools
	fi
else
	echo "All required packages have been installed"
fi

