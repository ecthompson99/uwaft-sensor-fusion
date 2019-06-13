# kaiROS
CAV system repo. This is the main software that will run on the Tank.




# Instruction: From clone to build and run
## Install ROS on your local machine
Execute the following one by one in your terminal.


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

The same info is available on [Confluence](https://wiki.uwaterloo.ca/display/UWAFT/Intro+to+ROS).

## Clone the repo
In terminal, change directory to where you want to store this repo locally using 'cd'.

Run

    git clone git@github.com:uwaft/kaiROS.git

## Build and run
Change directory into the cloned kaiROS folder by 'cd'.

Execute 

    catkin build

To test if ROS is running properly, Execute in terminal:

    roscore

Open a new terminal, then run

    ./kaiROS/devel/lib/my_first_package/hello_world
    

Open another new terminal and execute

    rosnode list

in the new terminal. You should see hello_world running as a node.