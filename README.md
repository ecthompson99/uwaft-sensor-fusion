

# kaiROS
CAV system repo. This is the main software that will run on the Tank.

https://img.shields.io/circleci/build/github/uwaft/kaiROS?token=cc2c9a2c8395746b93cead02eb170c6642d5cbf4



## Instruction: From clone to build and run
### Clone the repo
In terminal, change directory to where you want to store this repo locally using `cd`.

Before the next step, make sure you have SSH key set up for your GitHub account.

If you have not, please follow these two links to do so:

https://help.github.com/en/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent

https://help.github.com/en/articles/adding-a-new-ssh-key-to-your-github-account


Run

```
git clone git@github.com:uwaft/kaiROS.git
```

### Install ROS on your local machine
Change directory into the kaiROS folder by

```
cd kaiROS/
```

Execute the 'install_tools.sh' by running the following:

```
./install_tools.sh
```

This will install all the ROS related packages, along with the latest version of Git and Clang for you automatically.

The same info is available on [Confluence](https://wiki.uwaterloo.ca/display/UWAFT/Intro+to+ROS).


### Build and run
Change directory into the cloned kaiROS folder by 'cd'.

Execute

```
catkin build
```

To test if ROS is running properly, Execute in terminal (It could take a while):
```
roscore
```
Open a new terminal, then run
```
rosrun ecmc hello_world_publisher
```

Open another new terminal and execute
```
rosnode list
```
in the new terminal. You should see hello_world_publisher running as a node.
