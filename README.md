# kaiROS
CAVs team system repository. This is the main software that will run on the Tank.

## CircleCI build status
![CircleCI](https://img.shields.io/circleci/build/github/uwaft/kaiROS?token=cc2c9a2c8395746b93cead02eb170c6642d5cbf4)


## Instruction: From clone to build and run
### Clone the repo
The system is built in [Robot Operating System (ROS)](http://wiki.ros.org/) running in Ubuntu 16.04. You can get Ubuntu 16.04 on a Virtual Machine or Dual Booting.

Before we begin, make sure you have SSH key set up for your GitHub account.

If you have not, please follow these two links to do so:

1. [Generate SSH key](https://help.github.com/en/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)

2. [Add SSH key to GitHub account](https://help.github.com/en/articles/adding-a-new-ssh-key-to-your-github-account)


Now, go to your ubuntu os and open a terminal window, change directory to where you want to store this repo locally using `cd`.

Then run

```
git clone git@github.com:uwaft/kaiROS.git
```

### Install tools on your local machine
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


### Build kaiROS for the first time
Under the kaiROS folder, 

Execute

```
catkin build
```
If everything is correct, you will see
```
[build] Summary: All X packages succeeded!
```

To test if ROS is running properly, Execute in one terminal (It could take a while):
```
roscore
```
Open a new terminal, then run
```
rosrun ecmc raw_data_publisher
```

Open a third new terminal and execute
```
rosnode list
```
in the new terminal. You should see raw_data_publisher running as a node.
