# AR Drone

## Install

### Prepare environment

You need to get Ubuntu 14 and ROS Indigo.
For better experience install Ubuntu alongside with your OS.
Install [ROS-Indigo](http://wiki.ros.org/indigo/Installation)

### Install Gazeebo

You need 2 version of Gazeebo. [See website](http://gazebosim.org/download).
It's quite old enough, but you can try with new versions

### Install Tum simulator and Ardrone Autonomy

Just [follow instructions here](https://github.com/dougvk/tum_simulator)
Ardrone Autonomy is a driver for AR Drone
Tum simulator providing simulation with Gazeebo

At this point, you should be able to run:

`roslaunch cvg_sim_gazebo ardrone_testworld.launch`

And see something like this:
![alt text](http://wiki.coins-lab.org/images/4/44/Ardrone_tutorial_2.png "AR Drone Gazeebo")

But, during gazeebo install, by some reasons, not all of Gazeebo models are downloaded. To fix this problem see [this link](http://machineawakening.blogspot.ru/2015/05/how-to-download-all-gazebo-models.html)

Now, it's will be better.

You've installed all the packages, now it's time to make some code.
## Create your [package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)

## Run
Code your program. Actually, `src` directory here is code src from my package.
Don't forget to:

`chmod +x your_program.py` 

To make it executable.

After that, in you workspace:

`source devel/setup.bash`

And you are able to run:

`rosrun your_package_name your_program_name`

To see it all together launch Gazeebo at first:

`roslaunch cvg_sim_gazebo ardrone_testworld.launch`

In second Terminal window launch:

`rosrun your_package_name your_program_name`

For example, you can take forward.py script from [Roy](https://github.com/amroygaol/AR_Drone_Example_code)
