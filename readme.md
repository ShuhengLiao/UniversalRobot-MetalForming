## Sheet metal forming using UR5

Author: Vishwajeet Karmarkar <br>
Email : vishwajeet@u.northwestern.edu 


### What to expect : <br>

1) The robot will first move in front of the workpiece. It should then follow the trajectory from the csv file. Post that it should come back to the home 
position. Please note that there is no active sensing and if the start position of the trolley is not correct, the arm could hit a pillar of the cnc machine.
Please keep a hand on the emergency stop at all times!  <br>

    In between each task you will have to press enter on the terminal window to move onto the next task

2) Tuning : Inside the 'metal forming/config' folder change velocity scaling value to change end-effector speed

3) Trajectory : change file_loc to point where the csv file is ( hint: pwd command will show current folder)


### Installation on a new system : <br>

1) Install Ubuntu in a virtual machine or natively <br>
 
2) Install moveit software <br>

```
sudo apt install ros-melodic-moveit*
```

3) Install the UR Ros driver 

From your home directory
```
mkdir -p ur_drivers/src && cd ur_drivers

git clone https://github.com/vishwajeet-NU/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

git clone -b calibration_devel https://github.com/vishwajeet-NU/universal_robot.git src/fmauch_universal_robot

sudo apt update -qq

rosdep update

rosdep install --from-paths src --ignore-src -y

catkin_make

echo "source /home/"ACCOUNT_NAME"/ur_drivers/devel/setup.bash" >> ~/.bashrc

```
4) Clone this repository and build the package

From your home directory:

```
mkdir -p dsif_forming/src && cd dsif_forming

cd src 

git clone https://github.com/vishwajeet-NU/UniversalRobot-MetalForming.git

cd ..

catkin_make

echo "source /home/"ACCOUNT_NAME"/dsif_forming/devel/setup.bash" >> ~/.bashrc
```

### Setting up external control on Ur5

coming soon....


### Running on the Ur5 

1) On the teach pendant go to 'Run Programs' 
2) Select and run program called 'external'



### Running metal forming : <br>



1) Open up 4 terminal windows (shortcut new window: cntrl + alt + T)

2) On window1 : <br>

```
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=129.105.69.208
```
Notes: If the 'external' program is running on the robot, a message should pop up on the terminal, stating 'robot ready to recieve commnads' <br>

Notes: If for some reason the robot IP changes ( change of network etc) make sure to check ip from 'setup robot' using the teach pendant. Change it in the above command. 

Notes: If using virtual machine on the lab computer make sure to turn on anti-virus firewall protection. If not then connection will not be established

3) On window2 <br>

```
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch
```

4) Optional: If you want to see robot visualization on the computer  <br>
On window3 <br>

```
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```

5) On Window4 (or 3 if you skipped visualization from point 4) <br>

```
roslaunch metal_forming forming.launch
```
Press enter to move between each task

### Troubleshooting: 

1) Firewall is not turned off on windows if using virtual machine 
2) Ip address of computer or robot is not correct 
3) Incorrect path of csv file in dsif.yaml file ( metal_forming/config/dsif.yaml)
4) Account_name is incorrect ( If using the virtual machine I installed, account name is dsif)

