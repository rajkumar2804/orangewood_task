# TASK : Visualise OW_ARMV3 in RVIZ
Rviz is a visualisation tool which is used to visuial the ROS application, So we have to import the URDF of **OW_ARMV3** in the rviz and implement a straight line trajectory or any random trajectory.  To achieve the task I have used Moveit setup assistant which generate the moveit configuration package of the arm. So the configuration package is all we need. But before using the configuration pakage I have visualised the arm using a simple launch file configuration. I have all the things  in detail below


# Software Prerequisites
ROS distribution I have used is ROS Melodic and the Operating System is Ubuntu 18.04 as it have more stability than the newer version of Ubuntu.

 - ## Moveit Installation
   Moveit  is a very popular Motion Planning Framework. It is a set of packages and tools used for Robotic Manipulation in ROS.Before moving forward make sure to install MoveIt! on your system.
   **Moveit**
    ` sudo apt install ros-melodic-moveit`
    
   **Moveit  ROS planning**
   ` sudo apt install ros-melodic-moveit-ros-planning`
   
     **ROS**
     ` sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers`





  

# ROS Workspace

I have created a ROS workspace **orangewood_ws** which have following directory and package hierarchy


# orangewoo_pkg
This ros package contain two launch file `robot_rviz.launch` and `robot.launch` . The `robot_rviz.launch` is used to only visualise the robot arm in the riviz without moveit and motion planning.  The `robot.launch` is used to launch the moveit config launch file which is named as `demo.launch` and a ros node `node_joint_angles.py` which set the two different joint angles one and after another one is straight line trajectory and other one is random trajectory path.
###  Issue Faced
While using the moveit setup assistant  to configure moveit pkg I faced an issue. I am unable to define pose    in moveit setup assistant. I am sharing the snap of the issue below

![Issue](/images/issue.png)

I can above GUI to define any pose of robot. But I am unable to drag the joint angle slider and input in the box. I found the issue on ROS Answers [here.](https://answers.ros.org/question/311838/i-have-a-problem-with-define-robot-poses-in-moveit-setup-assistant/) So, I made some changes in the urdf which resolves the issue.
Here is a part of urdf where one of the joint of robot arm is defined :
```xml
 <joint
    name="BJ"
    type="revolute">
    <origin
      xyz="0 0 0.1181"
      rpy="1.5708 0 0" />
    <parent
      link="base_link" />
    <child
      link="BS" />
    <axis
      xyz="0 1 0" />
    <limit
      lower="-2.1"
      upper="2.1"
      effort="0"
      velocity="0" />
    <safety_controller
      soft_upper="2"
      soft_lower="-2"
      k_velocity="0" />
  </joint>
```
I changed the effort and velocity value to **150** and **3.14** respectively, but this doesn't resolve the issue, what's wrong is that in the safety controller tag `soft_upper` and `soft_lower` limit is not defined correctly it should be `soft_upper_limit` and `soft_lower_limit`. After making these changes to the urdf the moveit configuration appears valid and everything worked properly.

## Launching robot_rviz.launch

Before launching the launch file make sure that joint_state_publisher_gui package is installed on the system to get gui window of joints so that we can interact with the joints of the robot. The following video shows the same, click on the image:

[![robot_rviz_launch](/images/thumbnail.png)](https://drive.google.com/file/d/16NpTzJ7Qa2VLTQBtPDbATi37RVX9BxeK/view?usp=sharing)

**Launch File** :
```xml
<?xml version="1.0"?>
<launch>    
    <param name="robot_description" command="cat '$(find OW_arm42v3)/urdf/OW_arm42v3.urdf'" />
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find orangewood_pkg)/config/OW_arm42v3_config.rviz"/>
    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
       <param name="use_gui" value="True"/>
    </node>
</launch>
```

##  Launching robot.launch

This launch file launch the moveit configured demo launch file and a ros node `node_joint_angles.py`  which execute two joints angles. The ros node has the following main function:
```python
def main():
    """Main Function"""
    
    #wait for rviz to up
    rospy.sleep(4)
    
    #moveit arm instance
    arm = ArmMoveit()
    
    #joint angle one
    lst_joint_angles_1 = [math.radians(0),
                          math.radians(-80),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]
    
    #joint angle two
    lst_joint_angles_2 = [math.radians(101),
                          math.radians(62),
                          math.radians(-9),
                          math.radians(0),
                          math.radians(-53),
                          math.radians(-101)]
    
    #set joint angle one
    arm.set_joint_angles(lst_joint_angles_1)
    
    #sleep for ten seconds
    rospy.sleep(10)
    
    #set joint angle two
    arm.set_joint_angles(lst_joint_angles_2)
    
    del arm 
  ```
  The node first wait for rviz to come up and then it create an instance of ArmMoveit class which is used to send the goal to the arm. There are two joints angle goal `lst_joint_angle_1` which follow a straight trajectory and after 10 secs  `lst_joint_angle_2` will be executed which follow a random  valid path trajcetory.
  
  The follwing video demonstarte the same:
  [![robot_rviz.launch](https://drive.google.com/file/d/1oXpeKVcIxBqgkrFaZ3cZJbGohG31deEz/view?usp=sharing)]( https://drive.google.com/file/d/1w5RedSN03IrzQ-8_mprYMg7Yw6f4GKvh/view?usp=sharing)

**Launch File** : 
```xml
<?xml version="1.0"?>
<launch>    
    <include file="$(find arm_moveit_config)/launch/demo.launch" />
    <node name="node_joint_angles" pkg="orangewood_pkg" type="node_joint_angles.py" output="screen"/> 
</launch>
```
# arm_moveit_config  

This ros package is generated using moveit setup assistant it have all the moveit and motion planning configuration. It is used  for the motion planning of the robot arm. 


# OW_arm42v3 
This is the given ros package which conatain the robot model description files and urdf that is used to import the urdf when only using `robot_rviz.launch`
