#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import CliffEvent, Sound, SensorState, WheelDropEvent, BumperEvent

from TurtleBotInputs import TurtleBotInputs as turtlebotInputs

from turtlebot_controller import turtlebot_controller, _SET_GOAL

global localTurtleBotInputs, localSoundValue, localLinearSpeed, startUpTimer, localAngularSpeed, soundValueUpdateCounter, amcl_present

def poseCallback(pose):
    _SET_GOAL(pose.pose.position.x, pose.pose.position.y)

def wheelDropCallback(wheel_drop_holder):
    global localTurtleBotInputs, localSoundValue, localLinearSpeed, startUpTimer, localAngularSpeed, soundValueUpdateCounter, amcl_present


    if (wheel_data_holder.wheel == wheel_data_holder.LEFT):
	localTurtleBotInputs.leftWheelDropped = wheel_data_holder.state 
	ROS_INFO("left wheel dropped state is: %u",wheel_data_holder.state)

    if (wheel_data_holder.wheel == wheel_data_holder.RIGHT):
	localTurtleBotInputs.rightWheelDropped = wheel_data_holder.state
	ROS_INFO("right wheel dropped state is: %u",wheel_data_holder.state)


def bumperMessageCallback(bumper_data_holder):
    global localTurtleBotInputs, localSoundValue, localLinearSpeed, startUpTimer, localAngularSpeed, soundValueUpdateCounter, amcl_present

    if (bumper_data_holder.bumper == bumper_data_holder.LEFT):
	localTurtleBotInputs.leftBumperPressed = bumper_data_holder.state
	ROS_INFO("left bumper pressed state is: %u",bumper_data_holder.state)

    if (bumper_data_holder.bumper == bumper_data_holder.CENTER):
	localTurtleBotInputs.centerBumperPressed = bumper_data_holder.state
	ROS_INFO("center bumper pressed state is: %u",bumper_data_holder.state)
        
    if (bumper_data_holder.bumper == bumper_data_holder.RIGHT):
        localTurtleBotInputs.rightBumperPressed = bumper_data_holder.state 
        ROS_INFO("right bumper pressed state is: %u",bumper_data_holder.state)

    
def cliffCallback(cliff_data):
    global localTurtleBotInputs, localSoundValue, localLinearSpeed, startUpTimer, localAngularSpeed, soundValueUpdateCounter, amcl_present

    if (cliff_data.sensor == 0):
	localTurtleBotInputs.sensor0State = cliff_data.state
	ROS_INFO("cliff sensor 0 state is: %u", localTurtleBotInputs.sensor0State)
        
    elif (cliff_data.sensor == 1):
	localTurtleBotInputs.sensor1State = cliff_data.state
	ROS_INFO("cliff sensor 1 state is: %u", localTurtleBotInputs.sensor1State)
        
    elif (cliff_data.sensor == 2):
	localTurtleBotInputs.sensor2State = cliff_data.state
	ROS_INFO("cliff sensor 2 state is: %u", localTurtleBotInputs.sensor2State)
	


def scanCallback(scan_data):
    global localTurtleBotInputs, localSoundValue, localLinearSpeed, startUpTimer, localAngularSpeed, soundValueUpdateCounter, amcl_present

    # for(indx in range(0, scan_data.ranges.size()):
    #     localTurtleBotInputs.ranges[indx] = scan_data.ranges[indx]
	
    localTurtleBotInputs.ranges = scan_data.ranges
    localTurtleBotInputs.minAngle = scan_data.angle_min
    localTurtleBotInputs.maxAngle = scan_data.angle_max 
    localTurtleBotInputs.angleIncrement = scan_data.angle_increment
    # localTurtleBotInputs.numPoints = 640
    localTurtleBotInputs.numPoints = len(scan_data.ranges)


def imuCallback(imu_data):
    global localTurtleBotInputs, localSoundValue, localLinearSpeed, startUpTimer, localAngularSpeed, soundValueUpdateCounter, amcl_present

    localTurtleBotInputs.linearAccelX = imu_data.linear_acceleration.x
    localTurtleBotInputs.linearAccelY = imu_data.linear_acceleration.y 
    localTurtleBotInputs.linearAccelZ = imu_data.linear_acceleration.z 
    
    localTurtleBotInputs.angularVelocityX = imu_data.angular_velocity.x
    localTurtleBotInputs.angularVelocityY = imu_data.angular_velocity.y 
    localTurtleBotInputs.angularVelocityZ = imu_data.angular_velocity.z
    
    localTurtleBotInputs.orientationX = imu_data.orientation.x
    localTurtleBotInputs.orientationY = imu_data.orientation.y
    localTurtleBotInputs.orientationZ = imu_data.orientation.z

def coreCallback(sensor_state):
    global localTurtleBotInputs, localSoundValue, localLinearSpeed, startUpTimer, localAngularSpeed, soundValueUpdateCounter, amcl_present

    localTurtleBotInputs.battVoltage = sensor_state.battery
    localTurtleBotInputs.battVoltage = localTurtleBotInputs.battVoltage*0.1


def odomCallback(pose):
    global localTurtleBotInputs, localSoundValue, localLinearSpeed, startUpTimer, localAngularSpeed, soundValueUpdateCounter, amcl_present

    if (~amcl_present and (startUpTimer > 150)):
	localTurtleBotInputs.x = pose.pose.pose.position.x
	localTurtleBotInputs.y = pose.pose.pose.position.y
	localTurtleBotInputs.z_angle = pose.pose.pose.orientation.z
	localTurtleBotInputs.orientation_omega = pose.pose.pose.orientation.w

def amclCallback(pose):
    global localTurtleBotInputs, localSoundValue, localLinearSpeed, startUpTimer, localAngularSpeed, soundValueUpdateCounter, amcl_present

    amcl_present = 1
    localTurtleBotInputs.x = pose.pose.pose.position.x
    localTurtleBotInputs.y = pose.pose.pose.position.y
    localTurtleBotInputs.z_angle = pose.pose.pose.orientation.z
    localTurtleBotInputs.orientation_omega = pose.pose.pose.orientation.w
    

def main():
    global localTurtleBotInputs, localSoundValue, localLinearSpeed, startUpTimer, localAngularSpeed, soundValueUpdateCounter, amcl_present

    localTurtleBotInputs = turtlebotInputs()
    localSoundValue = 0
    localLinearSpeed = 0.0
    localAngularSpeed = 0.0
    soundValueUpdateCounter = 0
    amcl_present = 0

    startUpTimer = 0

    # Name this node
    nh = rospy.init_node('minimal_turtlebot_py')

    # Set a rate for the loop
    naptime = rospy.Rate(10) # 10hz

    my_wheel_drop_subscription = rospy.Subscriber('mobile_base/events/wheel_drop', WheelDropEvent, wheelDropCallback)
    my_bumper_subscription = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, bumperMessageCallback)
    my_cliff_subscription = rospy.Subscriber('mobile_base/events/cliff', CliffEvent, cliffCallback)
    my_imu_subscription = rospy.Subscriber('mobile_base/sensors/imu_data_raw', Imu, imuCallback)
    my_core_subscription = rospy.Subscriber('mobile_base/sensors/core', SensorState, coreCallback)
    my_odom_subscription = rospy.Subscriber('odom', Odometry, odomCallback)
    my_amcl_subscription = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, amclCallback)
    my_pose2d_subscription = rospy.Subscriber('goal_pose2d', PoseStamped, poseCallback)
    scanSubscription = rospy.Subscriber('scan', LaserScan, scanCallback)
    
    # Publish to the velocity topic
    cmd_vel_pub_ = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
    # Publish to the sound topic
    my_publisher_object = rospy.Publisher('mobile_base/commands/sound', Sound, queue_size=10)

    soundValue = Sound()
    base_cmd = Twist()
    
    # Main loop
    while not rospy.is_shutdown():
        localTurtleBotInputs.nanoSecs = rospy.Time.now().to_nsec()

        localTurtleBotInputs, localSoundValue, localLinearSpeed, localAngularSpeed = turtlebot_controller(localTurtleBotInputs, localSoundValue, localLinearSpeed, localAngularSpeed)

        soundValue.value = localSoundValue
        base_cmd.linear.x = localLinearSpeed
        base_cmd.angular.z = localAngularSpeed
        
        if (startUpTimer < 1000):
            startUpTimer = startUpTimer + 1
        naptime.sleep()

        cmd_vel_pub_.publish(base_cmd)
