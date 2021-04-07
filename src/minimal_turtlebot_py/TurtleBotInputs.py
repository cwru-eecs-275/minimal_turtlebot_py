
from sensor_msgs.msg import Image

class TurtleBotInputs:
    """TurtleBot Inputs and Misc"""

    # time
    nanoSecs = 0;

    # wheel drop states
    leftWheelDropped = 0;
    rightWheelDropped = 0;

    # bumper states
    leftBumperPressed = 0;
    centerBumperPressed = 0;
    rightBumperPressed = 0;

    # color and depth images
    colorImage = Image();
    depthImage = Image();

    # cliff states
    sensor0State = 0;
    sensor1State = 0;
    sensor2State = 0;

    # laserscan data
    ranges = [];
    minAngle = 0.0;
    maxAngle = 0.0;
    angleIncrement = 0.0;
    numPoints = 0;

    # imu data
    linearAccelX = 0.0;
    linearAccelY = 0.0;
    linearAccelZ = 0.0;

    angularVelocityX = 0.0;
    angularVelocityY = 0.0;
    angularVelocityZ = 0.0;

    orientationX = 0.0;
    orientationY = 0.0;
    orientationZ = 0.0;
    orientationW = 1.0;

    # batt voltage
    battVoltage = 0.0;

    # odom
    x = 0.0;
    y = 0.0;
    z_angle = 0.0;
    orientation_omega = 0.0;
