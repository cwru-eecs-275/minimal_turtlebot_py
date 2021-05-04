import rospy

import angles
from kobuki_msgs.msg import Sound

global global_x, global_y
global_x = float('nan')
global_y = float('nan')

def turtlebot_controller(localTurtleBotInputs, localSoundValue, localLinearSpeed, localAngularSpeed):
    localLinearSpeed = 0.2
    localAngularSpeed = 0.2

    # Get the coordinates 
    x, y = GET_POSE()

	# Send a debug message throttled to 1 Hz.
    rospy.loginfo_throttle(1, "x: %2.2f; y %2.2f" % (x, y))
    # Send a debug message every loop.
    # rospy.loginfo_throttle("x: %2.2f; y %2.2f" % (x, y))
    
	# here are the various sound value enumeration options
	# Sound.SOUND_OFF
	# Sound.SOUND_RECHARGE
	# Sound.SOUND_BUTTON
	# Sound.SOUND_ERROR
	# Sound.SOUND_CLEANINGSTART
	# Sound.SOUND_CLEANINGEND 
	localSoundValue = Sound.SOUND_OFF

    return localTurtleBotInputs, localSoundValue, localLinearSpeed, localAngularSpeed

# In Python, it is customary to begin element names with underscores
# when they should not externally used.  This function is not for
# use within turtlebot_controller.
def _SET_GOAL(x, y):
    global global_x, global_y
    global_x = x
    global_y = y

def GET_GOAL():
    global global_x, global_y

    return global_x, global_y