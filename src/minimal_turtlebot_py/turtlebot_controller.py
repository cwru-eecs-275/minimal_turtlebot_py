import rospy

global global_x, global_y
global_x = float('nan')
global_y = float('nan')

def turtlebot_controller(localTurtleBotInputs, localSoundValue, localLinearSpeed, localAngularSpeed):
    localLinearSpeed = 0.2
    localAngularSpeed = 0.2

    # Get the coordinates 
    x, y = GET_POSE()

    # Sends log once per second
    rospy.loginfo_throttle(1, "x: %2.2f; y %2.2f" % (x, y))
    # ROSPY log functions, all have throttle options
    # "debug" is not the level that should be used here
    # rospy.logdebug("x: %2.2f; y %2.2f" % (x, y))
    # rospy.loginfo("x: %2.2f; y %2.2f" % (x, y))
    # rospy.logwarn("x: %2.2f; y %2.2f" % (x, y))
    # rospy.logerr("x: %2.2f; y %2.2f" % (x, y))
    # rospy.logfatal("x: %2.2f; y %2.2f" % (x, y))
    
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