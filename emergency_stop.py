#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import Bool

class EmergencyStopNode:
    def __init__(self):
        # TEMPORARY FIX, OTHER TOPIC NOT WORKING
        # self.emergency_stop_sub = rospy.Subscriber("emergency_stop", Bool, self.emergency_stop_callback)
        self.emergency_stop_sub = rospy.Subscriber("robotnik_base_hw/emergency_stop_by_input", Bool, self.emergency_stop_callback)
        self.status = False
        

    def emergency_stop_callback(self, msg):
        # TEMPORARY FIX, OTHER TOPIC NOT WORKING
        # if msg.data == True: TEMPORARY FIX, OTHER TOPIC NOT WORKING
        if msg.data == False:
            rospy.logerr(rospy.get_caller_id() + "Emergency stop button pressed. Exit...")
            #self.status = True
            rospy.signal_shutdown("Emergency stop button pressed")
            
            

if __name__ == '__main__':
    rospy.init_node('emergency_stop', anonymous=True, disable_signals=True)
    
    emergency_stop_node = EmergencyStopNode()

    rospy.spin()
    
    
    # rate = rospy.Rate(500)
    # while not rospy.is_shutdown() and not emergency_stop_node.status:
        
    #     rate.sleep()