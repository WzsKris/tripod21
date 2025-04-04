#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

from std_msgs.msg import Float64
import rospy

def main():
    rospy.init_node('limb')
    pubList = []

    # Define the joint position controllers
    pubList.append(rospy.Publisher('/leg01_position_controller/command', Float64, queue_size=10))
    pubList.append(rospy.Publisher('/high01_position_controller/command', Float64, queue_size=10))
    pubList.append(rospy.Publisher('/low01_position_controller/command', Float64, queue_size=10))
    pubList.append(rospy.Publisher('/leg02_position_controller/command', Float64, queue_size=10))
    pubList.append(rospy.Publisher('/high02_position_controller/command', Float64, queue_size=10))
    pubList.append(rospy.Publisher('/low02_position_controller/command', Float64, queue_size=10))
    pubList.append(rospy.Publisher('/leg03_position_controller/command', Float64, queue_size=10))
    pubList.append(rospy.Publisher('/high03_position_controller/command', Float64, queue_size=10))
    pubList.append(rospy.Publisher('/low03_position_controller/command', Float64, queue_size=10))
    
    # Define the target position for all joints
    target_position = Float64()
    target_position.data = 0.5  

    rate = rospy.Rate(1)  # Publish rate in Hz

    while not rospy.is_shutdown():
        for pub in pubList:
            pub.publish(target_position)

        rate.sleep()
        print('------')
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")
