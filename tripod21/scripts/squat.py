#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

from std_msgs.msg import Header
from std_msgs.msg import Float64
import rospy

def main():
    rospy.init_node('limb')
    pubList = []
    
    pubList.append(rospy.Publisher('/leg01_position_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/high01_position_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/low01_position_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/leg02_position_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/high02_position_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/low02_position_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/leg03_position_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/high03_position_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/low03_position_controller/command', Float64, queue_size=9))

    # Initialize joint values
    leg_positions = [Float64() for _ in range(3)]
    high_positions = [Float64() for _ in range(3)]
    low_positions = [Float64() for _ in range(3)]
    
    leg = 0 # standing position 0
    high_min, high_max = -1.3, -0.1 #-1.0 --- -0.6
    low_min, low_max = -0.6, -1.2  #-1.2  ---  -0.8 ####-0.2 --- -1.0  -0.6, -1.4

    for i in range(3):
        leg_positions[i].data = leg 
        high_positions[i].data = high_min
        low_positions[i].data = low_min

    rate = rospy.Rate(10)
    direction = 1  # 1 for increasing, -1 for decreasing

    while not rospy.is_shutdown():
        for i in range(3):
            pubList[i * 3].publish(leg_positions[i])
            pubList[i * 3 + 1].publish(high_positions[i])
            pubList[i * 3 + 2].publish(low_positions[i])

        # Update high and low positions
        for i in range(3):
            high_positions[i].data += direction * 0.02
            low_positions[i].data += direction *1.5 * -0.02
        
        # Check if we need to change direction
        if low_positions[0].data <= low_max or low_positions[0].data >= low_min:
            direction *= -1

        rate.sleep()

        print('Published joint positions:')
        for i in range(3):
            print(f'Leg {i + 1}: {leg_positions[i].data}, High {i + 1}: {high_positions[i].data}, Low {i + 1}: {low_positions[i].data}')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")
