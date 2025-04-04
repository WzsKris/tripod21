#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

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
    
    leg_min, leg_max = -0.01, 0.01
    high_min, high_max = -1.2, -0.8
    low_min, low_max = -0.6, -0.61

    for i in range(3):
        leg_positions[i].data = leg_min
        high_positions[i].data = high_min
        low_positions[i].data = low_min

    rate = rospy.Rate(10)
    direction = 1  # 1 for increasing, -1 for decreasing
    current_leg = 0  # Start with leg 1

    while not rospy.is_shutdown():
        for i in range(3):
            pubList[i * 3].publish(leg_positions[i])
            pubList[i * 3 + 1].publish(high_positions[i])
            pubList[i * 3 + 2].publish(low_positions[i])

        # Update high, low, and leg positions for the current leg
        high_positions[current_leg].data += direction * 0.5
        low_positions[current_leg].data += direction * 1.5 * -0.01
        leg_positions[current_leg].data += direction * 0.01

        # Check if we need to change direction for high, low, or leg
        if (high_positions[current_leg].data >= high_max or high_positions[current_leg].data <= high_min or
            leg_positions[current_leg].data >= leg_max or leg_positions[current_leg].data <= leg_min):
            direction *= -1

        # Check if the cycle for the current leg is complete
        if direction == 1 and high_positions[current_leg].data >= high_min and high_positions[current_leg].data <= high_max:
            current_leg = (current_leg + 1) % 3  # Move to the next leg
            direction = 1  # Reset direction to increasing

        rate.sleep()

        print('Published joint positions:')
        for i in range(3):
            print(f'Leg {i + 1}: {leg_positions[i].data}, High {i + 1}: {high_positions[i].data}, Low {i + 1}: {low_positions[i].data}')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")
