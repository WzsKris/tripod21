#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

from std_msgs.msg import Header
from std_msgs.msg import Float64
import rospy
import scipy.io
import numpy

# Load the MATLAB matrix
mat_data_leg1= scipy.io.loadmat('leg1_file.mat')  # Replace with the actual path to your .mat file
mat_data_leg2= scipy.io.loadmat('leg2_file.mat')  # Replace with the actual path to your .mat file
mat_data_leg3= scipy.io.loadmat('leg3_file.mat')  # Replace with the actual path to your .mat file

# Assume the .mat file contains a matrix called 'joint_positions' (3x1981)
joint_positions_leg1 = mat_data_leg1['theta_from_sol1']  # Shape: (3, 1981)
joint_positions_leg2 = mat_data_leg2['theta_from_sol2']  # Shape: (3, 1981)
joint_positions_leg3 = mat_data_leg3['theta_from_sol3']  # Shape: (3, 1981)
joint_for_index=joint_positions_leg1[:2]
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
    
    leg_min, leg_max = 0.0642803, 0.119915 # standing position 0
    high_min, high_max = -0.978129, -0.570255 #-1.0 --- -0.6
    low_min, low_max = -1.52033, -1.50083  #-1.2  ---  -0.8 ####-0.2 --- -1.0  -0.6, -1.4

    for i in range(3):
        leg_positions[i].data = leg_min
        high_positions[i].data = high_min
        low_positions[i].data = low_min

    speed_time=50 # Hert
    rate = rospy.Rate(speed_time)
    direction = 1  # 1 for increasing, -1 for decreasing
    num=0
    while not rospy.is_shutdown():
        
        for i in range(3):
            pubList[i * 3].publish(leg_positions[i])
            pubList[i * 3 + 1].publish(high_positions[i])
            pubList[i * 3 + 2].publish(low_positions[i])

        # Update high and low positions
        
        leg_positions[0].data = joint_positions_leg1[0][num]
        leg_positions[1].data = joint_positions_leg2[0][num]
        # leg_positions[2].data = joint_positions_leg3[0][num]

        high_positions[0].data = joint_positions_leg1[1][num]
        high_positions[1].data = joint_positions_leg2[1][num]
        # high_positions[2].data = -joint_positions_leg3[1][num]

        low_positions[0].data = joint_positions_leg1[2][num]
        low_positions[1].data = joint_positions_leg2[2][num]
        # low_positions[2].data = -joint_positions_leg3[2][num]
        
        leg_positions[2].data = -20*3.14/180
        high_positions[2].data = 45*3.14/180
        low_positions[2].data = -90*3.14/180

        # num=num+1

        if direction > 0:
            num=num+1
        # Check if we need to change direction
        elif direction < 0:
            num=num-1

        if num>=len(joint_positions_leg1[0,:])-1:
            direction=-1
        elif num<=0:
            direction=1
        rate.sleep()

        #reset
        # if num>=len(joint_positions_leg1[0,:])-1:
        #     num=0
        # rate.sleep()

        print('Published joint positions:')
        for i in range(3):
            print(f'Leg {i + 1}: {leg_positions[i].data}, High {i + 1}: {high_positions[i].data}, Low {i + 1}: {low_positions[i].data}')
            print(num)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")
