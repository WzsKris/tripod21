 #!/usr/bin/python
#
# Send joint values to UR5 using messages
#

from std_msgs.msg import Header
from std_msgs.msg import Float64
import rospy

def main():

    rospy.init_node('limb')
    pubList=[]
    
    pubList.append(rospy.Publisher('/leg01_position_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/high01_position_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/low01_position_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/leg02_position_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/high02_position_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/low02_position_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/leg03_position_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/high03_position_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/low03_position_controller/command', Float64, queue_size=9))



    # Create the topic message
    pi=3.14
    jointValList=[]

    for i in range(9):
        jointValList.append(Float64())
       # jointValList[i].data=0
        jointValList[i].data = (pi / 2)


    jointVal=Float64()
    jointVal.data= 0
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        for j in range(9):
            pubList[j].publish(jointValList[j])


        #pub.publish(jointVal)
        rate.sleep() #similar to rospy.sleep(1)

        print('--------')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
