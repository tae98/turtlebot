#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import rospkg

# reads the file in forms of array
def converter(file):
    for i in range(len(file)):
        file[i] = file[i].split(',')

    return file

# operates the turtlebot3
def operate(order):
    # declaring object for class Twist
    msg = Twist()
    for i in range(len(order)):
        rate = rospy.Rate(2)

        msg.linear.x = float(order[i][0])
        msg.angular.z = float(order[i][1])

        rospy.loginfo(msg)
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    global pub
    # initializing and publisher
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('controller', anonymous=True)
    # read the txt file with linear and angular velocity in tuples
    ros_root = rospkg.RosPack()
    with open(ros_root.get_path('astarsubmission') + '/generatedPath/astarPath.txt', 'r') as command:
        orders = command.readlines()
    # convert tupels in to arrays and run them
    orders = converter(orders)
    while True:
        operate(orders)
	break
	rospy.on_shutdown(h) 
    # runnning infinte times
    # rospy.spin()
