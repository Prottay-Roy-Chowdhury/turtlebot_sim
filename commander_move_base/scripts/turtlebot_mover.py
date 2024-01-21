import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow, sqrt, atan2

class TurtlebotMover:
    def __init__(self):
        rospy.init_node('turtlebot_mover', anonymous=True)
        self.rate = rospy.Rate(1)  # 1 Hz, adjust as needed

        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.positions = [(1.0, 0.0, 0.0), (2.0, 1.0, 1.57), (0.0, 0.0, 0.0)]  # (x, y, theta)
        self.current_position = (0.5, 0.5, 0.1)

    def move_robot(self, target_position):
        while not rospy.is_shutdown():
            goal_x, goal_y, goal_theta = target_position

            # Calculate the distance and angle to the goal
            distance = sqrt(pow((goal_x - self.current_position[0]), 2) + pow((goal_y - self.current_position[1]), 2))
            angle = atan2(goal_y - self.current_position[1], goal_x - self.current_position[0])

            # Create a Twist message to send velocity commands
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = min(0.2, distance)  # Limit linear velocity
            cmd_vel_msg.angular.z = 0.3 * (angle - self.current_position[2])  # Adjust angular velocity

            # Publish the Twist message
            self.cmd_vel_pub.publish(cmd_vel_msg)
            
            # Check if the robot has reached the goal
            if distance < 0.1 and abs(angle - self.current_position[2]) < 0.1:
                rospy.loginfo("Reached the goal position")
                break

            self.rate.sleep()
    
    def odom_callback(self, odom_data):
        # Extract the robot's current position from odometry data
        self.current_position = (
            odom_data.pose.pose.position.x,
            odom_data.pose.pose.position.y,
            atan2(
                odom_data.pose.pose.orientation.z,
                odom_data.pose.pose.orientation.w
            )
        )
    
    def run(self):
        for target_position in self.positions:
            rospy.loginfo(f"Moving to position: {target_position}")
            self.move_robot(target_position)

if __name__ == '__main__':
    try:
        turtlebot_mover = TurtlebotMover()
        turtlebot_mover.run()
    except rospy.ROSInterruptException:
        pass