import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import csv
import time



LINEAR_VEL = 0.05
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX=150
LEFT_SIDE_INDEX=90



class RandomWalk(Node):

    def __init__(self):
        super().__init__('random_walk_node')
        self.scan_cleaned = []
        self.stall = False
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.laser_forward = 0
        timer_period = 0.5
        self.pose_saved=''
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback_Kaden2)
        #Check right above this, this is where you call your code
        # Get the current working directory, I needed to know where "path.csv" was
        #current_directory = os.getcwd()
        #self.get_logger().info(f'Current working directory: {current_directory}')


    def listener_callback1(self, msg1):
        #self.get_logger().info('scan: "%s"' % msg1.ranges)
        scan = msg1.ranges
        self.scan_cleaned = []
    
        #self.get_logger().info('scan: "%s"' % scan)
        # Assume 360 range measurements
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
                self.scan_cleaned.append(reading)

    def stop(self):  
        print("I SHOULD BE STOPPING!!!!")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        self.turnCCW()
    time.sleep(1)


    def movingForward(self, front_lidar_min):    
        if front_lidar_min > SAFE_STOP_DISTANCE:
            self.cmd.linear.x = LINEAR_VEL  
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)

        # Stop the robot when the front distance from the obstacle is smaller than 1.0  
        if front_lidar_min < SAFE_STOP_DISTANCE:
            self.stop()
        time.sleep(1)

    def turnCCW(self):
        start_time = time.time()
        while time.time() - start_time < 2:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = -(math.pi)/8
            self.publisher_.publish(self.cmd)
        time.sleep(1)
        self.cmd.linear.x = LINEAR_VEL
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        time.sleep(1)

    def timer_callback_Kaden2(self):
        if len(self.scan_cleaned) == 0:
            self.turtlebot_moving = False
            return
        
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])

        print(front_lidar_min)
        self.movingForward(front_lidar_min)





def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    random_walk_node = RandomWalk()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(random_walk_node)
    # Explicity destroy the node
    random_walk_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()



if __name__ == '__main__':
    main()
