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
import time




LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX=150
LEFT_SIDE_INDEX=90

#These are all variables I created. Not all of them are in use now, but they were at some point
DISTANCE = 0
DISTANCE_OBJECTIVE = 0
DEGREES = 0
DEGREES_OBJECTIVE = 0
USER_INTEGER = 100
LAST_POS_X = 0
LAST_POS_Y = 0


class RandomWalk(Node):

    def __init__(self):
        super().__init__('random_walk_node')
        self.scan_cleaned = []
        self.stall = False
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        #self.subscriber1 = self.create_subscription(LaserScan,'/scan',self.listener_callback1,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.start_orientation = None
        self.laser_forward = 0
        self.odom_data = 0
        timer_period = 0.5
        self.pose_saved = {'position': (0, 0, 0), 'orientation': (0, 0, 0, 1)}
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback_Homework2)
        #Check right above this, this is where you call your code
        # Get the current working directory, I needed to know where "path.csv" was
        #current_directory = os.getcwd()
        #self.get_logger().info(f'Current working directory: {current_directory}')


    #def listener_callback1(self, msg1):
        #self.get_logger().info('scan: "%s"' % msg1.ranges)
        #scan = msg1.ranges
        #self.scan_cleaned = []
       
        #self.get_logger().info('scan: "%s"' % scan)
        # Assume 360 range measurements
        #for reading in scan:
        #    if reading == float('Inf'):
        #        self.scan_cleaned.append(3.5)
        #    elif math.isnan(reading):
        #        self.scan_cleaned.append(0.0)
        #    else:
        #    	self.scan_cleaned.append(reading)

    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        #self.get_logger().info('self position: {},{},{}'.format(position.x, position.y, position.z))
        #(posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        #self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz))
        # similarly for twist message if you need
        self.pose_saved = {'position': (position.x, position.y, position.z), 'orientation': (orientation.x, orientation.y, orientation.z, orientation.w)}
        
        #Example of how to identify a stall..need better tuned position deltas; wheels spin and example fast
        #diffX = math.fabs(self.pose_saved.x- position.x)
        #diffY = math.fabs(self.pose_saved.y - position.y)
        #if (diffX < 0.0001 and diffY < 0.0001):
           #self.stall = True
        #else:
           #self.stall = False
           
        return None
         

    def timer_callback_Homework2(self):
        global DISTANCE
        global DISTANCE_OBJECTIVE
        global DEGREES
        global DEGREES_OBJECTIVE
        global USER_INTEGER
        global LAST_POS_X
        global LAST_POS_Y
        
        #self.get_logger().info('DISTANCE: %s, DISTANCE_OBJECTIVE: %s' % (DISTANCE, DISTANCE_OBJECTIVE))
        
        if DISTANCE_OBJECTIVE > 0:
            if DISTANCE >= DISTANCE_OBJECTIVE:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0 
                self.publisher_.publish(self.cmd)
                self.get_logger().info('Distance: %s meter(s)' % DISTANCE)
                DISTANCE_OBJECTIVE = 0
                DISTANCE = 0
        if DEGREES_OBJECTIVE > 0:
            if DEGREES >= DEGREES_OBJECTIVE:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0 
                self.publisher_.publish(self.cmd)
                self.get_logger().info('Degrees: %s degrees' % DEGREES)
                DEGREES_OBJECTIVE = 0
                DEGREES = 0
                
        if DISTANCE_OBJECTIVE == 0 and DEGREES_OBJECTIVE == 0:
            LAST_POS_X = self.pose_saved['position'][0]
            LAST_POS_Y = self.pose_saved['position'][1]
            self.start_orientation = self.pose_saved['orientation']
            USER_INTEGER = 100
            USER_INTEGER = int(input("Enter an integer: "))
                

        #1 meter - 0.3 m/s
        if USER_INTEGER == 0:
            DISTANCE_OBJECTIVE = 1
            self.cmd.linear.x = 0.21
            self.cmd.angular.z = 0.0 
            self.publisher_.publish(self.cmd)
            DISTANCE = DISTANCE + math.sqrt(pow((self.pose_saved['position'][0] - LAST_POS_X), 2) + pow((self.pose_saved['position'][1] - LAST_POS_Y),2))
            LAST_POS_X = self.pose_saved['position'][0]
            LAST_POS_Y = self.pose_saved['position'][1]
            self.get_logger().info('Distance: %s' % DISTANCE)
        #5 meters - 0.3 m/s
        elif USER_INTEGER == 1:
            DISTANCE_OBJECTIVE = 5
            self.cmd.linear.x = 0.21
            self.cmd.angular.z = 0.0 
            self.publisher_.publish(self.cmd)
            DISTANCE = DISTANCE + math.sqrt(pow((self.pose_saved['position'][0] - LAST_POS_X), 2) + pow((self.pose_saved['position'][1] - LAST_POS_Y),2))
            LAST_POS_X = self.pose_saved['position'][0]
            LAST_POS_Y = self.pose_saved['position'][1]
            self.get_logger().info('Distance: %s' % DISTANCE)
        #10 degrees - 30 degree/s
        elif USER_INTEGER == 2:
            DEGREES_OBJECTIVE = 10
            #ChatGPT - Start
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = math.radians(30)
            self.publisher_.publish(self.cmd)
            current_orientation = self.pose_saved['orientation']
            delta_orientation = math.atan2(
                2.0 * (current_orientation[3] * current_orientation[2] + current_orientation[0] * current_orientation[1]),
                1.0 - 2.0 * (current_orientation[1] ** 2 + current_orientation[2] ** 2)
            ) - math.atan2(
                2.0 * (self.start_orientation[3] * self.start_orientation[2] + self.start_orientation[0] * self.start_orientation[1]),
                1.0 - 2.0 * (self.start_orientation[1] ** 2 + self.start_orientation[2] ** 2)
            )

            # Calculate degrees and ensure it's in the range [0, 360)
            degreesTurned = abs(math.degrees(delta_orientation) % 360)
            
            DEGREES = DEGREES + degreesTurned
            self.start_orientation = current_orientation

            self.get_logger().info('Degrees: %s' % DEGREES)

            if DEGREES >= DEGREES_OBJECTIVE:
                self.get_logger().info('Degrees: %s degrees' % DEGREES)
                DEGREES_OBJECTIVE = 0
                DEGREES = 0
            #ChatGPT - End
        #180 degrees - 30 degree/s
        elif USER_INTEGER == 3:
            DEGREES_OBJECTIVE = 180
            #Chat GPT - start                
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = math.radians(30)
            self.publisher_.publish(self.cmd)
            current_orientation = self.pose_saved['orientation']
            delta_orientation = math.atan2(
                2.0 * (current_orientation[3] * current_orientation[2] + current_orientation[0] * current_orientation[1]),
                1.0 - 2.0 * (current_orientation[1] ** 2 + current_orientation[2] ** 2)
            ) - math.atan2(
                2.0 * (self.start_orientation[3] * self.start_orientation[2] + self.start_orientation[0] * self.start_orientation[1]),
                1.0 - 2.0 * (self.start_orientation[1] ** 2 + self.start_orientation[2] ** 2)
            )

            # Calculate degrees and ensure it's in the range [0, 360)
            degreesTurned = abs(math.degrees(delta_orientation) % 360)

            
            DEGREES = DEGREES + degreesTurned
            self.start_orientation = current_orientation

            self.get_logger().info('Degrees: %s' % DEGREES)

            if DEGREES >= DEGREES_OBJECTIVE:
                self.get_logger().info('Degrees: %s degrees' % DEGREES)
                DEGREES_OBJECTIVE = 0
                DEGREES = 0
            #ChatGPT - End
        #360 degrees - 30 degree/s
        elif USER_INTEGER == 4:
            DEGREES_OBJECTIVE = 360
            #ChatGPT - Start
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = math.radians(30)
            self.publisher_.publish(self.cmd)
            current_orientation = self.pose_saved['orientation']
            delta_orientation = math.atan2(
                2.0 * (current_orientation[3] * current_orientation[2] + current_orientation[0] * current_orientation[1]),
                1.0 - 2.0 * (current_orientation[1] ** 2 + current_orientation[2] ** 2)
            ) - math.atan2(
                2.0 * (self.start_orientation[3] * self.start_orientation[2] + self.start_orientation[0] * self.start_orientation[1]),
                1.0 - 2.0 * (self.start_orientation[1] ** 2 + self.start_orientation[2] ** 2)
            )

            # Calculate degrees and ensure it's in the range [0, 360)
            degreesTurned = abs(math.degrees(delta_orientation) % 360)

            DEGREES = DEGREES + degreesTurned
            self.start_orientation = current_orientation

            self.get_logger().info('Degrees: %s' % DEGREES)

            if DEGREES >= DEGREES_OBJECTIVE:
                self.get_logger().info('Degrees: %s degrees' % DEGREES)
                DEGREES_OBJECTIVE = 0
                DEGREES = 0
            #ChatGPT - End
        #1 meter - 0.08 m/s
        elif USER_INTEGER == 5:
            DISTANCE_OBJECTIVE = 1
            self.cmd.linear.x = 0.08
            self.cmd.angular.z = 0.0 
            self.publisher_.publish(self.cmd)
            DISTANCE = DISTANCE + math.sqrt(pow((self.pose_saved['position'][0] - LAST_POS_X), 2) + pow((self.pose_saved['position'][1] - LAST_POS_Y),2))
            LAST_POS_X = self.pose_saved['position'][0]
            LAST_POS_Y = self.pose_saved['position'][1]
            self.get_logger().info('Distance: %s' % DISTANCE)
        #5 meters - 0.08 m/s
        elif USER_INTEGER == 6:
            DISTANCE_OBJECTIVE = 5
            self.cmd.linear.x = 0.08
            self.cmd.angular.z = 0.0 
            self.publisher_.publish(self.cmd)
            DISTANCE = DISTANCE + math.sqrt(pow((self.pose_saved['position'][0] - LAST_POS_X), 2) + pow((self.pose_saved['position'][1] - LAST_POS_Y),2))
            LAST_POS_X = self.pose_saved['position'][0]
            LAST_POS_Y = self.pose_saved['position'][1]
            self.get_logger().info('Distance: %s' % DISTANCE)
        #10 degrees - 120 degree/s
        elif USER_INTEGER == 7:
            DEGREES_OBJECTIVE = 10
            #ChatGPT - Start
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = math.radians(120)
            self.publisher_.publish(self.cmd)
            current_orientation = self.pose_saved['orientation']
            delta_orientation = math.atan2(
                2.0 * (current_orientation[3] * current_orientation[2] + current_orientation[0] * current_orientation[1]),
                1.0 - 2.0 * (current_orientation[1] ** 2 + current_orientation[2] ** 2)
            ) - math.atan2(
                2.0 * (self.start_orientation[3] * self.start_orientation[2] + self.start_orientation[0] * self.start_orientation[1]),
                1.0 - 2.0 * (self.start_orientation[1] ** 2 + self.start_orientation[2] ** 2)
            )

            # Calculate degrees and ensure it's in the range [0, 360)
            degreesTurned = abs(math.degrees(delta_orientation) % 360)
            
            DEGREES = DEGREES + degreesTurned
            self.start_orientation = current_orientation

            self.get_logger().info('Degrees: %s' % DEGREES)

            if DEGREES >= DEGREES_OBJECTIVE:
                self.get_logger().info('Degrees: %s degrees' % DEGREES)
                DEGREES_OBJECTIVE = 0
                DEGREES = 0
            #ChatGPT - End
        #180 degrees - 120 degree/s
        elif USER_INTEGER == 8:
            DEGREES_OBJECTIVE = 180
            #ChatGPT - Start
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = math.radians(120)
            self.publisher_.publish(self.cmd)
            current_orientation = self.pose_saved['orientation']
            delta_orientation = math.atan2(
                2.0 * (current_orientation[3] * current_orientation[2] + current_orientation[0] * current_orientation[1]),
                1.0 - 2.0 * (current_orientation[1] ** 2 + current_orientation[2] ** 2)
            ) - math.atan2(
                2.0 * (self.start_orientation[3] * self.start_orientation[2] + self.start_orientation[0] * self.start_orientation[1]),
                1.0 - 2.0 * (self.start_orientation[1] ** 2 + self.start_orientation[2] ** 2)
            )

            # Calculate degrees and ensure it's in the range [0, 360)
            degreesTurned = abs(math.degrees(delta_orientation) % 360)
            
            DEGREES = DEGREES + degreesTurned
            self.start_orientation = current_orientation

            self.get_logger().info('Degrees: %s' % DEGREES)

            if DEGREES >= DEGREES_OBJECTIVE:
                self.get_logger().info('Degrees: %s degrees' % DEGREES)
                DEGREES_OBJECTIVE = 0
                DEGREES = 0
            #ChatGPT - End
        #360 degrees - 120 degree/s
        elif USER_INTEGER == 9:
            DEGREES_OBJECTIVE = 360
            #ChatGPT - Start
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = math.radians(120)
            self.publisher_.publish(self.cmd)
            current_orientation = self.pose_saved['orientation']
            delta_orientation = math.atan2(
                2.0 * (current_orientation[3] * current_orientation[2] + current_orientation[0] * current_orientation[1]),
                1.0 - 2.0 * (current_orientation[1] ** 2 + current_orientation[2] ** 2)
            ) - math.atan2(
                2.0 * (self.start_orientation[3] * self.start_orientation[2] + self.start_orientation[0] * self.start_orientation[1]),
                1.0 - 2.0 * (self.start_orientation[1] ** 2 + self.start_orientation[2] ** 2)
            )

            # Calculate degrees and ensure it's in the range [0, 360)
            degreesTurned = abs(math.degrees(delta_orientation) % 360)
            
            DEGREES = DEGREES + degreesTurned
            self.start_orientation = current_orientation

            self.get_logger().info('Degrees: %s' % DEGREES)

            if DEGREES >= DEGREES_OBJECTIVE:
                self.get_logger().info('Degrees: %s degrees' % DEGREES)
                DEGREES_OBJECTIVE = 0
                DEGREES = 0
            #ChatGPT - End
            



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
