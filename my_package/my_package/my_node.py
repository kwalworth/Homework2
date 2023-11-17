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
DISTANCE_I_SET = 1.0
DISTANCE_I_SET2 = 1.0
DISTANCE_I_SET_LEFT = 0.4
FOUND_WALL = False
KEEP_TURNING = False
COUNT = 100
COUNT2 = 0
COUNT3 = 25
TURNED = False
RESET = False
FRONT = 0
RIGHT = 0
LEFT = 0

#Start of code from: https://www.theconstructsim.com/wall-follower-algorithm/
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'turn right',
    3: 'follow the wall',
    4: 'follow the wall drift',
    5: 'get away from the wall',
}
#End of code from: https://www.theconstructsim.com/wall-follower-algorithm/



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
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.laser_forward = 0
        self.odom_data = 0
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
    
    
    def listener_callback2(self, msg2):
        #These COUNT is to only do something every x amount of cycles. And TURNED is a value controlled by a different function
        global COUNT
        global TURNED
        global COUNT2
        
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        #self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz));
        #similarly for twist message if you need
        #self.pose_saved=position
        
        #Writing my position to a file. I had to space it out because my file was filling up before the trial was over
        if COUNT2 == 50:
            #Try catch and writing to csv from Chat GPT
            try:
                with open('path.csv', 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([position.x, position.y])
            except Exception as e:
                self.get_logger().info('An error occurred')
            COUNT2 = 0
        else:
            COUNT2 = COUNT2 + 1
        
        #self.get_logger().info('x="%s",y="%s"' % (position.x, position.y))
        #This code was in effort to identify and stop stalling. It ends up never actually doing anything though. I left it so I can see my thought process
        if TURNED == True:
            COUNT = 100
        else:
            if COUNT == 100:
                #self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz)); I commented this out
                # similarly for twist message if you need
                self.pose_saved=position
            if COUNT == 0:
                #self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz)); I commented this out
                # similarly for twist message if you need
                #self.pose_saved=position
                
                #Example of how to identify a stall..need better tuned position deltas; wheels spin and example fast
                diffX = math.fabs(self.pose_saved.x- position.x)
                diffY = math.fabs(self.pose_saved.y - position.y)
                #self.get_logger().info('Saved position: ("%s", "%s"), Current position: ("%s", "%s"), diffX: "%s", diffY: "%s"' % (self.pose_saved.x, self.pose_saved.y, position.x, position.y, diffX, diffY))
                #self.get_logger().info('We are stuck: "%s", "%s", "%s", front_min: "%s", left_min: "%s", right_min: "%s", rightfront_min: "%s"' % (self.stall, wall_found, state_description, front_lidar_min, left_lidar_min, right_lidar_min, rightfront_lidar_min))
                if (diffX < 0.01 and diffY < 0.01):
                    #self.get_logger().info('We are stuck')
                    self.stall = True
                else:
                    self.stall = False
                    #self.get_logger().info('We are not stuck')
                COUNT = 100
            else:
                COUNT = COUNT - 1
                
            #csv_file_name = "path.csv"
            #with open(csv_file_name, mode='w', newline='') as file:
                # Create a CSV writer
                #writer = csv.writer(file)
                #writer.writerow([0, 1])
        
        return None
   
        
    def timer_callback(self):
        if (len(self.scan_cleaned)==0):
    	    self.turtlebot_moving = False
    	    return
    	    
        #left_lidar_samples = self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX]
        #right_lidar_samples = self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX]
        #front_lidar_samples = self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX]
        
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])

        #self.get_logger().info('left scan slice: "%s"'%  min(left_lidar_samples))
        #self.get_logger().info('front scan slice: "%s"'%  min(front_lidar_samples))
        #self.get_logger().info('right scan slice: "%s"'%  min(right_lidar_samples))

        if front_lidar_min < SAFE_STOP_DISTANCE:
            if self.turtlebot_moving == True:
                self.cmd.linear.x = 0.0 
                self.cmd.angular.z = 0.0 
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = False
                self.get_logger().info('Stopping')
                return
        elif front_lidar_min < LIDAR_AVOID_DISTANCE:
                self.cmd.linear.x = 0.07 
                if (right_lidar_min > left_lidar_min):
                   self.cmd.angular.z = -0.3
                else:
                   self.cmd.angular.z = 0.3
                self.publisher_.publish(self.cmd)
                self.get_logger().info('Turning')
                self.turtlebot_moving = True
        else:
            self.cmd.linear.x = 0.3
            self.cmd.linear.z = 0.0
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
            

        self.get_logger().info('Distance of the obstacle : %f' % front_lidar_min)
        self.get_logger().info('I receive: "%s"' %
                               str(self.odom_data))
        if self.stall == True:
           self.get_logger().info('Stall reported')
        
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % self.cmd)
        
        
 

    def timer_callback_Kaden2(self):
        
        global FOUND_WALL
        global KEEP_TURNING
        global TURNED
        #global FRONT
        global RIGHT
        #global LEFT
        global COUNT3
        
        if(FOUND_WALL == True):
            wall_found = 'We found a wall'
        else:
            wall_found = "We have not found a wall"
        
        if (len(self.scan_cleaned)==0):
    	    self.turtlebot_moving = False
    	    return
        
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])
        rightfront_lidar_min = self.scan_cleaned[RIGHT_FRONT_INDEX]
        
        if COUNT3 == 25:
            FRONT = front_lidar_min
            RIGHT = right_lidar_min
            LEFT = left_lidar_min
        
        #If self.stall is true, I want it to run for a little before becoming false, thus COUNT3 == 20    
        if COUNT3 == 20 and self.stall == True:
            self.stall = False
        
        COUNT3 = COUNT3 - 1
        
        #The follow wall code I found was good for finding a wall, but not for following it. This is my code to follow the wall.
        if FOUND_WALL == True and right_lidar_min > DISTANCE_I_SET or KEEP_TURNING == True:
            state_description = 'case 0 - we need to turn right'
            state_ = 2
            KEEP_TURNING = True
            if KEEP_TURNING == True:
                if rightfront_lidar_min < DISTANCE_I_SET:
                    KEEP_TURNING = False
        elif FOUND_WALL == True and right_lidar_min < DISTANCE_I_SET - .2 and front_lidar_min > SAFE_STOP_DISTANCE:
            state_description = 'case 0.01 - we need to drift left'
            state_ = 5
        elif FOUND_WALL == True and right_lidar_min < DISTANCE_I_SET and front_lidar_min > SAFE_STOP_DISTANCE and rightfront_lidar_min >= DISTANCE_I_SET - 0.1:
            state_description = 'case 0.1 - we need to drift right'
            state_ = 4
        elif FOUND_WALL == True and right_lidar_min < DISTANCE_I_SET and front_lidar_min > SAFE_STOP_DISTANCE and rightfront_lidar_min < DISTANCE_I_SET - 0.1:
            state_description = 'case 0.1 - we need to drift left'
            state_ = 3
        elif FOUND_WALL == True and right_lidar_min < DISTANCE_I_SET and front_lidar_min < SAFE_STOP_DISTANCE:
            state_description = 'case 0.2 - we need to turn left'
            state_ = 1
        #Start of code from: https://www.theconstructsim.com/wall-follower-algorithm/
        elif front_lidar_min > DISTANCE_I_SET and left_lidar_min > DISTANCE_I_SET_LEFT and right_lidar_min > SAFE_STOP_DISTANCE:
            state_description = 'case 1 - nothing'
            state_ = 0
        elif front_lidar_min < DISTANCE_I_SET and left_lidar_min > DISTANCE_I_SET_LEFT and right_lidar_min > DISTANCE_I_SET:
            state_description = 'case 2 - front'
            state_ = 1
            FOUND_WALL = True
        elif front_lidar_min > DISTANCE_I_SET and left_lidar_min > DISTANCE_I_SET_LEFT and right_lidar_min < DISTANCE_I_SET:
            state_description = 'case 3 - fright'
            state_ = 3
            FOUND_WALL = True
        elif front_lidar_min > DISTANCE_I_SET and left_lidar_min < DISTANCE_I_SET_LEFT and right_lidar_min > DISTANCE_I_SET:
            state_description = 'case 4 - fleft'
            state_ = 0
        elif front_lidar_min < DISTANCE_I_SET and left_lidar_min > DISTANCE_I_SET_LEFT and right_lidar_min < DISTANCE_I_SET:
            state_description = 'case 5 - front and fright'
            state_ = 1
            FOUND_WALL = True
        elif front_lidar_min < DISTANCE_I_SET and left_lidar_min < DISTANCE_I_SET_LEFT and right_lidar_min > DISTANCE_I_SET:
            state_description = 'case 6 - front and fleft'
            state_ = 1
        elif front_lidar_min < DISTANCE_I_SET and left_lidar_min < DISTANCE_I_SET_LEFT and right_lidar_min < DISTANCE_I_SET:
            state_description = 'case 7 - front and fleft and fright'
            state_ = 1
            FOUND_WALL = True
        elif front_lidar_min > DISTANCE_I_SET and left_lidar_min < DISTANCE_I_SET_LEFT and right_lidar_min < DISTANCE_I_SET:
            state_description = 'case 8 - fleft and fright'
            state_ = 0
            FOUND_WALL = True
        else:
            state_description = 'unknown case'
            rospy.loginfo(regions)
        
        if state_ == 0 or state_ == 3 or state_ == 4:
            if TURNED == True:
                TURNED = False
        
        if state_ == 1:
            #turn left
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.2
            self.publisher_.publish(self.cmd)
            TURNED = True
        elif state_ == 2:
            #turn right
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = -0.2
            self.publisher_.publish(self.cmd)
            TURNED = True
        elif self.stall == True:
            #move backwards
            self.cmd.linear.x = -0.1 
            self.cmd.angular.z = 0.2
            self.publisher_.publish(self.cmd)
        elif state_ == 0:
            #find the wall
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = -0.2
            self.publisher_.publish(self.cmd)   
        elif state_ == 3:
            #follow wall drift left
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = 0.2 
            self.publisher_.publish(self.cmd)
        elif state_ == 4:
            #follow wall drift right
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = -0.2 
            self.publisher_.publish(self.cmd)
        elif state_ == 5:
            #Get away from wall
            self.cmd.linear.x = 0.05
            self.cmd.angular.z = 0.2 
            self.publisher_.publish(self.cmd)
        #End of code from: https://www.theconstructsim.com/wall-follower-algorithm/ although I added some things in there        
        if COUNT3 == 0:
            #diff_lidar_front = FRONT - front_lidar_min
            diff_lidar_right = RIGHT - right_lidar_min
            #diff_lidar_left = LEFT - left_lidar_min
            self.get_logger().info('right_min_diff: "%s"' % (abs(diff_lidar_right)))
            COUNT3 = 25
            if(abs(diff_lidar_right) < 0.01):
                self.stall = True




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
