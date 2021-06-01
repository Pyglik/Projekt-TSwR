import rospy
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# start symulacji robota
# roslaunch mir_gazebo mir_maze_world.launch

PI = 3.1415926535897
MIN_RANGE = 2.0  # odległość wykrycia przeszkody
ANGLE_OFFSET = 45  # ustawienie skanera laserowego względem robota
ANGLE_RANGE = [90, 30, -30, -90]  # zakresy wykrywania po lewej, na wprost i po prawej

class MirControler:
    def __init__(self):
        rospy.init_node('mir_controler')
        self.callbacks = Callbacks()
        self.vel = 1.0
        self.rot = 1.0
        self.turn_angle = PI/6
        self.start_angle = None
    
    def idle():
        self.callbacks.velocity = 0
        self.callbacks.rotation = 0
        self.callbacks.publish()
    
    def drive():
        self.callbacks.velocity = self.vel
        self.callbacks.rotation = 0
        self.callbacks.publish()
        self.start_angle = self.callbacks.pose_angle
    
    def turn_left():
        self.callbacks.velocity = self.vel
        self.callbacks.rotation = self.rot
        self.callbacks.publish()
    
    def turn_right():
        self.callbacks.velocity = self.vel
        self.callbacks.rotation = -self.rot
        self.callbacks.publish()
    
    def turn_around():
        self.callbacks.velocity = 0
        self.callbacks.rotation = self.rot
        self.callbacks.publish()
    
    def turn_left_end():
        angle_diff = self.callbacks.pose_angle-self.start_angle
        if angle_diff < PI:
            angle_diff += 2*PI
        elif angle_diff > PI:
            angle_diff -= 2*PI
        
        if angle_diff >= self.turn_angle:
            return True
        else:
            return False
    
    def turn_right_end():
        angle_diff = self.callbacks.pose_angle-self.start_angle
        if angle_diff < PI:
            angle_diff += 2*PI
        elif angle_diff > PI:
            angle_diff -= 2*PI
        
        if angle_diff <= self.turn_angle:
            return True
        else:
            return False
    
    def turn_around_end():
        angle_diff = self.callbacks.pose_angle-self.start_angle
        if angle_diff < PI:
            angle_diff += 2*PI
        elif angle_diff > PI:
            angle_diff -= 2*PI
        
        if abs(angle_diff) >= 0.99*PI:
            return True
        else:
            return False
    
    def obstacle()
        return self.callbacks.detection[1]
    
    def obstacle_r()
        return not self.callbacks.detection[0] and self.callbacks.detection[2]
    
    def obstacle_lf()
        return not self.callbacks.detection[2]
    
    def obstacle_lr()
        return self.callbacks.detection[0] and self.callbacks.detection[2]
    

class Callbacks:
    def __init__(self):
        self.pose_angle = None
        self.angle_min = None
        self.angle_max = None
        self.angle_increment = None
        self.scan = None
        self.detection = None
        self.velocity = None
        self.rotation = None
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.state_callback)
        rospy.Subscriber("/f_scan", LaserScan, self.scan_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    def state_callback(self, data):
        i = data.name.index('mir')
        orientation_q = data.pose[i].orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.pose_angle = yaw

    def scan_callback(self, data):
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angle_increment = data.angle_increment
        self.scan = data.ranges
        
        self.detection = [False, False, False]
        for i, range in enumerate(self.scan):
            if range < MIN_RANGE:
                angle_rad = self.angle_min+i*self.angle_increment
                angle = angle_rad*180/PI+ANGLE_OFFSET
                
                if ANGLE_RANGE[0] > angle > ANGLE_RANGE[1]:
                    self.detection[0] = True
                elif ANGLE_RANGE[1] > angle > ANGLE_RANGE[2]:
                    self.detection[1] = True
                elif ANGLE_RANGE[2] > angle > ANGLE_RANGE[3]:
                    self.detection[2] = True

    def publish(self):
        if self.velocity and self.rotation:
            move_cmd = Twist()
            move_cmd.linear.x = self.velocity
            move_cmd.angular.z = self.rotation
            
            self.cmd_pub.publish(move_cmd)

