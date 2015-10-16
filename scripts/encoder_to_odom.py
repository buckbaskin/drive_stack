import rospy
from nav_msgs.msg import Odometry
from snowmower_msgs.msg import EncMsg

class WheelOdometryGenerator(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.heading = 0
        self.v = 0
        self.omega = 0

        self.last_enc = None

        self.seq = 1

        self.track_width = 1
        self.wheel_diam = .333
        self.encoders_per_rev = 100

    def process_encoder_msg(self, msg):
        if self.last_enc is None:
            self.last_enc = msg
        dleft = msg.left - self.last_enc.left
        dright = msg.right - self.last_enc.right
        dt = .1 # TODO(buckbaskin): fix timing

        if dleft == dright:
            v = self.distance(dleft) / dt
            omega = 0
        else:
            arc_left = self.distance(dleft)
            arc_right = self.distance(dright)

            #TODO(buckbaskin): calc the arc radius to relate v, omega
            if arc_left < arc_right:
                r = (self.track_width/2)*((arc_right+arc_left)/(arc_right-arc_left))
                theta = arc_left/(r-(self.track_width/2))
            else: # arc_left > arc_right:
                r = (self.track_width/2)*((arc_right+arc_left)/(arc_left-arc_right))
                theta = arc_left/(r+(self.track_width/2))
            v = ((dl+dr)/2)/dt
            omega = theta/dt

        # based on v, omega, update state
        self.update_state(v, omega)

        self.send_current_odom()
        self.last_enc = msg

    def send_current_odom(self):
        o = Odometry()
        o.header.seq = self.seq
        o.header.stamp = rospy.time.now()
        o.header.frame_id = '/odom'

        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y

        o.orientation.orientation = self.heading_to_quaternion() # (TODO(buckbaskin): implement)

        o.twist.twist.linear.x = self.v
        o.twist.twist.angular.z = self.omega

        self.seq += 1

        self.pub.publish(o)

    def start_node(self, msg):
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.sub = rospy.Subscriber('/enc', EncMsg, self.process_encoder_msg)

    # GEOMETRY

    def distance(self, ticks):
        return ticks*1.0/self.encoders_per_rev*math.PI*self.wheel_diam

    def update_state(self, v, omega):
        # TODO(buckbaskin): do the state update thing
        pass