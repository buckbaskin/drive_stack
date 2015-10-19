import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from snowmower_msgs.msg import EncMsg

def heading_to_quaternion(heading):
    q = Quaternion()
    return q

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
        # time is in secs
        dt = (msg.header.stamp.secs - self.last_enc.header.stamp.secs)*1.0
        if dt < 0:
            dt = 0.0

        if dleft == dright:
            v = self.distance(dleft) / dt
            omega = 0
        else:
            arc_left = self.distance(dleft)
            arc_right = self.distance(dright)

            if arc_left < arc_right:
                r = (self.track_width/2)*((arc_right+arc_left)/(arc_right-arc_left))
                theta = arc_left/(r-(self.track_width/2))
            else: # arc_left > arc_right:
                r = (self.track_width/2)*((arc_right+arc_left)/(arc_left-arc_right))
                theta = arc_left/(r+(self.track_width/2))
            v = ((dl+dr)/2)/dt
            omega = theta/dt

        # based on v, omega, update state
        self.update_state(v, omega, dt)

        self.send_current_odom()
        self.last_enc = msg

    def send_current_odom(self):
        o = Odometry()
        o.header.seq = self.seq
        o.header.stamp = rospy.Time.now()
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

    def update_state(self, avg_v, avg_omega, dt):
        # input is observed average velocity (distance / time), angular velocity
        # calculates the linear value from start v (current state) to end v (new
        # state) based on this average.

        dr = avg_v*dt
        dtheta = avg_omega*dt

        average_heading = self.heading + avg_omega/2.0

        dy = dr * math.sin(average_heading)
        dx = dr * math.cos(average_heading)
        end_v = avg_v + (avg_v - self.v)
        end_omega = avg_omega + (avg_omega - self.omega)

        self.x += dx
        self.y += dy
        self.heading += dtheta
        self.v = end_v
        self.omega = end_omega