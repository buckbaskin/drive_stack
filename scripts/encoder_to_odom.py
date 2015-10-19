"""
ENCODER TO ODOM

ROS Node for converting encoder ticks to a wheel odometry measurement of
position.
"""

import rospy
from tf import transformations as tft
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from snowmower_msgs.msg import EncMsg

def heading_to_quaternion(heading):
    """
    Converts a Euler yaw/heading angle to equivalent quaternion
    input: euler heading in radians
    output: nav_msgs.msg.Quaternion
    """

    quat = tft.quaternion_from_euler(0, 0, heading)

    quaternion = Quaternion()
    quaternion.x = quat[0]
    quaternion.y = quat[1]
    quaternion.z = quat[2]
    quaternion.w = quat[3]
    return quaternion

class WheelOdometryGenerator(object):
    """
    WheelOdometryGenerator: ROS Node that implements encoder ticks to position

    wog = WheelOdometryGenerator()
    wog.start_node()
    """

    # pylint: disable=too-many-instance-attributes
    # These attributes represent state, last data, and pub/sub

    # pylint: disable=invalid-name
    # The variables for state are valid as x, y, v (linear velocity)

    track_width = 1
    wheel_diam = .333
    encoders_per_rev = 100

    def __init__(self):
        self.x = 0
        self.y = 0
        self.heading = 0
        self.v = 0
        self.omega = 0

        self.last_enc = None

        self.seq = 1

        self.sub = None
        self.pub = None

    def process_encoder_msg(self, msg):
        """
        process_encoder_msg: callback to take in new data, and update state

        input: snowmower_msgs.msg.EncMsg
        output: None
        """

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
                r = ((self.track_width/2)*
                    ((arc_right+arc_left)/(arc_right-arc_left)))
                theta = arc_left/(r-(self.track_width/2))
            else: # arc_left > arc_right:
                r = ((self.track_width/2)*
                    ((arc_right+arc_left)/(arc_left-arc_right)))
                theta = arc_left/(r+(self.track_width/2))
            v = ((dleft+dright)/2)/dt
            omega = theta/dt

        # based on v, omega, update state
        self.update_state(v, omega, dt)

        self.send_current_odom()
        self.last_enc = msg

    def send_current_odom(self):
        """
        send_current_odom: publish the current state as an Odometry message

        input: None
        output: (ROS publish) nav_msgs.msg.Odometry
        """

        o = Odometry()
        o.header.seq = self.seq
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = '/odom'

        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y

        o.pose.pose.orientation = heading_to_quaternion(self.heading)

        o.twist.twist.linear.x = self.v
        o.twist.twist.angular.z = self.omega

        self.seq += 1

        self.pub.publish(o)

    def start_node(self):
        """
        start_node: Run the ROS node by intializing the publisher and subscriber
        """
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.sub = rospy.Subscriber('/enc', EncMsg, self.process_encoder_msg)

    # GEOMETRY

    def distance(self, ticks):
        """
        distance: calculate the distance traveled per given ticks based on given
            robot geometry
        """
        return ticks*1.0/self.encoders_per_rev*math.pi*self.wheel_diam

    def update_state(self, avg_v, avg_omega, dt):
        """
        update_state: move the state forward one step

        state has:
        initial x, y, heading, linear velocity, angular velocity

        input:
        average linear velocity, average angular velocity and change in time

        output:
        None
        (modifies instance state based on calculated final x, y,
            linear velocity, angular velocity)
        """
        # input is observed average velocity (distance / time), angular velocity
        # calculates the linear value from start v (current state) to end v (new
        # state) based on this average.

        dr = avg_v*dt # distance traveled forward
        dtheta = avg_omega*dt # angle swept

        average_heading = self.heading + avg_omega/2.0

        dy = dr * math.sin(average_heading) # distance traveled in x, y
        dx = dr * math.cos(average_heading)

        end_v = avg_v + (avg_v - self.v) # final linear, angular velocity
        end_omega = avg_omega + (avg_omega - self.omega)

        # assign new values to update state
        self.x += dx
        self.y += dy
        self.heading += dtheta
        self.v = end_v
        self.omega = end_omega

if __name__ == '__main__':
    wog = WheelOdometryGenerator()
    wog.start_node()