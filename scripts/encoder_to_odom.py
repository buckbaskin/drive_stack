#!/usr/bin/env python

"""
ENCODER TO ODOM

ROS Node for converting encoder ticks to a wheel odometry measurement of
position.
"""

import rospy
from tf import transformations as tft
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion
from snowmower_msgs.msg import EncMsg

def heading_to_quaternion(heading):
    """
    Converts a Euler yaw/heading angle to equivalent quaternion
    input: euler heading in radians
    output: nav_msgs.msg.Quaternion
    """
    # rospy.loginfo('heading out: '+str(heading))
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

    track_width = 1.05
    wheel_diam = .3302
    encoders_per_rev = 10000

    tpm_right = 26500
    tpm_left = 27150

    debug = False

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
        rospy.loginfo('msg recieved!')

        if self.last_enc is None:
            self.last_enc = msg
        dleft = msg.left - self.last_enc.left
        dright = msg.right - self.last_enc.right
        
        # time is in secs
        rospy.loginfo('msg type: '+str(type(msg)))
        rospy.loginfo('last_enc type: '+str(type(msg)))
        dt = (msg.header.stamp.nsecs - self.last_enc.header.stamp.nsecs)*pow(10,-9)
        if dt < 0:
            dt = 0.0

        if dleft == dright:
            if dt<= 0.0:
                v = 0.0
            else:
                v = self.distance(dleft, "left") / dt
            omega = 0.0
            arc_left = 0.0
            arc_right = 0.0
        else:
            arc_left = self.distance(dleft, "left")
            arc_right = self.distance(dright, "right")

            if arc_left < arc_right:
                r = ((self.track_width/2.0)*
                    ((arc_right+arc_left)/(arc_right-arc_left)))
                try:
                    # if r = track_width/2.0, calculate with right arc
                    theta = arc_left/(r-(self.track_width/2.0))
                except:
                    theta = arc_right/(r+(self.track_width/2.0))
            else: # arc_left > arc_right:
                r = ((self.track_width/2.0)*
                    ((arc_right+arc_left)/(arc_left-arc_right)))
                try:
                    # if r = - track_width/2.0, calculate with right arc
                    theta = arc_left/(r+(self.track_width/2.0))
                except:
                    theta = arc_right/(r-(self.track_width/2.0))
            if dt <= 0.0:
                if self.debug:
                    rospy.loginfo('negative time v = 0')
                v = 0
                omega = 0
            else:
                if self.debug:
                    rospy.loginfo('positive time v = ((dl+dr)/2) / dt')
                v = ((arc_left+arc_right)/2.0)/dt
                omega = theta/dt

        # based on v, omega, update state
        if dleft and dright and arc_left and arc_right:
            if self.debug:
                rospy.loginfo('dleft: '+str(dleft)+' dright: '+str(dright)+' dt: '+str(dt))
                rospy.loginfo('aleft: '+str(arc_left)+' aright: '+str(arc_right)+' dt: '+str(dt))
        self.update_state(v, omega, dt)
        if self.debug:
            rospy.loginfo('x: '+str(self.x)+' y: '+str(self.y))

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
        rospy.init_node('encoder2odom')
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.pub_head = rospy.Publisher('/heading', Float64, queue_size=1)
        self.sub = rospy.Subscriber('/enc', EncMsg, self.process_encoder_msg)
        rospy.loginfo('enc2odom: wheel odometry ready')
        rospy.loginfo('track_width  | wheel_diam   | encoders/rev')
        rospy.loginfo(str(self.track_width)+'         | '+str(self.wheel_diam)+'       | '+str(self.encoders_per_rev))
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.send_current_odom()
            floa = Float64(data=self.heading)
            self.pub_head.publish(floa)
            rate.sleep()

    # GEOMETRY

    def distance(self, ticks, side):
        """
        distance: calculate the distance traveled per given ticks based on given
            robot geometry
        """
        if side == "right":
            return ticks*1.0/self.tpm_right
        else:
            return ticks*1.0/self.tpm_left
        # return (ticks*1.0/self.encoders_per_rev)*math.pi*self.wheel_diam

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
        if self.debug:
            rospy.loginfo('v: '+str(avg_v)+' w: '+str(avg_omega))
            rospy.loginfo('dr: '+str(dr)+' dtheta: '+str(dtheta))

        average_heading = self.heading + dtheta/2.0

        dy = dr * math.sin(average_heading) # distance traveled in x, y
        dx = dr * math.cos(average_heading)

        end_v = avg_v + (avg_v - self.v) # final linear, angular velocity
        end_omega = avg_omega + (avg_omega - self.omega)

        # assign new values to update state
        self.x += dx
        self.y += dy
        rospy.loginfo('dtheta: '+str(dtheta))
        self.heading += dtheta
        self.v = end_v
        self.omega = end_omega

if __name__ == '__main__':
    # pylint: disable=invalid-name
    # wog is a placeholder to just run the node
    wog = WheelOdometryGenerator()
    wog.start_node()
