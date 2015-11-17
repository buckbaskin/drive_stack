#!/usr/bin/env python

import leader
import rospy
import math
from nav_msgs.msg import Odometry

from utils import heading_to_quaternion, quaternion_to_heading
from utils import calc_errors, dist, scale, minimize_angle
from utils import unit as to_unit_tuple

from tf import transformations as tft

def unit(function):
    def modded_function(*args, **varargs):
        val = function(*args, **varargs)
        return to_unit_tuple(val)
    return modded_function

# pylint: disable=no-self-use
# the class functions are used. I'm not sure why it's missing this.

class ForceLeader(leader.Leader):
    # methods to override:
    # generate_initial_path, generate_next_path

    def generate_initial_path(self):
        """
        Path creation for node
        """
        end = self.path_goal().goal
        start = self.path_start().goal
        self.common_path_gen(start, end)

    def generate_next_path(self):
        end = self.path_next().goal
        start = self.path_start().goal
        self.common_path_gen(start, end)

    def common_path_gen(self, start, end):
        """
        generate a new path, either forwards or backwards (rvs == True)
        """
        end = self.path_next().goal
        start = self.path_start().goal
        
        self.targets = []
        self.targets.append(start)

        # pylint: disable=invalid-name
        # dt, v, w, are accurately describing what I want in this case

        dt = .1



        next_ = start

        errors = calc_errors(next_, end)
        along = errors[0]

        count = 2

        while along < -0.01 and not rospy.is_shutdown():
            if count > 200:
                rospy.loginfo('path overflow')
                break
            current = StateModel(next_)
            force_vector = self.get_force_vector(start, end, next_)
            force_heading = math.atan2(force_vector[1], force_vector[0])
            rospy.loginfo('fhead: '+str(force_heading)[0:5])
            heading_err = minimize_angle(force_heading - current.theta)
            rospy.loginfo('headerr: '+str(heading_err)[0:5])

            # pylint: disable=invalid-name
            # v, w, are accurately describing what I want in this case
            w = heading_err/dt*0.75
            v = 0.55        
            # TODO(buckbaskin): calculate something smart based on vel
            # profile from start to end
            # possibly do vel profile based on distance, slow down proportional to
            #  heading error relative to force field

            # rospy.loginfo('gnxt: '+str(force_heading)+' ... '+str(current.theta))

            count += 1
            next_ = current.sample_motion_model2(v, w, dt)

            odom_next = self.convert_to_odom(next_)

            self.targets.append(next_)

            

            errors = calc_errors(next_, end)
            along = errors[0]
            # rospy.loginfo('alongher alenghi alphabet: '+str(along))

        self.index = 0



    @unit # forces a unit vector to be returned, scaled by weight
    def get_force_vector(self, start, end, current):
        """
        Aggregate the sum of the force vectors for 3 different actors: departing
        the original location, arriving at the destination and traversing
        between the two points.

        There may be more added, and the end output is
        only relevant for its direction, so the weighting is arbitrary and
        relative, where I've fixed the traverse weight to always be 1.

        This is a superposition of 3? differential equations representing a sum
        of forces.
        """
        wdep = self.weighted_depart(start, end, current)
        warr = self.weighted_arrive(start, end, current)
        wtrv = self.weighted_traverse(start, end, current)
        # wobs = self.weighted_obstacle(start, end, current)

        force = (wdep[0]+warr[0]+wtrv[0], wdep[1]+warr[1]+wtrv[1], 0,)
        rospy.loginfo('final: '+str(force[0])[0:5]+' '+str(force[1])[0:5]+' '+str(force[2])[0:5])
        return force

    def weighted_depart(self, start, end, current):
        depart_vector = self.depart_vector(start, end, current)
        w = self.depart_weight(start, end, current)
        rospy.loginfo('wdp: '+str(depart_vector[0])[0:4]+' , '+str(depart_vector[1])[0:4]+' , '+str(w)[0:4])
        return scale(depart_vector, w)

    # pylint: disable=unused-argument
    # it is being left in to maintain method signature consistency
    @unit # forces a unit vector to be returned, scaled by weight
    def depart_vector(self, start, unused_end, current):
        # axis direction
        axis_direction = quaternion_to_heading(start.pose.pose.orientation)

        # correction to move away from axis
        errors = calc_errors(current, start)
        off_axis = errors[1]
        heading_correction = math.atan(1.5*off_axis)

        final_direction = axis_direction+heading_correction

        return (math.cos(final_direction), math.sin(final_direction), 0,)

    # pylint: disable=unused-argument
    # it is being left in to maintain method signature consistency
    def depart_weight(self, start, unused_end, current):
        d = dist(start, current)
        if d < .3:
            d = .3
        return 2.0/d

    def weighted_arrive(self, start, end, current):
        arrive_vector = self.arrive_vector(start, end, current)
        w = self.arrive_weight(start, end, current)
        rospy.loginfo('war: '+str(arrive_vector[0])[0:4]+' , '+str(arrive_vector[1])[0:4]+' , '+str(w)[0:4])
        return scale(arrive_vector, w)

    # pylint: disable=unused-argument
    # it is being left in to maintain method signature consistency
    @unit # forces a unit vector to be returned, scaled by weight
    def arrive_vector(self, unused_start, end, current):
        # axis direction
        axis_direction = minimize_angle(quaternion_to_heading(end.pose.pose.orientation))

        # correction to move away from axis
        errors = calc_errors(current, end)
        off_axis = errors[1]
        heading_correction = minimize_angle(math.atan(-1.5*off_axis))

        final_direction = minimize_angle(axis_direction+heading_correction)

        rospy.loginfo('avr: axis '+str(axis_direction)[0:4]+' corr '+str(heading_correction)[0:4]+' off '+str(off_axis)[0:5]+' fina '+str(final_direction)[0:5])

        return (math.cos(final_direction), math.sin(final_direction), 0,)

    # pylint: disable=unused-argument
    # it is being left in to maintain method signature consistency
    def arrive_weight(self, unused_start, end, current):
        d = dist(current, end)
        if d < .3:
            d = .3
        return 2.0/d

    def weighted_traverse(self, start, end, current):
        traverse_vector = self.traverse_vector(start, end, current)
        w = self.traverse_weight(start, end, current)
        rospy.loginfo('wtr: '+str(traverse_vector[0])[0:4]+' , '+str(traverse_vector[1])[0:4]+' , '+str(w)[0:4])
        return scale(traverse_vector, w)

    # pylint: disable=unused-argument
    # it is being left in to maintain method signature consistency
    @unit # forces a unit vector to be returned, scaled by weight
    def traverse_vector(self, start, end, unused_current):
        dx = end.pose.pose.position.x - start.pose.pose.position.x
        dy = end.pose.pose.position.y - start.pose.pose.position.y
        return (dx, dy, 0,)

    # pylint: disable=unused-argument
    # it is being left in to maintain method signature consistency
    def traverse_weight(self, unused_start, unused_end, unused_current):
        # This is the standard unit. All other forces can use a weight of 1 to
        #  be of equal weight to the traverse weight. More indicates a greater
        #  requested priority.
        return 1.0

    def convert_to_odom(self, data):
        if data.header.frame_id == 'odom':
            return data
        else:
            listener = tf.TransformListener()
            try:
                (trans, rot) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
                return self.odom_transform_2d(data, trans, rot)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                return data

    def odom_transform_2d(self, data, new_frame, trans, rot):
        # NOTES: only in 2d rotation
        # also, it removes covariance, etc information
        odom_new = Odometry()
        odom_new.header = data.header
        odom_new.header.frame_id = new_frame

        odom_new.pose.pose.position.x = data.pose.pose.position.x + trans[0]
        odom_new.pose.pose.position.y = data.pose.pose.position.y + trans[1]
        odom_new.pose.pose.position.z = data.pose.pose.position.z + trans[2]
        odom_new.pose.pose.orientation = tft.quaternion_multiply(data.pose.pose.orientation, rot)

        heading_change = quaternion_to_heading(rot)

        odom_new.twist.twist.linear.x = data.twist.twist.linear.x*math.cos(heading_change) - data.twist.twist.linear.y*math.sin(heading_change)
        odom_new.twist.twist.linear.y = data.twist.twist.linear.y*math.cos(heading_change) + data.twist.twist.linear.x*math.sin(heading_change)
        odom_new.twist.twist.linear.z = 0

        odom_new.twist.twist.angular.x = 0
        odom_new.twist.twist.angular.y = 0
        odom_new.twist.twist.angular.z = data.twist.twist.angular.z

        return odom_new

class StateModel(object):
    def __init__(self, odom):
        self.x = odom.pose.pose.position.x
        self.y = odom.pose.pose.position.y
        self.theta = quaternion_to_heading(odom.pose.pose.orientation)
        self.v = odom.twist.twist.linear.x
        self.w = odom.twist.twist.angular.z
        self.a = 0
        self.alpha = 0
        self.frame_id = odom.header.frame_id

    def sample_motion_model2(self, v, w, dt):
        '''
        Return an odometry message sampled from the distribution defined by the
        probabilistic motion model based on this statemodel
        Does not yet take full advantage of state stored in above
        And does not check acceleration bounds for example
        '''
        accel_max = .1
        alpha_max = 1

        delta_v_req = v - self.v
        delta_v_max = accel_max*dt
        if delta_v_req > 0:
            if delta_v_max > delta_v_req:
                accel_time = delta_v_req / accel_max
                v_avg = v*(dt-accel_time) + (v+accel_max*2.0)*accel_time
                v_new = v
            else:
                v_avg = self.v+accel_max/2.0
                v_new = self.v+accel_max
        else:
            if delta_v_max > abs(delta_v_req):
                accel_time = abs(delta_v_req) / accel_max
                v_avg = v*(dt-accel_time) + (v-accel_max*2.0)*accel_time
                v_new = v
            else:
                v_avg = self.v-accel_max/2.0
                v_new = self.v-accel_max

        ds = v_avg*dt

        # rospy.loginfo('smm2: ds: '+str(ds))

        delta_w_req = w - self.w
        delta_w_max = alpha_max*dt
        if delta_w_req > 0:
            if delta_w_max > delta_w_req:
                accel_time = delta_w_req / alpha_max
                w_avg = w*(dt-accel_time) + (w+alpha_max*2.0)*accel_time
                w_new = w
            else:
                w_avg = self.w+alpha_max/2.0
                w_new = self.w+alpha_max
        else:
            if delta_w_max > abs(delta_w_req):
                accel_time = abs(delta_w_req) / alpha_max
                w_avg = w*(dt-accel_time) + (w-alpha_max*2.0)*accel_time
                w_new = w
            else:
                w_avg = self.w-alpha_max/2.0
                w_new = self.w-alpha_max

        dtheta = w_avg*dt
        theta_avg = self.theta + dtheta/2.0

        new_odom = Odometry()
        new_odom.pose.pose.position.x = self.x+ds*math.cos(theta_avg)
        new_odom.pose.pose.position.y = self.y+ds*math.sin(theta_avg)
        new_odom.pose.pose.orientation = heading_to_quaternion(self.theta + dtheta)
        new_odom.twist.twist.linear.x = v_new
        new_odom.twist.twist.angular.z = w_new
        new_odom.header.frame_id = self.frame_id

        return new_odom

    def sample_motion_model(self, v, w, dt):
        '''
        Return an odometry message sampled from the distribution defined by the
        probabilistic motion model based on this statemodel
        Does not yet take full advantage of state stored in above
        And does not check acceleration bounds for example
        '''
        # TODO(buckbaskin): use the v,w data stored above to add accel limits
        # TODO(buckbaskin): use the v,w data to create more accurate v_hat

        noise = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0) # no noise for now, ideal motion
        v_hat = v + self.sample_normal(noise[0]*v+noise[1]*w)
        w_hat = w + self.sample_normal(noise[2]*v+noise[3]*w)
        y_hat = self.sample_normal(noise[4]*v+noise[5]*w)

        if w_hat < .001:
            # rospy.loginfo('what is too small')
            x_new = self.x + v_hat*math.cos(self.theta)
            y_new = self.y + v_hat*math.sin(self.theta)
        else:
            # rospy.loginfo('smm: '+str(v)+' , '+str(w)+' , '+str(dt))
            x_new = (self.x - v_hat/w_hat*math.sin(self.theta)
                + v_hat/w_hat*math.sin(self.theta+w_hat*dt))
            # rospy.loginfo('smm: dx '+str(x_new-self.x))
            y_new = (self.y - v_hat/w_hat*math.cos(self.theta)
                - v_hat/w_hat*math.cos(self.theta+w_hat*dt))
            # rospy.loginfo('smm: dy '+str(y_new-self.y))

        theta_new = self.theta + w_hat*dt + y_hat*dt

        new_odom = Odometry()
        new_odom.pose.pose.position.x = x_new
        new_odom.pose.pose.position.y = y_new
        new_odom.pose.pose.orientation = heading_to_quaternion(theta_new)
        new_odom.twist.twist.linear.x = v_hat
        new_odom.twist.twist.angular.z = w_hat
        new_odom.header.frame_id = self.frame_id

        return new_odom

    # pylint: disable=unused-argument
    # it will be used when implmenented properly
    def sample_normal(self, unused_b):
        # TODO(buckbaskin): change to sample normal distribution
        # norm(mean=0, stdev = b)
        return 0



if __name__ == '__main__':
    # pylint: disable=invalid-name
    # leader is a fine name, it's not a constant
    leader = ForceLeader()
    leader.run_server()
