import driver
from driver import heading_to_quaternion, quaternion_to_heading
from driver import dot_product, cross_product, scale, unit

class ExampleDriver(driver.Driver):
    smoothness = 10
    k = 1.0/smoothness
    a = 3*k
    b = 3*pow(k,2)
    c = pow(k,3)

    max_v = 1.0
    max_accel = .05
    max_omega = 0.5
    max_alpha = .05

    def __init__(self):
        super(ExampleDriver, self).__init__()
        self.last_odom = None

    def process_odom(self, odom):
        """
        Converts new best estimate of position to command
        Based on theory from:
        Y.J. Kanayama and F. Fahroo
        "A New Line Tracking Method for Non-Holonomic Vehicles,"
        in International Conference on Robotics and Automation,
        Albuquerque, NM, 1997, pp.2908-2913
        """
        if self.last_odom is None:
            self.last_odom = odom

        next_goal = self.lead_goal()
        if self.dist(next_goal, odom) < .04:
            # if you are .04 m or less from the goal, move forward
            # NOTE: this might change.
            # for example, you may want to look at the next_goal two points
            #  ahead and see if you want to skip the current one, or if you are
            #  ahead of the current one in the direction that you want to go
            next_goal = self.lead_next()

        # errors along axis "x", off axis "y", heading "theta"
        along, off, heading = self.calc_errors(odom, next_goal)

        # d(kurvature)/ds = -a(kurvature) - b(heading_error) - c(off_error)
        # kurvature is defined as: 1/instant radius
        # v = rw
        # w = v/r
        # instant radius = v/w

        # choose a, b, c based on:
        # a = 3k
        # b = 3(k^2)
        # c = k^3
        # choose k based on smoothness = 1/k

        # implement the math
        if odom.twist.twist.angular.z < .001:
            last_radius = float("inf")
        else:
            last_radius = (abs(odom.twist.twist.linear.x) / 
                abs(odom.twist.twist.angular.z))
        
        if last_radius < .001:
            kurvature = 1000 # upper bound on kurvature
        else:
            kurvature = 1.0 / last_radius

        dt = odom.header.time.secs - self.last_odom.header.time.secs
        deltaKurv_dt = -1.0*self.a*kurvature - self.b*heading - self.c*off
        delta_kurv_discrete = deltaKurv_dt*dt

        new_kurvature = kurvature+delta_kurv_discrete
        if new_kurvature < .001:
            new_radius = float("inf")
        else:
            new_radius = 1.0/new_kurvature

        # match speed to goal, with adjustment for position error
        linear_vel = goal.twist.twist.linear.x
        linear_vel += -0.5*along

        # limit maximum acceleration
        if odom.twist.twist.linear.x + self.max_accel < linear_vel:
            linear_vel = odom.twist.twist.linear.x + self.max_accel
        elif linear_vel < odom.twist.twist.linear.x - self.max_accel:
            linear_vel = odom.twist.twist.linear.x - self.max_accel

        # limit maximum speed
        if linear_vel > max_v:
            linear_vel = max_v
        elif linear_vel < -1.0*max_v:
            linear_vel = -1.0*max_v

        # calculate angular velocity
        angular_vel = linear_vel / new_radius

         # limit maximum angular acceleration
        if odom.twist.twist.angular.z + self.max_alpha < angular_vel:
            angular_vel = odom.twist.twist.angular.z + self.max_alpha
            # reset linear_vel based on new (lower, limited) angular vel
            linear_vel = angular_vel * new_radius
        elif angular_vel < odom.twist.twist.angular.z - self.max_alpha:
            angular_vel = odom.twist.twist.angular.z - self.max_alpha
            # reset linear_vel based on new (lower, limited) angular vel
            linear_vel = angular_vel * new_radius

        # limit maximum angular speed
        if angular_vel > max_omega:
            angular_vel = max_omega
            # reset linear_vel based on new (lower, limited) angular vel
            linear_vel = angular_vel * new_radius
        elif angular_vel < -1.0*max_omega:
            angular_vel = -1.0*max_omega
            # reset linear_vel based on new (lower, limited) angular vel
            linear_vel = angular_vel * new_radius

        # linear and angular velocity are now within dx/dt, d2x/dt2 limits
        twist_out = Twist()
        twist_out.linear.x = linear_vel
        twist_out.angular.z = angular_vel
        self.cmd_vel.publish(twist_out)
