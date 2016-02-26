goal_vel = 0.5;
heading_err = linspace(-pi,pi,100);
off = -1;
along = -0.1;

header_err_ad = atan(4.8284*off) + heading_err;

a = 1;
b = 2.7468;

omega = -atan(a*header_err_ad)/b;

scaling_factor = min(1,max((0.5-abs(omega))/0.5,0));
% velocity = (0*(-atan(10*sqrt(off^2 + along^2))/2.7468) + goal_vel)*scaling_factor;
velocity = goal_vel*scaling_factor;

figure(1)
subplot(3,1,1)
plot(heading_err,omega)
xlim([-pi pi])
xlabel('heading error (rad)')
ylabel('omega (rad/s)')
subplot(3,1,2)
plot(heading_err,scaling_factor)
xlim([-pi pi])
xlabel('heading error (rad)')
ylabel('scaling factor')
subplot(3,1,3)
plot(heading_err,velocity)
xlim([-pi pi])
xlabel('heading error (rad)')
ylabel('velocity (m/s)')