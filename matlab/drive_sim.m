xTarget = 10;
yTarget = 10;
thetaTarget = 0;
velTarget = 0.5;
omegaTarget = 0;
target = [xTarget yTarget thetaTarget]';

xState = 5;
yState = 5;
thetaState = -3*pi/4;
velState = 0.5;
omegaState = 0;
state0 = [xState yState thetaState]';

vec1  = @(target,state)[target(1)-state(1) target(2)-state(2) 0];
vec2  = @(target,state)[cos(target(3)) sin(target(3)), 0];
vec3  = @(target,state)dot(vec1(target,state),vec2(target,state))*vec2(target,state);
vec4  = @(target,state)vec1(target,state)-vec3(target,state);
along = @(target,state)-dot(vec1(target,state),vec2(target,state));
off   = @(target,state)sum(sign(cross(vec2(target,state),vec4(target,state))))*norm(vec4(target,state));
head_parallel  = @(target,state)mod(((state(3)-target(3))+pi),2*pi)-pi;
head_perpendicular = @(target,state)mod(((sum(sign(cross(vec2(target,state),vec4(target,state))))*head_parallel(target,state)-pi/2)+pi),2*pi)-pi;


omega_max = 0.5; %rad/s
k_off     = 0.012;
k_head    = 0.4;
vel   = @(target,state)0.5;
% omega = @(target,state)max(-omega_max,min(omega_max,(k_off*off(target,state)-k_head*head(target,state))));
omega = @(target,state)-k_off*off(target,state)*head_perpendicular(target,state)-k_head*head_parallel(target,state);

cf    = @(omega,dt)(omega==0)*1+(omega~=0)*sin(omega*dt/2)/(omega*dt/2);

dstate= @(target,state,dt)[cf(omega(target,state),dt)*vel(target,state)*dt*cos(state(3)+omega(target,state)*dt/2); ...
                           cf(omega(target,state),dt)*vel(target,state)*dt*sin(state(3)+omega(target,state)*dt/2); ...
                           omega(target,state)*dt];

dt = 0.01;
N = 2000;
x = NaN(1,N);
y = NaN(1,N);
theta = NaN(1,N);
x(1) = state0(1);
y(1) = state0(2);
theta(1) = state0(3);
a = NaN(1,N-1);
o = NaN(1,N-1);
h1 = NaN(1,N-1);
h2 = NaN(1,N-1);
w = NaN(1,N-1);
for i = 2:N
    temp = [x(i-1) y(i-1) theta(i-1)]' + dstate(target,[x(i-1) y(i-1) theta(i-1)],dt);
    x(i) = temp(1);
    y(i) = temp(2);
    theta(i) = temp(3);
    a(i-1) = along(target,temp);
    o(i-1) = off(target,temp);
    h1(i-1) = head_parallel(target,temp);
    h2(i-1) = head_perpendicular(target,temp);
    w(i-1) = omega(target,[x(i-1) y(i-1) theta(i-1)]);
end

figure(1)
plot(x(1),y(1),'bo', ...
     [x(1) x(1)+cos(theta(1))],[y(1) y(1)+sin(theta(1))],'b-', ...
     target(1),target(2),'ro', ...
     [target(1) target(1)+cos(target(3))],[target(2) target(2)+sin(target(3))],'r-', ...
     x,y,'g-')
xlim([-5 15])
ylim([-5 15])
axis equal
figure(2)
subplot(4,1,1)
plot(a)
ylabel('along')
subplot(4,1,2)
plot(o)
ylabel('off')
subplot(4,1,3)
plot(h1)
ylabel('parrallel error')
subplot(4,1,4)
plot(w)
ylabel('perpendicular error')