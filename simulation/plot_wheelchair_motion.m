%% Plot the motion of wheelchair
% Constant Definition
global m_ I_ d_ m_simple_ I_simple_ d_simple_ m_sample I_sample d_sample fit_num
global memory
RADIUS_WHEEL = 0.127; DIST_WHEELS = 0.342; 
LIM = 5;
desire = 0; 
% Plotting 
sim('sim_ex_wheelchair');
speed_R = ScopeData.signals(1,1).values(:,1); % rpm -> rad/s
speed_L = ScopeData.signals(1,1).values(:,2); % rpm -> rad/s
phi_ref = ScopeData.signals(1,2).values(:,1);
phi_desired = ScopeData.signals(1,2).values(:,2);
phi_nctrl = ScopeData.signals(1,2).values(:,3);
d_speed_R = ScopeData.signals(1,3).values(:,1);
d_speed_L = ScopeData.signals(1,3).values(:,2);
nctrl_speed_R = ScopeData.signals(1,4).values(:,1);
nctrl_speed_L = ScopeData.signals(1,4).values(:,2);

dt = tout(2)-tout(1);
x = zeros(length(speed_R),1);
y = zeros(length(speed_R),1);
x_d = zeros(length(d_speed_R),1);
y_d = zeros(length(d_speed_L),1);
x_nctrl = zeros(length(nctrl_speed_R),1);
y_nctrl = zeros(length(nctrl_speed_L),1);

for i = 1 : length(speed_R)-1
     x(i+1) = x(i) + 1/2*RADIUS_WHEEL*cos(phi_ref(i))*(speed_R(i)+speed_L(i))*dt;
     y(i+1) = y(i) + 1/2*RADIUS_WHEEL*sin(phi_ref(i))*(speed_R(i)+speed_L(i))*dt;
end
for i = 1 : length(d_speed_R)-1
     x_d(i+1) = x_d(i) + 1/2*RADIUS_WHEEL*cos(phi_desired(i))*(d_speed_R(i)+d_speed_L(i))*dt;
     y_d(i+1) = y_d(i) + 1/2*RADIUS_WHEEL*sin(phi_desired(i))*(d_speed_R(i)+d_speed_L(i))*dt;
end
for i = 1 : length(nctrl_speed_R)-1
     x_nctrl(i+1) = x_nctrl(i) + 1/2*RADIUS_WHEEL*cos(phi_nctrl(i))*(nctrl_speed_R(i)+nctrl_speed_L(i))*dt;
     y_nctrl(i+1) = y_nctrl(i) + 1/2*RADIUS_WHEEL*sin(phi_nctrl(i))*(nctrl_speed_R(i)+nctrl_speed_L(i))*dt;
end

figure
plot(x,y, 'b');
hold on;
plot(x_d,y_d,'r--');
hold on;
plot(x_nctrl, y_nctrl, 'k');
%quiver(x(300:20:400),y(300:20:400),x(301:20:401)-x(300:20:400),y(301:20:401)-y(300:20:400),'b');
grid on;
title('Motion of Wheelchair in 2-D plane');
xlabel('x distance(m) along gravity');
ylabel('y distance(m) traverse');
legend('real_ trajectory', 'desired_ trajectory', 'no_ ctrl_ trajectory');
xlim([-LIM LIM]); ylim([-LIM LIM]);


