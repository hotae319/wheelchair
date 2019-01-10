function desired_acc = DesiredMotion(current, wheel_speed)

M_BODY = 61;M_WHEEL = 6; gravity =9.81; theta = 0*pi/180;
RADIUS_WHEEL = 0.0825; DIST_WHEELS = 0.342; I_WHEEL = 0.03; I_BODY = 5.16;
Kt = 0.4; Rk = [0.5 0; 0 0.5]; % resistance speed
torque = Kt * current;
M = [I_WHEEL+RADIUS_WHEEL^2/(4*DIST_WHEELS^2)*(M_BODY*DIST_WHEELS^2+I_BODY), RADIUS_WHEEL^2/(4*DIST_WHEELS^2)*(M_BODY*DIST_WHEELS^2-I_BODY);RADIUS_WHEEL^2/(4*DIST_WHEELS^2)*(M_BODY*DIST_WHEELS^2-I_BODY), I_WHEEL+RADIUS_WHEEL^2/(4*DIST_WHEELS^2)*(M_BODY*DIST_WHEELS^2+I_BODY)];

% a = M(1,1); b = M(1,2); c = M(2,1); d = M(2,2); k1 = Rk(1,1); k2 = Rk(2,2);
% lambda1 = (-(k1*d+k2*a)+sqrt((k1*d+k2*a)^2 - 4*(a*d-b*c)*k1*k2))/2/(a*d-b*c);
% lambda2 = (-(k1*d+k2*a)-sqrt((k1*d+k2*a)^2 - 4*(a*d-b*c)*k1*k2))/2/(a*d-b*c);
% e = (wheel_speed(1)+wheel_speed(2)-torque(1)/k1/b/lambda2 - torque(2)/k2/(a*lambda2+k1))/...
%     (1/(a*lambda1+k1)/b/lambda2-1/(a*lambda2+k1)/b/lambda1);
% f = (wheel_speed(1) - e/(a*lambda1+k1) - torque(1)/k1)*(a(lambda2+k1);

desired_acc = inv(M)*(torque-Rk*wheel_speed);

end
   