function control_torque = CompensationControl(wheel_speed, phi_ref, gz, theta, disturbance, desired_acc, m, I, d)

M_BODY = 61+60;M_WHEEL = 8; gravity =9.81; 
RADIUS_WHEEL = 0.127; DIST_WHEELS = 0.342; I_WHEEL = 0.03; I_BODY = 5.16+6;
D_MASSCENTER = 0.239+0.1; 

M_BODY = m; I_BODY = I; D_MASSCENTER = d;

V = [ 0 RADIUS_WHEEL^2/(2*DIST_WHEELS)*(M_BODY-2*M_WHEEL)*D_MASSCENTER*gz; -RADIUS_WHEEL^2/(2*DIST_WHEELS)*(M_BODY-2*M_WHEEL)*D_MASSCENTER*gz 0];
StG = 1/2*[-M_BODY*gravity*cos(phi_ref)*sin(theta)*RADIUS_WHEEL+M_BODY*gravity*sin(phi_ref)*sin(theta)*D_MASSCENTER/DIST_WHEELS*RADIUS_WHEEL;-M_BODY*gravity*cos(phi_ref)*sin(theta)*RADIUS_WHEEL-M_BODY*gravity*sin(phi_ref)*sin(theta)*D_MASSCENTER/DIST_WHEELS*RADIUS_WHEEL];
M = [I_WHEEL+RADIUS_WHEEL^2/(4*DIST_WHEELS^2)*(M_BODY*DIST_WHEELS^2+I_BODY), RADIUS_WHEEL^2/(4*DIST_WHEELS^2)*(M_BODY*DIST_WHEELS^2-I_BODY);RADIUS_WHEEL^2/(4*DIST_WHEELS^2)*(M_BODY*DIST_WHEELS^2-I_BODY), I_WHEEL+RADIUS_WHEEL^2/(4*DIST_WHEELS^2)*(M_BODY*DIST_WHEELS^2+I_BODY)];
control_torque = V*wheel_speed + StG - disturbance + M*desired_acc;

end
