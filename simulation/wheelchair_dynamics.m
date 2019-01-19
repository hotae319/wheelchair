%% Calculation of DOB 

syms M_BODY M_WHEEL gravity phi_ref gz theta RADIUS_WHEEL DIST_WHEELS I_WHEEL I_BODY D_MASSCENTER D_HANDLE;
syms TorqueInput1 TorqueInput0 DisturbanceTemp1 DisturbanceTemp0 desired_acc1 desired_acc0;
syms rpm1 rpm0; 
syms rk1 rk2;
M = [I_WHEEL+RADIUS_WHEEL^2/(4*DIST_WHEELS^2)*(M_BODY*DIST_WHEELS^2+I_BODY), RADIUS_WHEEL^2/(4*DIST_WHEELS^2)*(M_BODY*DIST_WHEELS^2-I_BODY);RADIUS_WHEEL^2/(4*DIST_WHEELS^2)*(M_BODY*DIST_WHEELS^2-I_BODY), I_WHEEL+RADIUS_WHEEL^2/(4*DIST_WHEELS^2)*(M_BODY*DIST_WHEELS^2+I_BODY)];
V = [ 0 RADIUS_WHEEL^2/(2*DIST_WHEELS)*(M_BODY-2*M_WHEEL)*D_MASSCENTER*gz; -RADIUS_WHEEL^2/(2*DIST_WHEELS)*(M_BODY-2*M_WHEEL)*D_MASSCENTER*gz 0];
S = 1/2*[RADIUS_WHEEL*cos(phi_ref) RADIUS_WHEEL*cos(phi_ref); RADIUS_WHEEL*sin(phi_ref) RADIUS_WHEEL*sin(phi_ref); RADIUS_WHEEL/DIST_WHEELS -RADIUS_WHEEL/DIST_WHEELS;2 0; 0 2];
B = [cos(phi_ref) cos(phi_ref) 0 0; sin(phi_ref) sin(phi_ref) 0 0; D_HANDLE -D_HANDLE 0 0; 0 0 1 0; 0 0 0 1];
S.'*B
StB = [ (RADIUS_WHEEL*(DIST_WHEELS + D_HANDLE))/(2*DIST_WHEELS), (RADIUS_WHEEL*(DIST_WHEELS - D_HANDLE))/(2*DIST_WHEELS), 1, 0;...
 (RADIUS_WHEEL*(DIST_WHEELS - D_HANDLE))/(2*DIST_WHEELS), (RADIUS_WHEEL*(DIST_WHEELS + D_HANDLE))/(2*DIST_WHEELS), 0, 1];
tau = [TorqueInput1;TorqueInput0];
dis = [DisturbanceTemp1;DisturbanceTemp0];
rpm = [rpm1;rpm0];
StG = 1/2*[-M_BODY*gravity*cos(phi_ref)*sin(theta)*RADIUS_WHEEL+M_BODY*gravity*sin(phi_ref)*sin(theta)*D_MASSCENTER/DIST_WHEELS*RADIUS_WHEEL;-M_BODY*gravity*cos(phi_ref)*sin(theta)*RADIUS_WHEEL-M_BODY*gravity*sin(phi_ref)*sin(theta)*D_MASSCENTER/DIST_WHEELS*RADIUS_WHEEL];
% acc = inv(M)*(tau+dis-V*rpm-StG);
% acc = simplify(acc);

% Rk = [rk1 0; 0 rk2];
% accacc = inv(M)*(tau-V*rpm);
% accacc = simplify(accacc);
% desired_acc = inv(M)*(tau-Rk*rpm);
% desired_acc = simplify(desired_acc);
desired_acc = [desired_acc1;desired_acc0];
control_torque = V*rpm + StG - dis + M*desired_acc