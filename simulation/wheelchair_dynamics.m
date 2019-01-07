%% Calculation of DOB 

syms M_BODY M_WHEEL gravity phi_ref gz theta RADIUS_WHEEL DIST_WHEELS I_WHEEL I_BODY D_MASSCENTER;
syms TorqueInput1 TorqueInput0 DisturbanceTemp1 DisturbanceTemp0; 
syms rpm1 rpm0; 
M = [I_WHEEL+RADIUS_WHEEL^2/(4*DIST_WHEELS^2)*(M_BODY*DIST_WHEELS^2+I_BODY), RADIUS_WHEEL^2/(4*DIST_WHEELS^2)*(M_BODY*DIST_WHEELS^2-I_BODY);RADIUS_WHEEL^2/(4*DIST_WHEELS^2)*(M_BODY*DIST_WHEELS^2-I_BODY), I_WHEEL+RADIUS_WHEEL^2/(4*DIST_WHEELS^2)*(M_BODY*DIST_WHEELS^2+I_BODY)];
V = [ 0 RADIUS_WHEEL^2/(2*DIST_WHEELS)*(M_BODY-2*M_WHEEL)*D_MASSCENTER*gz; -RADIUS_WHEEL^2/(2*DIST_WHEELS)*(M_BODY-2*M_WHEEL)*D_MASSCENTER*gz 0];
tau = [TorqueInput1;TorqueInput0];
dis = [DisturbanceTemp1;DisturbanceTemp0];
rpm = [rpm1;rpm0];
StG = 1/2*[-M_BODY*gravity*cos(phi_ref)*sin(theta)*RADIUS_WHEEL+M_BODY*gravity*sin(phi_ref)*sin(theta)*D_MASSCENTER/DIST_WHEELS*RADIUS_WHEEL;-M_BODY*gravity*cos(phi_ref)*sin(theta)*RADIUS_WHEEL-M_BODY*gravity*sin(phi_ref)*sin(theta)*D_MASSCENTER/DIST_WHEELS*RADIUS_WHEEL];
acc = inv(M)*(tau+dis-V*rpm-StG);
acc = simplify(acc);