function [m, I, d, m_simple, I_simple, d_simple] = ParameterEstimator(angle_acc, angle_speed, phi_dot, torque, revision)
%% General
M_WHEEL = 6; RADIUS_WHEEL = 0.0825; DIST_WHEELS = 0.342; I_WHEEL = 0.03; 
persistent sigmaAtA sigmaAtB sigmaA sigmaAt sigmaB
persistent count
A = [angle_acc(1) angle_acc(2) angle_speed(2)*phi_dot; angle_acc(2) angle_acc(1) -angle_speed(1)*phi_dot];
B = torque;
if isempty(count)
    count = 0;
end
if isempty(sigmaAtA)
    sigmaAtA = zeros(size(A.'*A));
end
if isempty(sigmaAtB)
    sigmaAtB = zeros(size(A.'*B));
end
if isempty(sigmaA)
    sigmaA = zeros(size(A));
end
if isempty(sigmaAt)
    sigmaAt = zeros(size(A.'));
end
if isempty(sigmaB)
    sigmaB = zeros(size(B));
end

sigmaAtA = sigmaAtA + A.'*A; 
sigmaAtB = sigmaAtB + A.'*B; 
sigmaA = sigmaA + A;
sigmaAt = sigmaAt + A.';
sigmaB = sigmaB + B;
count = count + 1;
X = inv(sigmaAtA - 1/count*sigmaAt * sigmaA)*(sigmaAtB - 1/count*sigmaAt*sigmaB);
Xsimple = inv(sigmaAtA)*sigmaAtB;
%null(sigmaAtA - 1/count*sigmaAt * sigmaA)
C = 1/count*(sigmaA*X - sigmaB);
m = (X(1)+X(2)-I_WHEEL)/(1/2*RADIUS_WHEEL*RADIUS_WHEEL);
I = (X(1)-X(2)-I_WHEEL)/(1/2*RADIUS_WHEEL*RADIUS_WHEEL/(DIST_WHEELS*DIST_WHEELS));
d = 2*DIST_WHEELS*X(3)/(m-2*M_WHEEL)/(RADIUS_WHEEL*RADIUS_WHEEL);
m_simple = (Xsimple(1)+Xsimple(2)-I_WHEEL)/(1/2*RADIUS_WHEEL*RADIUS_WHEEL);
I_simple = (Xsimple(1)-Xsimple(2)-I_WHEEL)/(1/2*RADIUS_WHEEL*RADIUS_WHEEL/(DIST_WHEELS*DIST_WHEELS));
d_simple = 2*DIST_WHEELS*Xsimple(3)/(m-2*M_WHEEL)/(RADIUS_WHEEL*RADIUS_WHEEL);
 
%% Revision
if(revision == 1)
    
end
      
end