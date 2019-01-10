function [X, C] = SampleParameter(sample_mem)
M_WHEEL = 6; RADIUS_WHEEL = 0.0825; DIST_WHEELS = 0.342; I_WHEEL = 0.03; 
A = zeros(2,3);
B = zeros(2,1);
sigmaAtA = zeros(size(A.'*A));
sigmaAtB = zeros(size(A.'*B));
sigmaA = zeros(size(A));
sigmaAt = zeros(size(A.'));
sigmaB = zeros(size(B));
for(i = 1 : size(sample_mem,1))
    A = [sample_mem(i,1) sample_mem(i,2) sample_mem(i,3);sample_mem(i,4) sample_mem(i,5) sample_mem(i,6)];
    B = [sample_mem(i,7); sample_mem(i,8)];
    sigmaAtA = sigmaAtA + A.'*A; 
    sigmaAtB = sigmaAtB + A.'*B; 
    sigmaA = sigmaA + A;
    sigmaAt = sigmaAt + A.';
    sigmaB = sigmaB + B;
end 
X = inv(sigmaAtA - 1/length(sample_mem)*sigmaAt * sigmaA)*(sigmaAtB - 1/length(sample_mem)*sigmaAt*sigmaB);
Xsimple = inv(sigmaAtA)*sigmaAtB;
%null(sigmaAtA - 1/count*sigmaAt * sigmaA)
C = 1/length(sample_mem)*(sigmaA*X - sigmaB);

m = (X(1)+X(2)-I_WHEEL)/(1/2*RADIUS_WHEEL*RADIUS_WHEEL);
I = (X(1)-X(2)-I_WHEEL)/(1/2*RADIUS_WHEEL*RADIUS_WHEEL/(DIST_WHEELS*DIST_WHEELS));
d = 2*DIST_WHEELS*X(3)/(m-2*M_WHEEL)/(RADIUS_WHEEL*RADIUS_WHEEL);
m_simple = (Xsimple(1)+Xsimple(2)-I_WHEEL)/(1/2*RADIUS_WHEEL*RADIUS_WHEEL);
I_simple = (Xsimple(1)-Xsimple(2)-I_WHEEL)/(1/2*RADIUS_WHEEL*RADIUS_WHEEL/(DIST_WHEELS*DIST_WHEELS));
d_simple = 2*DIST_WHEELS*Xsimple(3)/(m-2*M_WHEEL)/(RADIUS_WHEEL*RADIUS_WHEEL);
end