
function [sys,x0,str,ts] = sim_s_wheelchair(t,x,u,flag,desire)


format long;

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,desire);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u,desire);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    sys = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end
% end csfunc

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes

sizes = simsizes;
sizes.NumContStates  = 9;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 9;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0; %input x 0 ; input o 1 about output
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);


x0 = [0;0;0;0;0;0;0;0;0];
%x0=[0;0;3*pi/2]; %test for 2017.01.16 inverse kinematics

str = [];
ts  = [0  0];

% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u,desire)
global m_ I_ d_ m_simple_ I_simple_ d_simple_ m_sample I_sample d_sample fit_num
global memory
persistent i
if isempty(i)
    i = 0;
end
%memory definition
MEMORY_SIZE = 100;
if isempty(memory)  
    memory = zeros(MEMORY_SIZE,8); % 100 data with 6 A 2 B
end
%%const
M_BODY = 61+60;M_WHEEL = 6; gravity =9.81; theta = 0*pi/180;
RADIUS_WHEEL = 0.0825; DIST_WHEELS = 0.342; I_WHEEL = 0.03; I_BODY = 5.16+6;
D_MASSCENTER = 0.239+0.1;

% Contorller // when using gz or phi_ref in control algorithms, add noise
% or error

%Input current on loadcell
loadcell_current1 = 8 + 4*rand 
loadcell_current0 = 8 + 4*rand

%% ideal situation
desired_acc = DesiredMotion([loadcell_current1; loadcell_current0], [x(4);x(5)]);
gz_d = 1/2*[RADIUS_WHEEL/DIST_WHEELS -RADIUS_WHEEL/DIST_WHEELS]*[x(4);x(5)];

%% give same force to real-circumstance 
%Disturbance
mu = 0.01;  %from wikipedia of rolling resistance
mu_r = 0.5;
if(abs(x(7))>0.1)
    friction1 = -mu * M_BODY * gravity/2 * RADIUS_WHEEL * sign(x(7));
else
    friction1 = 0;
end
if(abs(x(8))>0.1)
    friction0 = -mu * M_BODY * gravity/2 * RADIUS_WHEEL * sign(x(8));
else
    friction0 = 0;
end
v_resistance1 = -mu_r*x(7);
v_resistance0 = -mu_r*x(8);
DisturbanceTemp1 = friction1+v_resistance1+0.2*(friction1+v_resistance1)*rand; % 0.2/2 ->10% randomness
DisturbanceTemp0 = friction0+v_resistance0+0.2*(friction0+v_resistance0)*rand; % max 20%, min 0% randomness
% Control and Dynamics
rpm1_no_ctrl=x(7); rpm0_no_ctrl=x(8); phi_ref_no_ctrl = x(9);
gz_no_ctrl = 1/2*[RADIUS_WHEEL/DIST_WHEELS -RADIUS_WHEEL/DIST_WHEELS]*[x(7);x(8)];
TorqueInput1_no_ctrl= 0.4*loadcell_current1; TorqueInput0_no_ctrl = 0.4*loadcell_current0;

acc_no_ctrl = [((M_BODY*DIST_WHEELS^2*RADIUS_WHEEL^2 + 4*I_WHEEL*DIST_WHEELS^2 + I_BODY*RADIUS_WHEEL^2)*(DisturbanceTemp1 + TorqueInput1_no_ctrl + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref_no_ctrl)*sin(theta))/2 - (D_MASSCENTER*RADIUS_WHEEL^2*gz_no_ctrl*rpm0_no_ctrl*(M_BODY - 2*M_WHEEL))/(2*DIST_WHEELS) - (D_MASSCENTER*M_BODY*RADIUS_WHEEL*gravity*sin(phi_ref_no_ctrl)*sin(theta))/(2*DIST_WHEELS)))/((2*I_WHEEL*DIST_WHEELS^2 + I_BODY*RADIUS_WHEEL^2)*(M_BODY*RADIUS_WHEEL^2 + 2*I_WHEEL)) + (RADIUS_WHEEL^2*(- M_BODY*DIST_WHEELS^2 + I_BODY)*(DisturbanceTemp0 + TorqueInput0_no_ctrl + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref_no_ctrl)*sin(theta))/2 + (D_MASSCENTER*RADIUS_WHEEL^2*gz_no_ctrl*rpm1_no_ctrl*(M_BODY - 2*M_WHEEL))/(2*DIST_WHEELS) + (D_MASSCENTER*M_BODY*RADIUS_WHEEL*gravity*sin(phi_ref_no_ctrl)*sin(theta))/(2*DIST_WHEELS)))/((2*I_WHEEL*DIST_WHEELS^2 + I_BODY*RADIUS_WHEEL^2)*(M_BODY*RADIUS_WHEEL^2 + 2*I_WHEEL));
 ((M_BODY*DIST_WHEELS^2*RADIUS_WHEEL^2 + 4*I_WHEEL*DIST_WHEELS^2 + I_BODY*RADIUS_WHEEL^2)*(DisturbanceTemp0 + TorqueInput0_no_ctrl + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref_no_ctrl)*sin(theta))/2 + (D_MASSCENTER*RADIUS_WHEEL^2*gz_no_ctrl*rpm1_no_ctrl*(M_BODY - 2*M_WHEEL))/(2*DIST_WHEELS) + (D_MASSCENTER*M_BODY*RADIUS_WHEEL*gravity*sin(phi_ref_no_ctrl)*sin(theta))/(2*DIST_WHEELS)))/((2*I_WHEEL*DIST_WHEELS^2 + I_BODY*RADIUS_WHEEL^2)*(M_BODY*RADIUS_WHEEL^2 + 2*I_WHEEL)) + (RADIUS_WHEEL^2*(- M_BODY*DIST_WHEELS^2 + I_BODY)*(DisturbanceTemp1 + TorqueInput1_no_ctrl + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref_no_ctrl)*sin(theta))/2 - (D_MASSCENTER*RADIUS_WHEEL^2*gz_no_ctrl*rpm0_no_ctrl*(M_BODY - 2*M_WHEEL))/(2*DIST_WHEELS) - (D_MASSCENTER*M_BODY*RADIUS_WHEEL*gravity*sin(phi_ref_no_ctrl)*sin(theta))/(2*DIST_WHEELS)))/((2*I_WHEEL*DIST_WHEELS^2 + I_BODY*RADIUS_WHEEL^2)*(M_BODY*RADIUS_WHEEL^2 + 2*I_WHEEL))];

%% with compensation control 
% Disturbance
if(abs(x(1))>0.1)
    friction1 = -mu * M_BODY * gravity/2 * RADIUS_WHEEL * sign(x(1));
else
    friction1 = 0;
end
if(abs(x(2))>0.1)
    friction0 = -mu * M_BODY * gravity/2 * RADIUS_WHEEL * sign(x(2));
else
    friction0 = 0;
end
v_resistance1 = -mu_r*x(1);
v_resistance0 = -mu_r*x(2);
DisturbanceTemp1 = friction1+v_resistance1+0.2*(friction1+v_resistance1)*rand % 0.2/2 -> 10% randomness
DisturbanceTemp0 = friction0+v_resistance0+0.2*(friction0+v_resistance0)*rand % max 20%, min 0% randomness
DisturbanceObserver1 = DisturbanceTemp1+0.1*DisturbanceTemp1*randn;  % observation error -> 70%possibility : under 10% error, 
DisturbanceObserver0 = DisturbanceTemp0+0.1*DisturbanceTemp0*randn; % 25%possibility : 20%error, 5%: over 30% error
% Control and Dynamics
rpm1=x(1); rpm0=x(2); phi_ref = x(3);
gz = 1/2*[RADIUS_WHEEL/DIST_WHEELS -RADIUS_WHEEL/DIST_WHEELS]*[x(1);x(2)];
control_torque = CompensationControl([x(1);x(2)], x(3), gz, theta, [DisturbanceObserver1;DisturbanceObserver0], desired_acc)
TorqueInput1 = control_torque(1); TorqueInput0 = control_torque(2);
acc = [((M_BODY*DIST_WHEELS^2*RADIUS_WHEEL^2 + 4*I_WHEEL*DIST_WHEELS^2 + I_BODY*RADIUS_WHEEL^2)*(DisturbanceTemp1 + TorqueInput1 + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref)*sin(theta))/2 - (D_MASSCENTER*RADIUS_WHEEL^2*gz*rpm0*(M_BODY - 2*M_WHEEL))/(2*DIST_WHEELS) - (D_MASSCENTER*M_BODY*RADIUS_WHEEL*gravity*sin(phi_ref)*sin(theta))/(2*DIST_WHEELS)))/((2*I_WHEEL*DIST_WHEELS^2 + I_BODY*RADIUS_WHEEL^2)*(M_BODY*RADIUS_WHEEL^2 + 2*I_WHEEL)) + (RADIUS_WHEEL^2*(- M_BODY*DIST_WHEELS^2 + I_BODY)*(DisturbanceTemp0 + TorqueInput0 + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref)*sin(theta))/2 + (D_MASSCENTER*RADIUS_WHEEL^2*gz*rpm1*(M_BODY - 2*M_WHEEL))/(2*DIST_WHEELS) + (D_MASSCENTER*M_BODY*RADIUS_WHEEL*gravity*sin(phi_ref)*sin(theta))/(2*DIST_WHEELS)))/((2*I_WHEEL*DIST_WHEELS^2 + I_BODY*RADIUS_WHEEL^2)*(M_BODY*RADIUS_WHEEL^2 + 2*I_WHEEL));
 ((M_BODY*DIST_WHEELS^2*RADIUS_WHEEL^2 + 4*I_WHEEL*DIST_WHEELS^2 + I_BODY*RADIUS_WHEEL^2)*(DisturbanceTemp0 + TorqueInput0 + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref)*sin(theta))/2 + (D_MASSCENTER*RADIUS_WHEEL^2*gz*rpm1*(M_BODY - 2*M_WHEEL))/(2*DIST_WHEELS) + (D_MASSCENTER*M_BODY*RADIUS_WHEEL*gravity*sin(phi_ref)*sin(theta))/(2*DIST_WHEELS)))/((2*I_WHEEL*DIST_WHEELS^2 + I_BODY*RADIUS_WHEEL^2)*(M_BODY*RADIUS_WHEEL^2 + 2*I_WHEEL)) + (RADIUS_WHEEL^2*(- M_BODY*DIST_WHEELS^2 + I_BODY)*(DisturbanceTemp1 + TorqueInput1 + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref)*sin(theta))/2 - (D_MASSCENTER*RADIUS_WHEEL^2*gz*rpm0*(M_BODY - 2*M_WHEEL))/(2*DIST_WHEELS) - (D_MASSCENTER*M_BODY*RADIUS_WHEEL*gravity*sin(phi_ref)*sin(theta))/(2*DIST_WHEELS)))/((2*I_WHEEL*DIST_WHEELS^2 + I_BODY*RADIUS_WHEEL^2)*(M_BODY*RADIUS_WHEEL^2 + 2*I_WHEEL))];

i = i+1;
if (theta == 0 && i<=300+MEMORY_SIZE && i>300) 
    memory(i-300,1:8) = [acc(1) acc(2) x(2)*gz acc(2) acc(1) -x(1)*gz TorqueInput1+DisturbanceObserver1 TorqueInput0+DisturbanceObserver0];
    [m,I,d, m_simple, I_simple, d_simple]= ParameterEstimator(acc, [x(1);x(2)], gz, [TorqueInput1;TorqueInput0],0);
     m_ = m; I_ = I; d_ = d;
     m_simple_ = m_simple; I_simple_= I_simple; d_simple_=d_simple;
end

SAMPLE_NUM = 15;
X_sample = zeros(3, SAMPLE_NUM);
C_sample = zeros(2, SAMPLE_NUM);
fit_count_max = 0; max_index = 0;
if (i == 400)
    i = i+1;
    for(j = 1 : SAMPLE_NUM)
        fit_count = 0; 
        sample_mem = datasample(memory, 5);
        [X, C] = SampleParameter(sample_mem);
        X_sample(:, j) = X;
        C_sample(:, j) = C;        
        for k = 1 : MEMORY_SIZE    
            A = [memory(k,1:3);memory(k,4:6)];
            B = [memory(k,7);memory(k,8)];
            if(norm(B-A*X-C)<0.1*norm(B))
                fit_count = fit_count + 1;
            end
        end
        if(fit_count_max < fit_count)
            fit_count_max = fit_count;
            max_index = j;
        else
            fit_count_max = fit_count_max;
            max_index = 1;
        end
    end
    [m_sample, I_sample, d_sample] = Mcalculator(X_sample(:,max_index));
    fit_num = fit_count_max;
end

sys=[acc;gz;desired_acc;gz_d;acc_no_ctrl;gz_no_ctrl]; 
% sys = zeros(8,1);



% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,desire)


sys=[x(1);x(2);x(3);x(4);x(5);x(6);x(7);x(8);x(9)];


% end mdlOutputs
