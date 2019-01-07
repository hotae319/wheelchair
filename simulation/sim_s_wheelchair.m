
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
sizes.NumContStates  = 6;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 6;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0; %input x 0 ; input o 1 about output
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);


x0 = [0;0;0;0;0;0];
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
global m_ I_ d_ m_simple_ I_simple_ d_simple_
%%const
M_BODY = 61+60;M_WHEEL = 6; gravity =9.81; theta = 10*pi/180;
RADIUS_WHEEL = 0.0825; DIST_WHEELS = 0.342; I_WHEEL = 0.03; I_BODY = 11+12;
D_MASSCENTER = 0.239+0.1;
%PID gain
% c=[60;30;60;30];
% k=[144;525;144;144];
%%MODEL
rpm1=x(1); rpm0=x(2); phi_ref = x(3);
% Contorller // when using gz or phi_ref in control algorithms, add noise
% or error

gz = 1/2*[RADIUS_WHEEL/DIST_WHEELS -RADIUS_WHEEL/DIST_WHEELS]*[x(1);x(2)];

% Disturbance
DisturbanceTemp1 = 0.5+0.5*randn;
DisturbanceTemp0 = 0.5+0.5*randn;
DisturbanceObserver1 = DisturbanceTemp1+1.0*randn; %observation error
DisturbanceObserver0 = DisturbanceTemp0+1.0*randn;

loadcell_current1 = 5 + 8*rand; loadcell_current0 = 6 - 8*rand;
desired_acc = DesiredMotion([loadcell_current1; loadcell_current0]);
gz_d = 1/2*[RADIUS_WHEEL/DIST_WHEELS -RADIUS_WHEEL/DIST_WHEELS]*[x(4);x(5)];

% control_torque = CompensationControl([x(1);x(2)], x(3), gz, theta, [DisturbanceObserver1;DisturbanceObserver0], desired_acc) 
% TorqueInput1 = control_torque(1); TorqueInput0 = control_torque(2);

% give same force to real-circumstance 
TorqueInput1 = 0.4*loadcell_current1; TorqueInput0 = 0.4*loadcell_current0;

acc = [((M_BODY*DIST_WHEELS^2*RADIUS_WHEEL^2 + 4*I_WHEEL*DIST_WHEELS^2 + I_BODY*RADIUS_WHEEL^2)*(DisturbanceTemp1 + TorqueInput1 + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref)*sin(theta))/2 - (D_MASSCENTER*RADIUS_WHEEL^2*gz*rpm0*(M_BODY - 2*M_WHEEL))/(2*DIST_WHEELS) - (D_MASSCENTER*M_BODY*RADIUS_WHEEL*gravity*sin(phi_ref)*sin(theta))/(2*DIST_WHEELS)))/((2*I_WHEEL*DIST_WHEELS^2 + I_BODY*RADIUS_WHEEL^2)*(M_BODY*RADIUS_WHEEL^2 + 2*I_WHEEL)) + (RADIUS_WHEEL^2*(- M_BODY*DIST_WHEELS^2 + I_BODY)*(DisturbanceTemp0 + TorqueInput0 + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref)*sin(theta))/2 + (D_MASSCENTER*RADIUS_WHEEL^2*gz*rpm1*(M_BODY - 2*M_WHEEL))/(2*DIST_WHEELS) + (D_MASSCENTER*M_BODY*RADIUS_WHEEL*gravity*sin(phi_ref)*sin(theta))/(2*DIST_WHEELS)))/((2*I_WHEEL*DIST_WHEELS^2 + I_BODY*RADIUS_WHEEL^2)*(M_BODY*RADIUS_WHEEL^2 + 2*I_WHEEL));
 ((M_BODY*DIST_WHEELS^2*RADIUS_WHEEL^2 + 4*I_WHEEL*DIST_WHEELS^2 + I_BODY*RADIUS_WHEEL^2)*(DisturbanceTemp0 + TorqueInput0 + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref)*sin(theta))/2 + (D_MASSCENTER*RADIUS_WHEEL^2*gz*rpm1*(M_BODY - 2*M_WHEEL))/(2*DIST_WHEELS) + (D_MASSCENTER*M_BODY*RADIUS_WHEEL*gravity*sin(phi_ref)*sin(theta))/(2*DIST_WHEELS)))/((2*I_WHEEL*DIST_WHEELS^2 + I_BODY*RADIUS_WHEEL^2)*(M_BODY*RADIUS_WHEEL^2 + 2*I_WHEEL)) + (RADIUS_WHEEL^2*(- M_BODY*DIST_WHEELS^2 + I_BODY)*(DisturbanceTemp1 + TorqueInput1 + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref)*sin(theta))/2 - (D_MASSCENTER*RADIUS_WHEEL^2*gz*rpm0*(M_BODY - 2*M_WHEEL))/(2*DIST_WHEELS) - (D_MASSCENTER*M_BODY*RADIUS_WHEEL*gravity*sin(phi_ref)*sin(theta))/(2*DIST_WHEELS)))/((2*I_WHEEL*DIST_WHEELS^2 + I_BODY*RADIUS_WHEEL^2)*(M_BODY*RADIUS_WHEEL^2 + 2*I_WHEEL))];

if (theta == 0) 
    [m,I,d, m_simple, I_simple, d_simple]= ParameterEstimator(acc, [x(1);x(2)], gz, [TorqueInput1;TorqueInput0]);
     m_ = m; I_ = I; d_ = d;
     m_simple_ = m_simple; I_simple_= I_simple; d_simple_=d_simple;
end
sys=[acc;gz;desired_acc;gz_d]; 
% sys = zeros(8,1);



% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,desire)


sys=[x(1);x(2);x(3);x(4);x(5);x(6)];


% end mdlOutputs
