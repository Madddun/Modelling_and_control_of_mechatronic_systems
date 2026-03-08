function Qpr = Qpr_abbIRB4Op(u)
%QPR_ABBIRB4Op Compute the Joint velocity reference in the Operational space 

persistent Nold dt

% Joint position
Q=u(1:4);

% Joint Velocity
Qp=u(5:8);

% Time
t=u(9);

%Desired EF Position
xd_0=u(10:12);

%Desired EF Velocity
xdp_0=u(13:15);

%Desired EF Acceleration
xdpp_0=u(16:18);

% Integral Gain offset (To fine tune Ki)
Kis=diag(u(19:21));

% Proportional Gain
Kp=diag(u(22:24));

% Itegral Dx
iDeltaX=u(25:27);

% Integral Gain
% TODO: define Ki
Ki=

% Kinematic Parameters (provided by robot manufacturer)
L=abbIRB4_params;

% EF position wrt base link 0
% TODO: get the EF position
xef_0=

% Geometric Jacobian
% TODO: get the Jacobian of the ef wrt base link 0
Jef=
% Jacobian Linear Velocities
% TODO: Get the Jacobian of the ef linear velocitites only
Jefv=

% Time derivative Jacobian
% TODO: finish the function Jp_EF_abbIRB4
Jefp=
% Linear part
% TODO: get the jacabian of the EF linear velocities
Jefpv=

% EF linear velocity
% TODO: compute the EF linear velocity
xefp_0=

% EF Errors (Position and velocity)
% TODO: Define the error functions
DeltaX=
DeltaXp=


% PID-Like Control (Primary Task)
% Operational velocity reference
% TODO: define the operational velocity and accceleration references to
% generate a PID-like controller in the operational space
xpr_0=
xppr_0=

% Joint velocity reference
% TODO: get the inverse of the EF jacobian (linear velocities)
Ji=

% Secondary Task
% Joint Posture Task
Qd=[0;0;0;0];
Qdp=zeros(4,1);
Qdpp=zeros(4,1);

% Joint Position and Velocity Errors
% TODO: define the error functions 
Dq=
Dqp=

% Null-space projector
% TODO: compute the Null-space projector
N=

if t==0
    Nold=N;
    smodel_name=get_param(bdroot, 'Name');
    dt=str2double(get_param(smodel_name,'FixedStep'));
end

% Proportional gain for the joint position errors
Kqp=5*diag([1,1,1,1]);

% Joint velocity (acceleration) reference for the secondary task 
% PD-Like in Joint Space to generate the secondary task (Posture task)
% TODO: Define the error functions (joint space)
Qjpr=
Qjppr=

% Secondary Task reference Velocity (Projected by the Null-space of Jef)
% TODO: Get the projected velocity (secondary task reference velocity)
Qpn=

% Time-derivative of the Secondary task reference Velocity. 
% HINT: You can use a numeric derivation of N since the analytic 
% is too complicated!
Ndot=
Nold=N;
Qppn=

% Joint Velocity reference
% TODO: define the joint velocity (acceleration) references to produce a
% PID-like controller in the operational space (primary task)
qpr=
qppr=

% Selects if the secondary task is active.
% This flag is controlled by the Switch NullSpaceControl
% 1: Null-space active
% 0: Null-space off (only the primary task is running)
s_task=u(28);
if s_task
    % TODO: Define the joint velocity (and acceleration) references with
    % the operational task (primary task) and the joint posture task
    % (secondary task)
    qpr=
    qppr=
end

Qpr=[qpr;qppr;DeltaX;DeltaXp;xd_0;xdp_0;xdpp_0;xef_0;xefp_0];

end

