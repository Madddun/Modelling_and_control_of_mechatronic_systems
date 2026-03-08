function Qpout = kinematicCtrlOp(u)
%KUKA Kinematic Control Position
%   u(1):     current simulation time
%   u(2:8):   current joint position
%   u(9:24):  desired ef pose wrt wcf (16X1)
%   u(25):    operational proportional gain
%   u(26):    switch pose/position control


persistent Qp

% Current simulation time
t=u(1);

% Joint Position
Q=u(2:8);

% Desired EF pose wrt wcf, deserialized (16X1) to (4x4) matrix
Hd_W=reshape(u(9:24),4,4);

%% HT robot base (link0) wrt world cf. In this case is I (4x4)
%TODO: Define the HT of the robot base (link 0) w.r.t. wcf (w)
H0_W=...;

% Operational Proportional Gain
kp=u(25);

% Flag to define if we use, the value of this flag is controlled by the
% switch "Switch pose/position".
% 0: position only controller {3D} or
% 1: pose controller (position and orientation) {6D}. 
ctrl_type=u(26);

% Kinematic Parameters L1,L2,L3, and L4
% TODO: Use the result from Lab. Assignment 1
L=kukaIIWA7_params;

if t==0
    % Initial Joint velocity
    Qp=zeros(7,1);
end



%% Get the absolute HTs for all the joints with the new joint position
[HT_0, ~] = getAbsoluteHT_kukaIIWA7(Q,L, H0_W);


%% Homogeneous transformation of Link7 wrt base frame (link0)
%TODO: Get the HT of the end-effector (ef) wrt base link (0)
H7_0=...

%% Compute FK of Link7 with respect to link 0
Xef_0 = FK_kukaIIWA7(H7_0);

%TODO: Get the position of the ef wrt 0 from Xef_0
x7_0=...

%TODO: Get the orientation of the ef wrt 0 (euler angles ZYZ) from Xef_0
phief_0=...

% TODO: Desired EF pose wrt 0.
Hd_0=...

% TODO: Desired position EF wrt 0 from Hd_0
xd_0=...

% TODO: Desired orientation EF wrt 0 from Hd_0
Rd_0=...

% Transform Rotation to euler angles ZYZ
% TODO: use the function R2_euler_zyz.m and define the correct input argument
phid_0=R2_euler_zyz(...);

%TODO: Position Errors EF wrt Link 0 (3x1)
Dxef_0=...

%TODO: Orientation Errors EF wrt Link 0 (3x1)
Dphi_0=...

%TODO: Pose Errors EF wrt Link 0 (6x1)
DXef=...

% link 7 (EF) Geometric Jacobian
J7_0=J_EF_kukaIIWA7(Q,L);


% Analytic Jacobian link 7 (EF)
%Get the Euler angles to angular velocity transformation
%TODO: complete the function Tf.m
T=Tf(Xef_0(4:6,1));

%TODO: Compute the Anality Jacobin using the the Geometric Jacobian

JA7_0=...

% Task Selector: Jacobian (linear and angular) or (only linear) velocities
if ctrl_type
    %TODO: define the correct analytic jacobian for the pose task
    J=...; % Analytic Jacobian linear and angular velocities
    
    % Operational task dimension
    m=6;

    % Pose Error
    DX=DXef;
else
    %TODO: define the correct analytic jacobian for the position task
    J=... % Analytic Jacobian only linear velocities
    
    % Operational task dimension
    m=3;

    % Position Error
    DX=Dxef_0;
end


% Inverse differential kinematics
%TODO: calculate the inverse differential kinematics
Ji=...

%% Commanded Joint Velocity
% Proportional gain with the correct dimension according to the task
% dimension m
KP=kp*eye(m);

% Commanded Joint Velocity
% TODO: compute the commanded joint velocity Qp using the Task error DX
Qp=...

%% Safety Stop
% Manipulability index calculation
%TODO: calculate the manipulability index

wi=...

% Emergency stop strategy: If the manipulabilty index is below 0.005 stop
% the robot!
if wi<0.005
    fprintf(1,'Close to singularity! Stopping robot: %f\n',wi);
    %TODO: define QP to stop the robot
    Qp=....
end


% Output commanded Joint Velocity and end effector velocity (6x1) wrt 0
Qpout=[Qp;Xef_0];

end

