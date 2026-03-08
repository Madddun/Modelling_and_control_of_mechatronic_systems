functionfunction Qpout = kinematicCtrlOpNS(u)
%KUKA Kinematic Control Position
%   Detailed explanation goes here
persistent Qp

% Robot Dofs
dof=7;

% Current simulation time
t=u(1);

% Joint Position
Q=u(2:8);

% Desired EF pose wrt wcf, deserialized (16X1) to (4x4) matrix
Hd_W=reshape(u(9:24),4,4);

%% HT robot base (link0) wrt world cf. In this case is I (4x4)
H0_W=eye(4);

% Operational Proportional Gain
kp=u(25);

% Flag to define if we use Null space control or not. 
% The value of this flag is controlled by the switch 
% "Switch NullSpaceControl (on/off)".
% 0: Only Operational control (No secondary task)
% 1: Null-Space Control (Secondary tasks in Joint space)
ctrl_type=u(26);

% Operational Task dimension. Defines which axes will be used in the
% operational task, e.g., 
% m=3 (position control), linear position x,y, and z. 
% m=4 linear  position x,y,z, and euler angular position Z.
% m=6 (Full-pose control), linear position x,y,z, and 
% euler angular position Z,Y,Z.
m=u(27);

% Kinematic Parameters L1,L2,L3, and L4
L=kukaIIWA7_params;

if t==0
    % Initial Joint velocity
    Qp=zeros(dof,1);
end



%% Get the absolute HTs for all the joints with the new joint position
[HT_0, ~] = getAbsoluteHT_kukaIIWA7(Q,L, H0_W);


%% Homogeneous transformation of Link7 wrt base frame (link0)
%TODO: Get the HT of the end-effector (ef) wrt base link (0)
H7_0=

%% Compute FK of Link7 with respect to link 0
Xef_0 = FK_kukaIIWA7(H7_0);

%TODO: Get the position of the ef wrt 0 from Xef_0
x7_0=

%TODO: Get the orientation of the ef wrt 0 (euler angles ZYZ) from Xef_0
phief_0=

% TODO: Desired EF pose wrt 0.
Hd_0=

% TODO: Desired position EF wrt 0 from Hd_0
xd_0=

% TODO: Desired orientation EF wrt 0 from Hd_0
Rd_0=

% Transform Rotation to euler angles ZYZ
phid_0=R2_euler_zyz(Rd_0);

%TODO: Position Errors EF wrt Link 0 (3x1)
Dxef_0=xd_0-x7_0;

%TODO: Orientation Errors EF wrt Link 0 (Euler Angles ZYZ 3x1) 
Dphi_0=

%TODO: Pose Errors EF wrt Link 0 (6x1)
DXef=

% link 7 (EF) Geometric Jacobian
J7_0=J_EF_kukaIIWA7(Q,L);


% Analytic Jacobian link 7 (EF)
%Get the Euler angles to angular velocity transformation
%Use the function Tf.m
T=Tf(...);

%TODO: Compute the Analytic Jacobin using the the Geometric Jacobian
B=
JA7_0=

% We select the m rows of the Analytic Jacobian
J=JA7_0(1:m,:);

% Position Error. Depends on the task dimension m.
DX=DXef(1:m,1);

% Inverse differential kinematics
%TODO: calculate the inverse differential kinematics
Ji=

%% Secondary Task (Time-variying Joint Posture)
% Joint Posture Task
Qd=deg2rad([-5;30;-4;-60;0;50;0]);

T=5;
w=2*pi/T;
A=deg2rad(30);

Qdt=A*sin(w*t)+Qd;
Qdtp=A*w*cos(w*t);

Kqp=55*eye(dof);
Kqd=0.1*eye(dof);

% Joint Position Error
%TODO: define the joint position error
Dq=

% Joint Velocity Error
%TODO: define the joint velocity error
Dqp=

% Null-space projector
% TODO: Define the null-space projector of the matrix Ji 
N=

% TODO: Compute the Null-space velocity using the Secondary task and N
Qpn=

%% Commanded Joint Velocity
% Proportional gain with the correct dimension according to the task
% dimension m
KP=kp*eye(m);

% Commanded Joint Velocity
% TODO: compute the commanded joint velocity Qp using the Task error DX
if ctrl_type
    % with secondary task (Null-space)
    Qp=
else
    % without secondary task
    Qp=
end

% Output commanded Joint Velocity and end effector velocity (6x1) wrt 0
Qpout=[Qp;Xef_0;Dq];

end

