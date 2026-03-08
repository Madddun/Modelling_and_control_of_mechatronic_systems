function Qpr = Qpr_abbIRB4(u)
%QPR_ABBIRB4 Compute the Joint velocity reference in the Joint Space

% Joint position
Q=u(1:4);

% Joint Velocity
Qp=u(5:8);

% Time
t=u(9);

%Desired Joint Position
Qd=u(10:13);

%Desired Joint Velocity
Qdp=u(14:17);

%Desired Joint Acceleration
Qdpp=u(18:21);

% Integral Gain Offset (To fine tune Ki)
Kis=diag(u(22:25));

% Itegral DQ
iDeltaQ=u(26:29);

% TODO: Inverse Derivative Gain
invKd=

% Proportional Gain
Kpi=diag(u(34:37));
% TODO: define Kp
Kp=

% Integral Gain
% TODO: define Ki
Ki=

% Joint Position and Velocity Errors
% TODO: Define the error functions
DeltaQ=
DeltaQp=

% PID-Like Control
% TODO: DEfine the joint velocity (acceleration )references to generate a
% PID-like controller in the joint space
qpr=
qppr=

Qpr=[qpr;qppr;DeltaQ;DeltaQp;Qd;Qdp;Qdpp];

end

