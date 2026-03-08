function Yr_out = Yr_abbIRB4(u)
%YR_ABBIRB4 Computes the Robot regressor

% Joint Position
q1=u(1);
q2=u(2);
q3=u(3);
q4=u(4);


% Joint Velocity
qp1=u(5);
qp2=u(6);
qp3=u(7);
qp4=u(8);


% Joint Velocity reference
qp1r=u(9);
qp2r=u(10);
qp3r=u(11);
qp4r=u(12);


% Joint Acceleration reference
qpp1r=u(13);
qpp2r=u(14);
qpp3r=u(15);
qpp4r=u(16);

%Gravity
gz=u(17);

% Kinematic Parameters
L=abbIRB4_params;

% Angle offset
al=L(5);

% Common substitutions
c22=cos(2*q2);
s22=sin(2*q2);

c2a_22_23=cos(2*al + 2*q2 + 2*q3);
s2a_22_23=sin(2*al + 2*q2 + 2*q3);

sa_22_3=sin(al + 2*q2 + q3);
ca_22_3=cos(al + 2*q2 + q3);

ca3=cos(al + q3);
sa3=sin(al + q3);

s22_23_24=sin(2*q2 + 2*q3 + 2*q4);
c22_23_24=cos(2*q2 + 2*q3 + 2*q4);

c22_3_4=cos(2*q2 + q3 + q4);
s22_3_4=sin(2*q2 + q3 + q4);

s34=sin(q3 + q4);
c34=cos(q3 + q4);

c2=cos(q2);
s2=sin(q2);

sa_22_23_4=sin(al + 2*q2 + 2*q3 + q4);
ca_22_23_4=cos(al + 2*q2 + 2*q3 + q4);

ca23=cos(al + q2 + q3);
sa23=sin(al + q2 + q3);

sam4=sin(al - q4);
cam4=cos(al - q4);

c234=cos(q2 + q3 + q4);
s234=sin(q2 + q3 + q4);

% TODO: Define the robot regressor Yr(q,qp,qpr,qppr)
Yr(1,1)=
Yr(1,2)=
Yr(1,3)=
Yr(1,4)=
Yr(1,5)=
Yr(1,6)=
Yr(1,7)=
...
Yr(n,p)=

% TODO: Define the robot regressor extensio with the viscous friction
% coefficients
Yr_Beta=diag([...]);

% TODO: Extended regressor with robot dynamic model and viscous friction
Yr_ex=[...];

% Regressor serialization
% TODO: define the variables n=DOFs, p=number of parameters
n=??;
p=???;
Yr_out=reshape(Yr_ex,n*p,1);

%To recover the Yr matrix
%Yr_in=reshape(Yr_out,n,p);
end

