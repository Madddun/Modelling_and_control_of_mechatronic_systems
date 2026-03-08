function Xd = trajGen_abbIRB4Op(u)
%TRAJGEN_ABBIRB4Op Generates a time-varying trajectory for the EF wrt world

% HT base link wrt to world coordiate frame w
HT0_w=eye(4);

% Circle Parameters
% circle's center wrt wcf
c_w=[u(1);u(2);u(3)];

% circle's center wrt base link 0
%TODO: calculate the circle center wrt base link 0
c_0=

% circle's radius
r=u(4);

% Desired EF position wrt world (based on the circle's parameters)
% TODO: calculate the desired EF position using the beginning of the
% circle, i.e. xc(0), yc(0), and zc(0)
xds_w  =

% EF desired linear velocity and acceleration
xdsp_w = zeros(3,1);
Xdspp_w= zeros(3,1);

% Desired EF position wrt base link 0
% TODO: Transform xds_w from wcf to base link 0
xds_0=

% Current simulation Time
t=u(5);

% Kinematic Parameters (Given by the robot manufacturer)
L=abbIRB4_params;

% Initial Joint position (defined in the simulink model)
Qini=u(6:9);

% Initial EF position
% TODO: get the initial EF position from Qini
xef_ini_0=

% Intervals between Trajectories
t1=5; % spline
t2=25;% slow
t3=35;% mid
t4=65;% fast

% Z task
fw=6;
r2=0.015;


if(t<t1) % Spline trajectory
    %TODO: Complete function Pol5th
    [xd_0,xdp_0,xdpp_0] = Pol5th(xef_ini_0,xds_0(1:3),0,t1,t);


elseif((t>=t1)&&(t<t2)) % Circle slow
    T=20.0;
    w=2*pi/T;

    td=t-t1;

    % TODO: x,y, and z ef positions based on a circle with center (c_0(1),c_0(2),
    % c_0(3), radius r, and frequency w
    xd_0(1,1)=
    xd_0(2,1)=
    xd_0(3,1)=

    % TODO: x,y, and z ef velocities based on a circle with center (c_0(1),c_0(2),
    % c_0(3), radius r, and frequency w
    % HINT: time derivative of position function.
    xdp_0(1,1)=
    xdp_0(2,1)=
    xdp_0(3,1)=

    % TODO: x,y, and z ef accelerations based on a circle with center (c_0(1),c_0(2),
    % c_0(3), radius r, and frequency w
    % HINT: time derivative of velocity function.
    xdpp_0(1,1)=
    xdpp_0(2,1)=
    xdpp_0(3,1)=



elseif((t>=t2)&&(t<t3)) % Crown mid
    T=10;
    w=2*pi/T;
    w2=fw*w;

    td=t-t2;

    % TODO: x,and y ef positions based on a crown with circle
    % center (c_0(1),c_0(2)), radius r, and frequency w.
    % z ef position oscillates with a sin function amplitud r2 and freq w2
    xd_0(1,1)=
    xd_0(2,1)=
    xd_0(3,1)=

    % TODO: x,and y ef velocitites based on a crown with circle
    % center (c_0(1),c_0(2)), radius r, and frequency w.
    % z ef velocity oscillates with a cos function amplitud r2 and freq w2
    % HINT: time derivative of position function.
    xdp_0(1,1)=
    xdp_0(2,1)=
    xdp_0(3,1)=

    % TODO: x,and y ef accelerations based on a crown with circle
    % center (c_0(1),c_0(2)), radius r, and frequency w.
    % z ef acceleration oscillates with a sin function amplitud r2 and freq w2
    % HINT: time derivative of velocity function.
    xdpp_0(1,1)=
    xdpp_0(2,1)=
    xdpp_0(3,1)=


elseif((t>=t3)&&(t<t4)) % Crown fast
    T=5;
    w=2*pi/T;
    w2=fw*w;

    td=t-t4;

    % TODO: x,and y ef positions based on a crown with circle
    % center (c_0(1),c_0(2)), radius r, and frequency w.
    % z ef position oscillates with a sin function amplitud 2*r2 and freq w2
    xd_0(1,1)=
    xd_0(2,1)=
    xd_0(3,1)=2*r2*sin(...

    % TODO: x,and y ef velocitites based on a crown with circle
    % center (c_0(1),c_0(2)), radius r, and frequency w.
    % z ef velocity oscillates with a cos function amplitud 2*r2 and freq w2
    % HINT: time derivative of position function.
    xdp_0(1,1)=
    xdp_0(2,1)=
    xdp_0(3,1)=

    % TODO: x,and y ef accelerations based on a crown with circle
    % center (c_0(1),c_0(2)), radius r, and frequency w.
    % z ef acceleration oscillates with a sin function amplitud 2*r2 and freq w2
    % HINT: time derivative of velocity function.
    xdpp_0(1,1)=
    xdpp_0(2,1)=
    xdpp_0(3,1)=


else % Constant position again!
    xd_0=xds_0(1:3,1);
    xdp_0=zeros(3,1);
    xdpp_0=zeros(3,1);
end

Xd=[xd_0;xdp_0;xdpp_0];

end

