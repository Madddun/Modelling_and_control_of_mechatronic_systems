function qd = trajGen_abbIRB4(u)
%TRAJGEN_ABBIRB4 Generates a time-varying sin trajectory in the joint space

% Desired joint position
Qds=  u(1:4);
Qdsp= u(5:8);
Qdspp=u(9:12);

% Current simulation Time
t=u(13);

% Initial Joint position
Qini=u(14:17);

% Intervals between Trajectories
t1=10;% spline
t2=30;% slow
t3=40;% mid
t4=65;% fast



if(t<t1) % Spline Trajectory
    %TODO: finish the function Pol5th
    [Qd,Qdp,Qdpp] = Pol5th(Qini,Qds,0,t1,t);
    
    
elseif((t>=t1)&&(t<t2)) % Sin slow
    T=20.0;
    w=2*pi/T;
    
    td=t-t1;
    
    % Joint position using a sin function with amplitud Qds, 
    % frequency w, and bias Qds. 
    Qd=Qds*sin(w*td)+Qds;
    % Joint velocity using a sin function with amplitud Qds, 
    % frequency w.
    Qdp=Qds*w*cos(w*td);
    % Joint acceleration using a sin function with amplitud Qds, 
    % frequency w.
    Qdpp=-Qds*w^2*sin(w*td);

    
elseif((t>=t2)&&(t<t3)) % Sin mid
    T=10;
    w=2*pi/T;

    td=t-t2;

    % Joint position using a sin function with amplitud Qds, 
    % frequency w, and bias Qini.
    Qini=Qds*sin(w*t2)+Qds;
    Qd=Qds*sin(w*td)+Qini;
    % Joint velocity using a sin function with amplitud Qds, 
    % frequency w.
    Qdp=Qds*w*cos(w*td);
    % Joint acceleration using a sin function with amplitud Qds, 
    % frequency w.
    Qdpp=-Qds*w^2*sin(w*td);

    
elseif((t>=t3)&&(t<t4)) % Sin fast
    T=5;
    w=2*pi/T;
    
    td=t-t4;
    
    % Joint position using a sin function with amplitud Qds, 
    % frequency w, and bias Qini.
    Qini=Qds*sin(w*t4)+Qds;
    Qd=Qds*sin(w*td)+Qini;
    % Joint velocity using a sin function with amplitud Qds, 
    % frequency w.
    Qdp=Qds*w*cos(w*td);
    % Joint acceleration using a sin function with amplitud Qds, 
    % frequency w.
    Qdpp=-Qds*w^2*sin(w*td);

    
else % Constant position again!
    Qd=Qds;
    Qdp=Qdsp;
    Qdpp=Qdspp;
end


qd=[Qd;Qdp;Qdpp];



end

