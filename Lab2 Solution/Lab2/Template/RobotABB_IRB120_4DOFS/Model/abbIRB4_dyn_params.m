function p=abbIRB4_dyn_params
% Kinematic and Dynamic parameters
% TODO: Define the kinematic and dynamic parameters

L1=0.29;
L2=0.27;
L3=0.07;
L4=0.302;
L5=0.072;
L6=0.1;
L7 = sqrt(L3^2 + L4^2);
L8 = 0.172; %TODO
al = atan2(L4, L3); %TODO
L11=L1/2;
L21=L2/2;
L31=L3/2;
L32=L4/2;
L41=L8/2;
L51 = L32/sin(al); %TODO


% Parameters
p=[L1,L2,L7,L8,al,L11,L21,L31,L32,L41,L51];

end

