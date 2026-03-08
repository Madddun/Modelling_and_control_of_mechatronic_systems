function p=abbIRB4_params
% abbIRB4_params Kinematic parameters (Simulates the information provided
% by the manufacturer)

L1=0.29;
L2=0.27;
L3=0.07;
L4=0.302;
L5=0.072;
L6=0.1;
L7=sqrt(L3^2 + L4^2);
L8=L5 + L6;
al=atan2(L4,L3);

% Parameters
p=[L1,L2,L7,L8,al];

end


