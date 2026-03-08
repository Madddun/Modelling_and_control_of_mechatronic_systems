function [HT_0, HT_W] = getAbsoluteHT_abbIRB4(q,L, T0_W)
%GETABSOLUTEHT calculates the absolute Homogeneous transformations wrt the
%robot base link0 and the wcf
% input:
%   q: is the joint position vector 1X4
%   T0_W: transformation (4x4) of the robot base wrt to wcf
% return:
%   HT_0: List with all the absolute HT (4x4) wrt robot base (0)
%   HT_W: List with all the absolute HT (4x4) wrt wcf (w)

% Use the solution from Lab. Assignment 1

%GETABSOLUTEHT_abbIRB4 calculates the absolute Homogeneous transformations wrt the
%robot base link0 and the wcf
% input:
%   q: is the joint position vector 1X4
%   L: is the kinematic parameter array (see abbIRB4_params.m)
%   T0_W: transformation (4x4) of the robot base wrt to wcf
% return:
%   HT_0: Cell array with all the absolute HT (4x4) wrt robot base
%   HT_W: Cell array with all the absolute HT (4x4) wrt wcf

% Joint postions

q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

% Kinematic Paramters
L1=L(1);
L2=L(2);
L7=L(3);
L8=L(4);
al=L(5);

% Common substitutions

c1=cos(q1);
s1=sin(q1);

c2=cos(q2);
s2=sin(q2);

ca23=cos(al + q2 + q3);
sa23=sin(al + q2 + q3);

s234=sin(q2 + q3 + q4);
c234=cos(q2 + q3 + q4);


% Absolute Homogeneous Transformations
% TODO: Define the absolute HT wrt robot base (link 0)
%relative HT
T0_1 = [ c1   0   -s1   0;
         s1   0    c1   0;
            0     -1      0      L1;
            0      0      0      1 ];
    
    
T1_2 = [  s2   c2   0   L2*s2;
          -c2  s2   0  -L2*c2;
              0        0      1      0;
              0        0      0      1 ];
    
T2_3 = [ cos(q3+al)  -sin(q3+al)  0   L7*cos(q3+al);
         sin(q3+al)   cos(q3+al)  0   L7*sin(q3+al);
              0            0      1        0;
              0            0      0        1 ];

T3_4 = [ -sin(q4-al)  -cos(q4-al)  0   -L8*sin(q4-al);
          cos(q4-al)  -sin(q4-al)  0    L8*cos(q4-al);
              0            0       1        0;
              0            0       0        1 ];

    %absolute HT
T1_0 = T0_1;
T2_0 = T0_1*T1_2;
T3_0 = T0_1*T1_2*T2_3;
T4_0 = T0_1*T1_2*T2_3*T3_4;


% List with all the HT wrt to link 0, starting with H0_0
HT_0={eye(4),T1_0, T2_0, T3_0, T4_0};

%TODO: Create a list with all the HT wrt to the wcf (w), starting with
%Hw_w. In total, HT_W should have 6 elements Hw_w, H0_w, ..., Hef_w.
HT_W={eye(4),              
    T0_W,                
    T0_W * T1_0,        
    T0_W * T2_0,        
    T0_W * T3_0,        
    T0_W * T4_0};


end

