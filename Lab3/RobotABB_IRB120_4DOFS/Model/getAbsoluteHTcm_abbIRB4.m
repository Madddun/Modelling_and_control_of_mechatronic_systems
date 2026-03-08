function [HTcm_0, HTcm_W] = getAbsoluteHTcm_abbIRB4(q,L, T0_W)
%GETABSOLUTEHTCM calculates the absolute Homogeneous transformations of
% the cms wrt the robot base link0 and the wcf
% input:
%   q: is the joint position vector 1X4
%   T0_W: transformation (4x4) of the robot base wrt to wcf
% return:
%   HTcm_0: List with all the absolute CM HT (4x4) wrt robot base (0)
%   HTcm_W: List with all the absolute CM HT (4x4) wrt wcf (w)

% Joint postion

q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

% Kinematic Paramters
%    1  2   3  4  5  6   7   8   9  10  11
% p=[L1,L2,L7,L8,al,L11,L21,L31,L32,L41,L51];

L1=L(1);
L2=L(2);
L7=L(3);
al=L(5);
L11=L(6);
L21=L(7);
L41=L(10);
L51=L(11);

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

 L8=L(4);
    % TODO: Define the absolute HT wrt robot base (link 0)
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
    
    %% relative HT cm
    Tcm1_0=[ c1,  -s1,   0,   0;
             s1,   c1,   0,   0;
              0,    0,   1,   L11;
              0,    0,   0,   1 ];
    
    Tcm2_1= [  s2,   c2,   0,   L21*s2;
              -c2,   s2,   0,     -L21*c2;
                0,     0,    1,     0;
                0,     0,    0,      1 ];
    
    Tcm3_2 = [ cos(q3+al),  -sin(q3+al),  0,   L51*cos(q3+al);
                sin(q3+al),   cos(q3+al),  0,   L51*sin(q3+al);
                 0,             0,         1,       0;
                 0,             0,          0,       1 ];
    
    Tcm4_3 = [ -sin(q4-al),  -cos(q4-al),  0,   -L41*sin(q4-al);
                cos(q4-al),  -sin(q4-al),    0,    L41*cos(q4-al);
                 0,             0,          1,        0;
                 0,             0,          0,        1 ];

    
    %%ABSOLUTE ht CM
    Tcm1_0 = Tcm1_0;
    
    
    Tcm2_0 = T1_0*Tcm2_1;
    
    
    Tcm3_0 = T2_0*Tcm3_2;
    
    
    Tcm4_0 = [T3_0*Tcm4_3];

    
    %%ABSOLUTE ht CM
    Tcm1_0 = Tcm1_0;
    
    
    Tcm2_0 = T1_0*Tcm2_1;
    
    
    Tcm3_0 = T2_0*Tcm3_2;
    
    
    Tcm4_0 = [T3_0*Tcm4_3];
    


    % List with all the HT wrt to link 0, starting with H0_0
    HTcm_0 = {eye(4), Tcm1_0, Tcm2_0, Tcm3_0, Tcm4_0};
    
    %TODO: Create a list with all the HT wrt to the wcf (w), starting with
    %Hw_w. In total, HT_W should have 6 elements Hw_w, H0_w, ..., Hef_w.
    HTcm_W = {eye(4), T0_W, T0_W*Tcm1_0, T0_W*Tcm2_0, T0_W*Tcm3_0, T0_W*Tcm4_0};

end

