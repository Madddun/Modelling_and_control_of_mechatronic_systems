%% Generate symbolic regressor Yr(q,qp,qpr,qppr) and parameter vector Theta
clear; clc;

%% 1) Symbols: states + references (4DOF used)
syms q1 q2 q3 q4 qp1 qp2 qp3 qp4 real
syms qp1r qp2r qp3r qp4r qpp1r qpp2r qpp3r qpp4r real
syms gz real

Q    = [q1;q2;q3;q4];
Qp   = [qp1;qp2;qp3;qp4];
Qpr  = [qp1r;qp2r;qp3r;qp4r];
Qppr = [qpp1r;qpp2r;qpp3r;qpp4r];
g0   = [0;0;gz];

%% 2) Kinematic parameters (numeric)
% TODO: replace with your IRB120 kinematic parameter function name
L = abbIRB4_params();     % must return numeric kinematic parameters used by kinematics funcs

%% 3) Define SYMBOLIC dynamic parameters (a generic non-minimal set)
% Masses
syms m1 m2 m3 m4 real

% Inertias in course convention (6 independent per link)
syms I111 I112 I113 I122 I123 I133 real
syms I211 I212 I213 I222 I223 I233 real
syms I311 I312 I313 I322 I323 I333 real
syms I411 I412 I413 I422 I423 I433 real

I1 = [I111 I112 I113 I122 I123 I133];
I2 = [I211 I212 I213 I222 I223 I233];
I3 = [I311 I312 I313 I322 I323 I333];
I4 = [I411 I412 I413 I422 I423 I433];

%% 4) Get COM transforms and COM Jacobians
% You need:
% - Absolute HTs of COM frames in base: Tcm{i}_0
% - Geometric Jacobians of COM frames: Jcm{i} = [Jv; Jw] (6x4)

% TODO: replace with your IRB120 COM HT function name
% Expected output example:
%   HTcm_0 is a cell array where HTcm_0{k} is 4x4 transform of CM-k in base
[HTcm_0, ~] = getAbsoluteHTcm_abbIRB4 (Q.', L, eye(4));

Tcm1_0 = HTcm_0{2};
Tcm2_0 = HTcm_0{3};
Tcm3_0 = HTcm_0{4};
Tcm4_0 = HTcm_0{5};

p1 = Tcm1_0(1:3,4);
p2 = Tcm2_0(1:3,4);
p3 = Tcm3_0(1:3,4);
p4 = Tcm4_0(1:3,4);

% TODO: replace with your IRB120 COM Jacobian function name
% Must return 6x4 Jacobians for each link COM, expressed in base frame
[Jcm1, Jcm2, Jcm3, Jcm4] = Jcm_abbIRB4(Q, L);

Jv1 = Jcm1(1:3,:);  Jw1 = Jcm1(4:6,:);
Jv2 = Jcm2(1:3,:);  Jw2 = Jcm2(4:6,:);
Jv3 = Jcm3(1:3,:);  Jw3 = Jcm3(4:6,:);
Jv4 = Jcm4(1:3,:);  Jw4 = Jcm4(4:6,:);

%% 5) Link rotations for inertia mapping to base
% Use CM frame rotations for link 1..4 (or link frames if your convention uses those)
R1 = Tcm1_0(1:3,1:3);
R2 = Tcm2_0(1:3,1:3);
R3 = Tcm3_0(1:3,1:3);
R4 = Tcm4_0(1:3,1:3);

%% 6) Inertia tensors in base
Icm1 = [I1(1) I1(2) I1(3);
        I1(2) I1(4) I1(5);
        I1(3) I1(5) I1(6)];
Icm2 = [I2(1) I2(2) I2(3);
        I2(2) I2(4) I2(5);
        I2(3) I2(5) I2(6)];
Icm3 = [I3(1) I3(2) I3(3);
        I3(2) I3(4) I3(5);
        I3(3) I3(5) I3(6)];
Icm4 = [I4(1) I4(2) I4(3);
        I4(2) I4(4) I4(5);
        I4(3) I4(5) I4(6)];

I1_0 = simplify(R1*Icm1*R1.');
I2_0 = simplify(R2*Icm2*R2.');
I3_0 = simplify(R3*Icm3*R3.');
I4_0 = simplify(R4*Icm4*R4.');

%% 7) M(q)
M = simplify( ...
    m1*(Jv1.'*Jv1) + (Jw1.'*I1_0*Jw1) + ...
    m2*(Jv2.'*Jv2) + (Jw2.'*I2_0*Jw2) + ...
    m3*(Jv3.'*Jv3) + (Jw3.'*I3_0*Jw3) + ...
    m4*(Jv4.'*Jv4) + (Jw4.'*I4_0*Jw4) );

%% 8) G(q)
P = simplify(m1*g0.'*p1 + m2*g0.'*p2 + m3*g0.'*p3 + m4*g0.'*p4);
G = simplify(jacobian(P, Q).');

%% 9) C(q,qp) via Christoffel
C = sym(zeros(4,4));
for i=1:4
    for j=1:4
        cij = sym(0);
        for k=1:4
            cij = cij + sym(1)/2 * ( diff(M(i,j),Q(k)) + diff(M(i,k),Q(j)) - diff(M(k,j),Q(i)) ) * Qp(k);
        end
        C(i,j) = simplify(cij);
    end
end

%% 10) tau_r and regressor wrt Theta
tau_r = simplify(M*Qppr + C*Qpr + G);

Theta = [m1; m2; m3; m4; ...
         I111; I112; I113; I122; I123; I133; ...
         I211; I212; I213; I222; I223; I233; ...
         I311; I312; I313; I322; I323; I333; ...
         I411; I412; I413; I422; I423; I433];

Yr = simplify(jacobian(tau_r, Theta));   % 4 x 28

% sanity check (should be zero)
disp(simplify(tau_r - Yr*Theta));

%% 11) Export MATLAB function for Simulink input vector u
% u = [q; qp; qpr; qppr; gz]
u = [Q; Qp; Qpr; Qppr; gz];

matlabFunction(Yr, 'File', 'Yr_abbIRB120_generated.m', 'Vars', {u});
save('Theta_definition.mat','Theta');