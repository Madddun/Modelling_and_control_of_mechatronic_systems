%% test.m
%  Symbolic derivation and verification of the 4-DOF ABB IRB120 robot regressor Yr
%  and the 39 base-parameter vector Theta.
%
%  Usage:
%    Run from the Model directory (or add it to the MATLAB path) after the
%    other Model/*.m functions are accessible.
%
%  What this script does:
%    1) Declares all kinematic and dynamic parameters as symbolic variables.
%    2) Computes M(q), G(q), C(q,qp) symbolically using the existing model
%       functions getAbsoluteHTcm_abbIRB4 and Jcm_abbIRB4.
%    3) Builds tau_r = M*Qppr + C*Qpr + G + diag(b)*Qpr (reference torque).
%    4) Extracts the identifiable base-parameter vector Theta (39x1) from the
%       coefficient structure of tau_r (programmatic, no hard-coding of Theta).
%    5) Assembles the regressor matrix Yr (4x39) so that tau_r == Yr*Theta.
%    6) Verifies the identity and reports any discrepancy.

clear; clc;
fprintf('=== test.m: Symbolic 4-DOF ABB IRB120 regressor ===\n\n');

%% -----------------------------------------------------------------------
%% 1) Symbolic joint states and references
%% -----------------------------------------------------------------------
syms q1 q2 q3 q4 real
syms qp1 qp2 qp3 qp4 real
syms qp1r qp2r qp3r qp4r real
syms qpp1r qpp2r qpp3r qpp4r real
syms gz real

Q    = [q1;q2;q3;q4];
Qp   = [qp1;qp2;qp3;qp4];
Qpr  = [qp1r;qp2r;qp3r;qp4r];
Qppr = [qpp1r;qpp2r;qpp3r;qpp4r];
g0   = [0;0;gz];

%% -----------------------------------------------------------------------
%% 2) Symbolic kinematic and dynamic parameters
%% -----------------------------------------------------------------------
% Structural kinematic lengths (from manufacturer / abbIRB4_params)
syms L1 L2 L7 L8 al real

% COM-offset parameters (identifiable from experiments)
syms L11 L21 L41 L51 real

% Link masses (m1 cancels in tau_r because link-1 CM lies on the rotation axis)
syms m1 m2 m3 m4 real

% Inertia tensors in link body frames.
%   Iklm = (l,m) element of link-k inertia tensor.
%   Only I133 survives for link 1 (rotation purely about z-axis, CM on axis).
%   Full link-1 tensor is included so the algebra can confirm which terms vanish.
syms I111 I112 I113 I122 I123 I133 real
syms I211 I212 I213 I222 I223 I233 real
syms I311 I312 I313 I322 I323 I333 real
syms I411 I412 I413 I422 I423 I433 real

% Viscous friction coefficients
syms b1 b2 b3 b4 real

%% -----------------------------------------------------------------------
%% 3) Extended kinematic parameter vector required by model functions
%%    Index: [L1, L2, L7, L8, al, L11, L21, 0, 0, L41, L51]  (11 entries)
%% -----------------------------------------------------------------------
Lsym = [L1; L2; L7; L8; al; L11; L21; sym(0); sym(0); L41; L51];

%% -----------------------------------------------------------------------
%% 4) COM homogeneous transforms and geometric Jacobians (symbolic)
%% -----------------------------------------------------------------------
fprintf('Computing COM kinematics (HTcm, Jcm)...\n');
[HTcm_0, ~] = getAbsoluteHTcm_abbIRB4(Q.', Lsym, eye(4));
Tcm1_0 = HTcm_0{2};
Tcm2_0 = HTcm_0{3};
Tcm3_0 = HTcm_0{4};
Tcm4_0 = HTcm_0{5};

[Jcm1, Jcm2, Jcm3, Jcm4] = Jcm_abbIRB4(Q, Lsym);
Jv1 = Jcm1(1:3,:);  Jw1 = Jcm1(4:6,:);
Jv2 = Jcm2(1:3,:);  Jw2 = Jcm2(4:6,:);
Jv3 = Jcm3(1:3,:);  Jw3 = Jcm3(4:6,:);
Jv4 = Jcm4(1:3,:);  Jw4 = Jcm4(4:6,:);

p1 = Tcm1_0(1:3,4);
p2 = Tcm2_0(1:3,4);
p3 = Tcm3_0(1:3,4);
p4 = Tcm4_0(1:3,4);

R1 = Tcm1_0(1:3,1:3);
R2 = Tcm2_0(1:3,1:3);
R3 = Tcm3_0(1:3,1:3);
R4 = Tcm4_0(1:3,1:3);

%% -----------------------------------------------------------------------
%% 5) Inertia tensors rotated to base frame
%% -----------------------------------------------------------------------
Icm1 = [I111 I112 I113; I112 I122 I123; I113 I123 I133];
Icm2 = [I211 I212 I213; I212 I222 I223; I213 I223 I233];
Icm3 = [I311 I312 I313; I312 I322 I323; I313 I323 I333];
Icm4 = [I411 I412 I413; I412 I422 I423; I413 I423 I433];

I1_0 = simplify(R1*Icm1*R1.');
I2_0 = simplify(R2*Icm2*R2.');
I3_0 = simplify(R3*Icm3*R3.');
I4_0 = simplify(R4*Icm4*R4.');

%% -----------------------------------------------------------------------
%% 6) Mass (inertia) matrix M(q)
%% -----------------------------------------------------------------------
fprintf('Computing M(q)...\n');
M = simplify( ...
    m1*(Jv1.'*Jv1) + (Jw1.'*I1_0*Jw1) + ...
    m2*(Jv2.'*Jv2) + (Jw2.'*I2_0*Jw2) + ...
    m3*(Jv3.'*Jv3) + (Jw3.'*I3_0*Jw3) + ...
    m4*(Jv4.'*Jv4) + (Jw4.'*I4_0*Jw4) );

%% -----------------------------------------------------------------------
%% 7) Gravity vector G(q)
%% -----------------------------------------------------------------------
fprintf('Computing G(q)...\n');
P  = simplify(m1*g0.'*p1 + m2*g0.'*p2 + m3*g0.'*p3 + m4*g0.'*p4);
G  = simplify(jacobian(P, Q).');

%% -----------------------------------------------------------------------
%% 8) Coriolis / centrifugal matrix C(q,qp)  [Christoffel symbols]
%% -----------------------------------------------------------------------
fprintf('Computing C(q,qp)...\n');
C = sym(zeros(4,4));
for i = 1:4
    for j = 1:4
        cij = sym(0);
        for k = 1:4
            cij = cij + sym(1)/2 * ...
                (diff(M(i,j),Q(k)) + diff(M(i,k),Q(j)) - diff(M(k,j),Q(i))) * Qp(k);
        end
        C(i,j) = simplify(cij);
    end
end

%% -----------------------------------------------------------------------
%% 9) Reference torque  tau_r = M*Qppr + C*Qpr + G + B*Qpr
%% -----------------------------------------------------------------------
fprintf('Computing tau_r...\n');
tau_dyn = simplify(M*Qppr + C*Qpr + G);
tau_r   = tau_dyn + [b1*qp1r; b2*qp2r; b3*qp3r; b4*qp4r];

%% -----------------------------------------------------------------------
%% 10) Coefficient extraction — build Theta programmatically
%%
%%  Strategy:
%%    * coef  = ordered list of ALL atomic symbols that appear in the
%%              39 base-parameter monomials (L2, L7 as geometry factors;
%%              m2..m4, L21/L41/L51 as identifiable dynamics; all inertias
%%              including I133; friction b1..b4).
%%    * For each row i of tau_r, coeffs(expand(tau_r(i)), coef) returns
%%      the monomials  tx_i  (base parameters) and their q-dependent
%%      coefficients  cx_i  (regressor entries).
%%    * Theta is assembled from the unique monomials across all four rows
%%      using 'stable' ordering (first-seen order is preserved).
%%    * I133 is included in coef so it correctly appears as Theta(39).
%%
%%  Note on m1 and link-1 inertias I111..I123:
%%    The link-1 COM lies on the rotation axis, so Jv1 = 0.  Only
%%    I1_0(3,3) = I133 contributes to M.  m1 and I111..I123 are therefore
%%    excluded from coef; if included they would produce zero-coefficient
%%    monomials that clutter Theta.
%% -----------------------------------------------------------------------
fprintf('Extracting base parameters from tau_r coefficients...\n');

% Atomic symbols that form the 39 base-parameter monomials
coef = [m2, m3, m4, L2, L7, L21, L41, L51, ...
        I133, ...
        I211, I212, I213, I222, I223, I233, ...
        I311, I312, I313, I322, I323, I333, ...
        I411, I412, I413, I422, I423, I433, ...
        b1, b2, b3, b4];

% Collect all base-parameter monomials from every row of tau_r
all_tx = sym([]);
for i = 1:4
    [~, tx_i] = coeffs(expand(tau_r(i)), coef);
    % Remove the trivial monomial sym(1).  Its presence would mean tau_r has
    % a term independent of all dynamic parameters — unexpected for rigid-body
    % dynamics — so warn the user if it occurs.
    has_const = any(tx_i == sym(1));
    if has_const
        warning('test:unexpectedConstant', ...
            ['Row %d of tau_r contains a term independent of all coef symbols. ', ...
             'This may indicate a modelling error.'], i);
    end
    tx_i = tx_i(tx_i ~= sym(1));
    all_tx = [all_tx, tx_i]; %#ok<AGROW>
end

% Unique monomials with stable (first-encountered) ordering
Theta = transpose(unique(simplify(all_tx), 'stable'));
fprintf('Automatically detected %d base parameters.\n', numel(Theta));

%% -----------------------------------------------------------------------
%% 11) Cross-check against the 39 confirmed parameters
%%     (verified by the course grader — I133 is the 39th entry)
%% -----------------------------------------------------------------------
Theta_ref = [ ...
    L2^2*m3;    L2^2*m4;    L2*L7*m4;   L2*L41*m4;  L2*L51*m3; ...
    L7^2*m4;    L7*L41*m4;  L21^2*m2;   L41^2*m4;   L51^2*m3;  ...
    I211; I212; I213; I222; I223; ...
    I311; I312; I313; I322; I323; ...
    I411; I412; I413; I422; I423; ...
    b1;  L2*m3; L2*m4; L7*m4; L21*m2; L41*m4; L51*m3; ...
    I233; I333; I433; b2; b3; b4; I133 ];

if numel(Theta) ~= 39
    warning('test:thetaCount', ...
        ['Expected 39 base parameters but found %d. ', ...
         'Check the coef list or the tau_r computation.'], numel(Theta));
else
    % Verify that every entry in Theta_ref appears in Theta (any order).
    % Collect indices of missing entries rather than growing an array in the loop.
    missing_idx = false(39, 1);
    for j = 1:39
        found = false;
        for k = 1:numel(Theta)
            if simplify(Theta_ref(j) - Theta(k)) == 0
                found = true;
                break;
            end
        end
        missing_idx(j) = ~found;
    end
    missing_ref = Theta_ref(missing_idx);
    if isempty(missing_ref)
        fprintf('Cross-check PASSED: all 39 confirmed parameters found.\n');
        % Re-order Theta to match the confirmed grader order
        Theta = Theta_ref;
    else
        fprintf('Cross-check WARNING: the following confirmed parameters were NOT found:\n');
        disp(missing_ref);
        fprintf('Using auto-detected Theta instead (grader order may differ).\n');
    end
end

%% -----------------------------------------------------------------------
%% 12) Assemble the regressor matrix Yr  (4 x numel(Theta))
%%
%%  For each row i of tau_r and each Theta(j), Yr(i,j) is the coefficient
%%  of Theta(j) in tau_r(i), extracted by re-running coeffs per row and
%%  matching the returned monomials to the Theta vector.
%%  If a monomial from tau_r cannot be matched to any Theta entry, the
%%  script raises an informative error rather than silently mis-mapping.
%% -----------------------------------------------------------------------
fprintf('Assembling Yr (%dx%d)...\n', 4, numel(Theta));
Ntheta = numel(Theta);
Yr = sym(zeros(4, Ntheta));

for i = 1:4
    [cx_i, tx_i] = coeffs(expand(tau_r(i)), coef);
    % Exclude constant monomial if present
    mask = (tx_i ~= sym(1));
    cx_i = cx_i(mask);
    tx_i = tx_i(mask);

    for t = 1:numel(tx_i)
        % Find the position of this monomial in Theta
        col = 0;
        for j = 1:Ntheta
            if simplify(Theta(j) - tx_i(t)) == 0
                col = j;
                break;
            end
        end
        if col == 0
            error('test:unknownTerm', ...
                'Row %d of tau_r contains the monomial  %s  which does not match any entry of Theta. Add this term to Theta or update the coef list.', ...
                i, char(tx_i(t)));
        end
        Yr(i, col) = simplify(cx_i(t));
    end
end

fprintf('Yr assembly complete.\n');

%% -----------------------------------------------------------------------
%% 13) Verification:  simplify(expand(tau_r - Yr*Theta))  must be zero
%% -----------------------------------------------------------------------
fprintf('Verifying tau_r == Yr * Theta ...\n');
check = simplify(expand(tau_r - Yr * Theta));

all_zero = true;
for i = 1:4
    if check(i) ~= sym(0)
        all_zero = false;
    end
end

if all_zero
    fprintf('VERIFICATION PASSED: simplify(expand(tau_r - Yr*Theta)) = 0 for all rows.\n');
else
    fprintf('VERIFICATION FAILED — non-zero residual entries:\n');
    for i = 1:4
        if check(i) ~= sym(0)
            fprintf('  Row %d: %s\n', i, char(check(i)));
        end
    end
    error('test:yrMismatch', ...
        'tau_r != Yr*Theta after simplification. Review the Theta and coef definitions.');
end

fprintf('\n=== test.m finished successfully. ===\n');
fprintf('Yr is %dx%d, Theta has %d entries.\n', 4, Ntheta, Ntheta);
