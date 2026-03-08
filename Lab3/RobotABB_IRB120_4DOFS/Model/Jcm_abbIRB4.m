function [Jcm1,Jcm2,Jcm3,Jcm4] = Jcm_abbIRB4(q,L)
    %JCM CM geometric Jacobians
    %   q: Joint position vector 4x1
    %   L: Dynamic parameters, see abbIRB4_dyn_params.m
    
    % Joint Positions
    q1=q(1);
    q2=q(2);
    q3=q(3);
    q4=q(4);
    
    % Kinematic and dynamic parameters
    L2=L(2);
    L7=L(3);
    al=L(5);
    L21=L(7);
    L41=L(10);
    L51=L(11);
    
    % Common substitutions
    c1=cos(q1);
    s1=sin(q1);
    
    c2=cos(q2);
    s2=sin(q2);
    
    c234=cos(q2 + q3 + q4);
    s234=sin(q2 + q3 + q4);
    
    ca23=cos(al + q2 + q3);
    sa23=sin(al + q2 + q3);
    
    % Jacobians CM
 L1  = L(1);
L11 = L(6);
   %% Joint origins (in base frame)
t0  = [0; 0; 0];
t1  = [0; 0; L1];

t2  = [ L2*c1*s2;
        L2*s1*s2;
        L1 + L2*c2 ];

t3  = [ c1*(L2*s2 + L7*sa23);
        s1*(L2*s2 + L7*sa23);
        L1 + L2*c2 + L7*ca23 ];

%% Joint axes z_{i-1} in base frame
z0 = [0; 0; 1];
z1 = [-s1; c1; 0];
z2 = z1;
z3 = z1;

%% CM positions (in base frame)
pcm1 = [0; 0; L11];

pcm2 = [ L21*c1*s2;
         L21*s1*s2;
         L1 + L21*c2 ];

% CM3: same structure as t3, but replace L7 by L51 for the CM offset
pcm3 = [ c1*(L2*s2 + L51*sa23);
         s1*(L2*s2 + L51*sa23);
         L1 + L2*c2 + L51*ca23 ];

% CM4: same structure as tef, but replace L8 by L41 for the CM offset of link4
pcm4 = [ c1*(L2*s2 + L7*sa23 + L41*c234);
         s1*(L2*s2 + L7*sa23 + L41*c234);
         L1 + L2*c2 + L7*ca23 - L41*s234 ];

%% Helper: build a CM Jacobian with the correct zero columns
    function J = cmJac(pcm, affects)
        % affects: logical [1x4], affects(k)=true if joint k affects this CM
        Jv = sym(zeros(3,4));
        Jw = sym(zeros(3,4));

        % joint 1
        if affects(1)
            Jv(:,1) = cross(z0, (pcm - t0));
            Jw(:,1) = z0;
        end

        % joint 2
        if affects(2)
            Jv(:,2) = cross(z1, (pcm - t1));
            Jw(:,2) = z1;
        end

        % joint 3
        if affects(3)
            Jv(:,3) = cross(z2, (pcm - t2));
            Jw(:,3) = z2;
        end

        % joint 4
        if affects(4)
            Jv(:,4) = cross(z3, (pcm - t3));
            Jw(:,4) = z3;
        end

        J = [Jv; Jw];
    end

%% Which joints affect which CM?
Jcm1 = cmJac(pcm1, [true  false false false]); % link1 CM
Jcm2 = cmJac(pcm2, [true  true  false false]); % link2 CM
Jcm3 = cmJac(pcm3, [true  true  true  false]); % link3 CM
Jcm4 = cmJac(pcm4, [true  true  true  true ]); % link4 CM


 
end

