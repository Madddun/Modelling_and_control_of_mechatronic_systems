function Jefp=Jp_EF_abbIRB4(q,qp,L)
% Jp_EF_abbIRB4 Computes the time derivative of the EF's geometric Jacobian

% Joint Position
q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

% Joint Velocity
qp1=qp(1);
qp2=qp(2);
qp3=qp(3);
qp4=qp(4);

% Kinematic Parameters
L2=L(2);
L7=L(3);
L8=L(4);
al=L(5);

% Common substitutions
c1=cos(q1);
s1=sin(q1);

c2=cos(q2);
s2=sin(q2);

c234=cos(q2 + q3 + q4);
s234=sin(q2 + q3 + q4);

ca23=cos(al + q2 + q3);
sa23=sin(al + q2 + q3);

e3=L8*s234;
e4=L8*c234;
e1=L7*ca23 - e3;
e2=e4 + L7*sa23;


d1=L2*s2 + e2;
d2=L2*c2 + e1;
d3=e3 - L7*ca23;

% TODO: compute the time derivative of the ef's geometric jacobian. You can
% use the chain rule to compute it, i.e. 
% Jp=\sum_{i=1}^{n}\frac{\partial Jef}{\partial q_i}\dot{q_i}

% Partial derivative Jef wrt q1

% time derivative of Jef
Jefp=




