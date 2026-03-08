function [P,Pp,Ppp] = Pol5th( Pini, Pend, tini,tend,t )
%POL5TH Generates a n-DOF trajectory based on a 5th degree polynomial
%function
%   The function calculates a trajectory between the initial position to
%   the final position with a duration of tend-tini
%   Pini:   initial position (nx1)
%   Pend:   final position (nx1)
%   tini:   initial time
%   tend:   final time
%   t:      current time (simulation time)
% Return:
%   P:  position vector (nx1)
%   Pp:  velocity vector (nx1)
%   Ppp:  velocity vector (nx1)

% Vector dimension
n=numel(Pini);

% Initialize vectors with the correct size
P=zeros(n,1);
Pp=zeros(n,1);
Ppp=zeros(n,1);

% The polynomial functions are only valid in the range t=[tini-tend].
% Therefore, if t is outside that range it will produce wrong outputs.
% Then, we need to avoid this by setting the output outside the given
% range, i.e., t<tini and t>tend
if t<tini
    P=Pini;
elseif t>tend
    P=Pend;
else

    % TODO: Define the polinomial functions, i.e. calcualte the
    % coefficients ai of:
    % P(i)=a0+a1*t+a2*t^2+...+an*t^n,
    % Pp(i)=a1+2*a2*t+...+n*an*t^(n-1),
    % Ppp(i)=2*a2+...+n*(n-1)*an*t^(n-2), for each axis

    Dt=t-tini;
    Fac=tend-tini;

    r=Dt/Fac;

    r2=r*r;
    r3=r2*r;
    r4=r3*r;
    r5=r4*r;

    % Polynomial function 5th degree for x, xp, and xpp
    

    % Generate the samples for each axis n
    for i=1:n
        % position P(t)
        P(i,1)=

        % velocity Pp(t)
        Pp(i,1)=

        % acceleration Ppp(t)
        Ppp(i,1)=
    end
end


