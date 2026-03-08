function [P] = Pol5th3DOF( Pini, Pend, tini,tend,t )
%POL5TH Generates a 3DOF trajectory based on a 5th degree polynomial
%function
%   The function calculates a trajectory between the initial position to
%   the final position with a duration of tend-tini
%   Pini:   initial position (3x1)
%   Pend:   final position (3x1)
%   tini:   initial time
%   tend:   final time
%   t:      current time (simulation time)
% Return:
%   P:  position vector (3x1) 


% The polynomial functions are only valid in the range t=[tini-tend].
% Therefore, if t is outside that range it will produce wrong outputs. 
% Then, we need to avoid this by setting the output outside the given
% range, i.e., t<tini and t>tend
if t<tini
    P=Pini;
elseif t>tend
    P=Pend;
else
    
    % TODO: Define the polinomial function p(i)=a0+a1*t+a2*t^2+...+an*t^n for
    % each axis
    
       
    % x axis
    P(1,1)=...;   
    % y axis
    P(2,1)=...;
    % z axis
    P(3,1)=...;
    
end



end


