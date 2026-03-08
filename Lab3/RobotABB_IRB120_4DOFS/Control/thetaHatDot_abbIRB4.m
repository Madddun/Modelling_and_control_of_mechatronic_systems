function Thetap = thetaHatDot_abbIRB4(u)
%THETAHATDOT_ABBIRB4 PArameter update function
%   Calculates the rate of change of the parameter vector Theta used in the
%   robot regressor Thetap=\Gamma^{-1}Yr^{\top}Sq

% Initialization;
% TODO: define the correct dimensions 
Dofs=4;
Theta_size=39;

% Thesholds to stop the update function
SqThreshold_lb=0.00005;
SqThreshold_ub=10.0;

% Joint Velocity Error (from simulink model)
Sq=u(1:4);

% Robot Regressor de-serialization (from simulink model)
% TODO: define the correct ranges, see function Model/Yr_abbIRB4.m and the
% simulink model to define the range for the input vector u

Yr=reshape(u(?:?),Dofs,Theta_size);

% Update factor
gamma_i=1/u(161);
Gamma_i=gamma_i*eye(Theta_size);


% Current time
t=u(162);

% Sq norm
nSq=norm(Sq);

% If Sq is wihtin the thresholds update Theta, else stop the updating
if ((nSq>SqThreshold_lb)&&(nSq<SqThreshold_ub))
    % TODO: define the update function for \Theta
    Thetap=
else
    Thetap=zeros(Theta_size,1);
end

end



