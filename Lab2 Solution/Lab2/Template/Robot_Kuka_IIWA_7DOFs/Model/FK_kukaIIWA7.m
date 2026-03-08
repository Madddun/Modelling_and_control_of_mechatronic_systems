function Xef_0 = FK_kukaIIWA7(Hef_0)
%FK_KUKAIIWA7 Computes the forward kinematics for the robot KUKA IIWA 7 DOF. Nothing to do here!
%   The FK generates a 6x1 vector where the orientation Ref_0 is
%   represented as ZYZ Euler Angles 
%   Hef_0: HT of end-effector wrt to Link 0
% Return:
%   Xef_0: EF pose vector (6x1) with ZYZ euler angles

% EF position wrt Link 0 from HT
Xef_0(1:3,1)=Hef_0(1:3,4);

% EF rotation matrix wrt Link 0 from HT
Ref_0=Hef_0(1:3,1:3);

% Simple calculation of Euler Angles ZYZ 
Xef_0(4:6,1)=R2_euler_zyz(Ref_0);

end

