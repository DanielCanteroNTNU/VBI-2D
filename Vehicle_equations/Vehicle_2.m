function [Veh] = Vehicle_2(Veh)
% Vehicle_2
% Created: 10-Oct-2022 17:55:37
 
% -- Vehicle information --
% num_axles_per_body = [2];
% num_axles_per_group = [1  1];
% with_articulation = [];
 
% -- Vehicle variables --
% Veh.Prop.mBi = [mB1]
% Veh.Prop.IBi = [IB1]
% Veh.Prop.kSj = [[kS1, kS2]]
% Veh.Prop.cSj = [[cS1, cS2]]
% Veh.Prop.mGj = [[mG1, mG2]]
% Veh.Prop.kTk = [[kT1, kT2]]
% Veh.Prop.cTk = [[cT1, cT2]]
% Veh.Prop.dj = [[d1, d2]]
 
% -- Degrees of Freedom --
Veh.DOF(1).name = 'yB1'; Veh.DOF(1).type = 'displacement'; Veh.DOF(1).dependency = 'independent';
Veh.DOF(2).name = 'thetaB1'; Veh.DOF(2).type = 'rotation'; Veh.DOF(2).dependency = 'independent';
Veh.DOF(3).name = 'yG1'; Veh.DOF(3).type = 'displacement'; Veh.DOF(3).dependency = 'independent';
Veh.DOF(4).name = 'yG2'; Veh.DOF(4).type = 'displacement'; Veh.DOF(4).dependency = 'independent';
Veh.DOF(1).num_independent = 4;
 
% -- DOF relations -- 
Veh.DOF(1).num_dependent = 0;
 
% -- Axle spacing and distance --
Veh.Prop.ax_sp = [0, Veh.Prop.dj(2) - Veh.Prop.dj(1)];
Veh.Prop.ax_dist = [0, Veh.Prop.dj(2) - Veh.Prop.dj(1)];
 
% -- Vehicle system matrices --
Veh.SysM.M = ...
    [[Veh.Prop.mBi(1), 0, 0, 0]; ...
    [0, Veh.Prop.IBi(1), 0, 0]; ...
    [0, 0, Veh.Prop.mGj(1), 0]; ...
    [0, 0, 0, Veh.Prop.mGj(2)]];
 
Veh.SysM.C = ...
    [[Veh.Prop.cSj(1) + Veh.Prop.cSj(2), Veh.Prop.cSj(1)*Veh.Prop.dj(1) + Veh.Prop.cSj(2)*Veh.Prop.dj(2), -Veh.Prop.cSj(1), -Veh.Prop.cSj(2)]; ...
    [Veh.Prop.cSj(1)*Veh.Prop.dj(1) + Veh.Prop.cSj(2)*Veh.Prop.dj(2), Veh.Prop.cSj(1)*Veh.Prop.dj(1)^2 + Veh.Prop.cSj(2)*Veh.Prop.dj(2)^2, -Veh.Prop.cSj(1)*Veh.Prop.dj(1), -Veh.Prop.cSj(2)*Veh.Prop.dj(2)]; ...
    [-Veh.Prop.cSj(1), -Veh.Prop.cSj(1)*Veh.Prop.dj(1), Veh.Prop.cSj(1) + Veh.Prop.cTk(1), 0]; ...
    [-Veh.Prop.cSj(2), -Veh.Prop.cSj(2)*Veh.Prop.dj(2), 0, Veh.Prop.cSj(2) + Veh.Prop.cTk(2)]];
 
Veh.SysM.K = ...
    [[Veh.Prop.kSj(1) + Veh.Prop.kSj(2), Veh.Prop.dj(1)*Veh.Prop.kSj(1) + Veh.Prop.dj(2)*Veh.Prop.kSj(2), -Veh.Prop.kSj(1), -Veh.Prop.kSj(2)]; ...
    [Veh.Prop.dj(1)*Veh.Prop.kSj(1) + Veh.Prop.dj(2)*Veh.Prop.kSj(2), Veh.Prop.dj(1)^2*Veh.Prop.kSj(1) + Veh.Prop.dj(2)^2*Veh.Prop.kSj(2), -Veh.Prop.dj(1)*Veh.Prop.kSj(1), -Veh.Prop.dj(2)*Veh.Prop.kSj(2)]; ...
    [-Veh.Prop.kSj(1), -Veh.Prop.dj(1)*Veh.Prop.kSj(1), Veh.Prop.kSj(1) + Veh.Prop.kTk(1), 0]; ...
    [-Veh.Prop.kSj(2), -Veh.Prop.dj(2)*Veh.Prop.kSj(2), 0, Veh.Prop.kSj(2) + Veh.Prop.kTk(2)]];
 
% -- Force vector to calculate static response --
% Note: When using this vector, multiply it by the gravity. Following the sign criteria defined here
%   gravity has negative value. The numerical value to use is grav = -9.81 m/s^2
Veh.Static.F_vector_no_grav = [Veh.Prop.mBi(1), 0, Veh.Prop.mGj(1), Veh.Prop.mGj(2)];
 
% -- Nodal disp. to wheel disp. relation --
Veh.SysM.N2w = ...
    [[0, 0, 1, 0]; ...
    [0, 0, 0, 1]];
 
% ---- End of function ----