function [Veh] = Vehicle_1(Veh)
% Vehicle_1
% Created: 10-Oct-2022 17:51:23
 
% -- Vehicle information --
% num_axles_per_body = [1];
% num_axles_per_group = [1];
% with_articulation = [];
 
% -- Vehicle variables --
% Veh.Prop.mBi = [mB1]
% Veh.Prop.kSj = [kS1]
% Veh.Prop.cSj = [cS1]
% Veh.Prop.mGj = [mG1]
% Veh.Prop.kTk = [kT1]
% Veh.Prop.cTk = [cT1]
 
% -- Degrees of Freedom --
Veh.DOF(1).name = 'yB1'; Veh.DOF(1).type = 'displacement'; Veh.DOF(1).dependency = 'independent';
Veh.DOF(2).name = 'yG1'; Veh.DOF(2).type = 'displacement'; Veh.DOF(2).dependency = 'independent';
Veh.DOF(1).num_independent = 2;
 
% -- DOF relations -- 
Veh.DOF(1).num_dependent = 0;
 
% -- Axle spacing and distance --
Veh.Prop.ax_sp = 0;
Veh.Prop.ax_dist = 0;
 
% -- Vehicle system matrices --
Veh.SysM.M = ...
    [[Veh.Prop.mBi(1), 0]; ...
    [0, Veh.Prop.mGj(1)]];
 
Veh.SysM.C = ...
    [[Veh.Prop.cSj(1), -Veh.Prop.cSj(1)]; ...
    [-Veh.Prop.cSj(1), Veh.Prop.cSj(1) + Veh.Prop.cTk(1)]];
 
Veh.SysM.K = ...
    [[Veh.Prop.kSj(1), -Veh.Prop.kSj(1)]; ...
    [-Veh.Prop.kSj(1), Veh.Prop.kSj(1) + Veh.Prop.kTk(1)]];
 
% -- Force vector to calculate static response --
% Note: When using this vector, multiply it by the gravity. Following the sign criteria defined here
%   gravity has negative value. The numerical value to use is grav = -9.81 m/s^2
Veh.Static.F_vector_no_grav = [Veh.Prop.mBi(1), Veh.Prop.mGj(1)];
 
% -- Nodal disp. to wheel disp. relation --
Veh.SysM.N2w = ...
    [0, 1];
 
% ---- End of function ----