function [Veh] = Sprung_mass(Veh)
% Single DOF vehicle or Sprung mass model
% Created: 28-Dec-2022 11:15:23

% Note: this script was not automatically generated using VeqMon2D

% -- Vehicle information --
% num_axles_per_body = [1];
% num_axles_per_group = [1];
% with_articulation = [];
 
% -- Vehicle variables --
% Veh.Prop.mGj = [mG1]
% Veh.Prop.kTk = [kT1]
% Veh.Prop.cTk = [cT1]
 
% -- Degrees of Freedom --
Veh.DOF(1).name = 'yG1'; Veh.DOF(1).type = 'displacement'; Veh.DOF(1).dependency = 'independent';
Veh.DOF(1).num_independent = 1;
 
% -- DOF relations -- 
Veh.DOF(1).num_dependent = 0;
 
% -- Axle spacing and distance --
Veh.Prop.ax_sp = 0;
Veh.Prop.ax_dist = 0;
 
% -- Vehicle system matrices --
Veh.SysM.M = Veh.Prop.mGj(1);
Veh.SysM.C = Veh.Prop.cTk(1);
Veh.SysM.K = Veh.Prop.kTk(1);

% -- Force vector to calculate static response --
% Note: When using this vector, multiply it by the gravity. Following the sign criteria defined here
%   gravity has negative value. The numerical value to use is grav = -9.81 m/s^2
Veh.Static.F_vector_no_grav = Veh.Prop.mGj(1);
 
% -- Nodal disp. to wheel disp. relation --
Veh.SysM.N2w = 1;
    
% Definition of variables needed for VBI-2D
Veh.Prop.mBi = 0;

% ---- End of function ----