function [Calc,Sol] = B37_ProfileUpdate(Calc,Veh,Beam,Sol)

% In the first iteration:
%   Initializes necessary variables
% Subsequent iterations:
%   Updates the profile with the beam deformation

% *************************************************************************
% *** Script part of VBI-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% *************************************************************************

% -------------------------------------------------------------------------
% ---- Input ----
% See A00 for description of input variables
% ---- Output ----
% See A00 for description of output variables
% -------------------------------------------------------------------------

if Calc.Proc.Iter.num == 0
    
    % Initialize variables
    for veh_num = 1:Veh(1).Event.num_veh
        Sol.Veh(veh_num).Under.def = zeros(Veh(veh_num).Prop.num_wheels,Calc.Solver.num_t_beam);
        Sol.Veh(veh_num).Under.def_old = Sol.Veh(veh_num).Under.def + 1;
        Sol.Veh(veh_num).Under.vel = zeros(Veh(veh_num).Prop.num_wheels,Calc.Solver.num_t_beam);
    end % for veh_num = 1:Veh(1).Event.num_veh
    Calc.Aux.old_BM_value_xt = zeros(Beam.Mesh.Node.num,Calc.Solver.num_t_beam);
    
end % if Calc.Proc.Iter.num == 0
    
% -- Addition of previous deformation to original profile --
for veh_num = 1:Veh(1).Event.num_veh
    Sol.Veh(veh_num).Under.h = Veh(veh_num).Pos.wheels_h(:,Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam) + ...
        + Sol.Veh(veh_num).Under.def;
    Sol.Veh(veh_num).Under.hd = Veh(veh_num).Pos.wheels_hd(:,Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam) + ...
        + Sol.Veh(veh_num).Under.vel;
end % for veh_num = 1:Veh(1).Event.num_veh

% Iterations counter
Calc.Proc.Iter.num = Calc.Proc.Iter.num + 1;

% ---- End of function ----