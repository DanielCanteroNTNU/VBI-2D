function [Sol] = B29_ForceOnBeam(Calc,Veh,Sol)

% Calculates the force on the beam due to the vehicle(s)

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

for veh_num = 1:Veh(1).Event.num_veh
    
    % Relevant time indices
    inds = Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam;

    % Due to relative wheel displacements (and velocities)
    Sol.Veh(veh_num).Under.onBeamF = ...
        Sol.Veh(veh_num).Wheels.Urel(:,inds).*(Veh(veh_num).Prop.kTk'*ones(1,Calc.Solver.num_t_beam)) + ...
        Sol.Veh(veh_num).Wheels.Vrel(:,inds).*(Veh(veh_num).Prop.cTk'*ones(1,Calc.Solver.num_t_beam));

    % Due to static vehicle load
    Sol.Veh(veh_num).Under.onBeamF = Sol.Veh(veh_num).Under.onBeamF + Veh(veh_num).Static.load*ones(1,Calc.Solver.num_t_beam);

end % for veh_num = 1:Veh.Event.num_veh

% ---- End of function ----