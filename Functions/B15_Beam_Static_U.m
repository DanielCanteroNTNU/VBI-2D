function [Sol] = B15_Beam_Static_U(Calc,Veh,Beam,Sol)

% Calculates the static deformation of the Beam due to static load of the vehicle(s)

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

% Vehicle loop
for veh_num = 1:Veh(1).Event.num_veh

    % Temporary copy of vehicle force on beam
    Aux.Veh(veh_num).Under.onBeamF = Sol.Veh(veh_num).Under.onBeamF;
    % Definition of static force in time
    Sol.Veh(veh_num).Under.onBeamF = Veh(veh_num).Static.load*ones(1,Calc.Solver.num_t_beam);
    
end % for veh_num = 1:Veh(1).Event.num_veh

% Nodal forces calculation
[F] = B14_EqVertNodalForce(Calc,Veh,Beam,Sol);

% Static beam deformation
Sol.Beam.U_static.value_DOFt = Beam.SysM.K\F;

% Restoring original values of force on beam
for veh_num = 1:Veh(1).Event.num_veh
    Sol.Veh(veh_num).Under.onBeamF = Aux.Veh(veh_num).Under.onBeamF;
end % for veh_num = 1:Veh(1).Event.num_veh

% ---- End of function ----