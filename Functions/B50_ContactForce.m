function [Calc,Sol] = B50_ContactForce(Calc,Veh,Beam,Sol)

% Calculates the vehicle contact forces when crossing the beam

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

% Deformation under each wheel
[Sol] = B17_Calc_U_at(Calc,Veh,Beam,Sol);

% Force on Beam (Contact Force)
for veh_num = 1:Veh(1).Event.num_veh

    % On bridge time indices
    inds = Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam;

    % Addition under wheel deformation to original profile --
    Sol.Veh(veh_num).Under.h = Veh(veh_num).Pos.wheels_h(:,inds) + Sol.Veh(veh_num).Under.def*(Calc.Opt.VBI);
    Sol.Veh(veh_num).Under.hd = Veh(veh_num).Pos.wheels_hd(:,inds) + Sol.Veh(veh_num).Under.vel*(Calc.Opt.VBI);

    % Wheels displacements
    Sol.Veh(veh_num).Wheels.U(:,inds) = Veh(veh_num).SysM.N2w*Sol.Veh(veh_num).U(:,inds);
    % Relative wheel displacements
    Sol.Veh(veh_num).Wheels.Urel(:,inds) = ...
        Sol.Veh(veh_num).Wheels.U(:,inds) - Sol.Veh(veh_num).Under.h;
    % Wheel velocities
    Sol.Veh(veh_num).Wheels.V(:,inds) = Veh(veh_num).SysM.N2w*Sol.Veh(veh_num).V(:,inds);
    % Relative wheel velocities
    Sol.Veh(veh_num).Wheels.Vrel(:,inds) = ...
        Sol.Veh(veh_num).Wheels.V(:,inds) - Sol.Veh(veh_num).Under.hd;

    % Force on Beam: Initialized with the static force
    Sol.Veh(veh_num).Under.onBeamF = Veh(veh_num).Static.load*ones(1,Calc.Solver.num_t_beam);

    % Force on Beam: Addition of tyre properties contributions
    Sol.Veh(veh_num).Under.onBeamF = Sol.Veh(veh_num).Under.onBeamF + ...
        (Veh(veh_num).Prop.kTk'*ones(1,Calc.Solver.num_t_beam)).*Sol.Veh(veh_num).Wheels.Urel(:,inds) + ...
        (Veh(veh_num).Prop.cTk'*ones(1,Calc.Solver.num_t_beam)).*Sol.Veh(veh_num).Wheels.Vrel(:,inds);

end % for veh_num = 1:Veh(1).Event.num_veh

% ---- End of script ----