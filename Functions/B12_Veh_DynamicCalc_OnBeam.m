function [Sol] = B12_Veh_DynamicCalc_OnBeam(Calc,Veh,Sol)

% Solving the vehicle(s) dynamic response when located over the beam

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
    
    % -- Effective stiffness matrix --
    eff_K_veh = Veh(veh_num).SysM.K + Veh(veh_num).SysM.M/(Calc.Solver.NewMark.beta*Calc.Solver.dt^2) + ...
        Calc.Solver.NewMark.delta/(Calc.Solver.NewMark.beta*Calc.Solver.dt)*Veh(veh_num).SysM.C;
    
    % -- Force Matrix --
    F_ext = (Veh(veh_num).Prop.kTk'*ones(1,Calc.Solver.num_t_beam)).*Sol.Veh(veh_num).Under.h + ...
            (Veh(veh_num).Prop.cTk'*ones(1,Calc.Solver.num_t_beam)).*Sol.Veh(veh_num).Under.hd;
    F_ext = Veh(veh_num).SysM.N2w'*F_ext;

    % -- Step-by-step calculation --
    for t = Veh(veh_num).Pos.t0_ind_beam:Veh(veh_num).Pos.t_end_ind_beam-1

        ti = t - Calc.Solver.t0_ind_beam + 1;    % "Local" time index for F_ext matrix

        % ---- Vehicle System ----
        % Newmark-beta scheme
        A = Sol.Veh(veh_num).U(:,t)/(Calc.Solver.NewMark.beta*Calc.Solver.dt^2) + ...
                Sol.Veh(veh_num).V(:,t)/(Calc.Solver.NewMark.beta*Calc.Solver.dt) + ...
                Sol.Veh(veh_num).A(:,t)*(1/(2*Calc.Solver.NewMark.beta)-1);
        B = (Calc.Solver.NewMark.delta/(Calc.Solver.NewMark.beta*Calc.Solver.dt)*Sol.Veh(veh_num).U(:,t) - ...
                (1-Calc.Solver.NewMark.delta/Calc.Solver.NewMark.beta)*Sol.Veh(veh_num).V(:,t) - ...
                (1-Calc.Solver.NewMark.delta/(2*Calc.Solver.NewMark.beta))*Calc.Solver.dt*Sol.Veh(veh_num).A(:,t));
        Sol.Veh(veh_num).U(:,t+1) = eff_K_veh\(F_ext(:,ti+1) + Veh(veh_num).SysM.M*A + Veh(veh_num).SysM.C*B);
        Sol.Veh(veh_num).V(:,t+1) = Calc.Solver.NewMark.delta/(Calc.Solver.NewMark.beta*Calc.Solver.dt)*Sol.Veh(veh_num).U(:,t+1) - B;
        Sol.Veh(veh_num).A(:,t+1) = Sol.Veh(veh_num).U(:,t+1)/(Calc.Solver.NewMark.beta*Calc.Solver.dt^2) - A;

    end % for t

    % -- Additional Output generation --
    inds = Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam;
    % Wheel displacements
    Sol.Veh(veh_num).Wheels.U(:,inds) = Veh(veh_num).SysM.N2w*Sol.Veh(veh_num).U(:,inds);
    % Relative wheel displacements
    Sol.Veh(veh_num).Wheels.Urel(:,inds) = ...
        Sol.Veh(veh_num).Wheels.U(:,inds) - Sol.Veh(veh_num).Under.h;
    % Wheel velocities
    Sol.Veh(veh_num).Wheels.V(:,inds) = Veh(veh_num).SysM.N2w*Sol.Veh(veh_num).V(:,inds);
    % Relative wheel velocities
    Sol.Veh(veh_num).Wheels.Vrel(:,inds) = ...
        Sol.Veh(veh_num).Wheels.V(:,inds) - Sol.Veh(veh_num).Under.hd;

end % for veh_num = 1:Veh(1).Event.num_veh

% ---- End of script ----