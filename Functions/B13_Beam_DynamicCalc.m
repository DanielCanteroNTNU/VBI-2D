function [Sol] = B13_Beam_DynamicCalc(Calc,Veh,Beam,Sol)

% Solving the dynamic problem of a FEM beam due to a moving load

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

% -- Effective Stiffness Matrix --
eff_K_beam = Beam.SysM.K + Beam.SysM.M/(Calc.Solver.NewMark.beta*Calc.Solver.dt^2) + ...
    Calc.Solver.NewMark.delta/(Calc.Solver.NewMark.beta*Calc.Solver.dt)*Beam.SysM.C;

% -- Initialize variables --
Sol.Beam.U.value_DOFt = zeros(Beam.Mesh.DOF.num,Calc.Solver.num_t_beam);
Sol.Beam.A.value_DOFt = Sol.Beam.U.value_DOFt; 
Sol.Beam.V.value_DOFt = Sol.Beam.A.value_DOFt;

% -- Force Matrix --
[F] = B14_EqVertNodalForce(Calc,Veh,Beam,Sol);

% -- Initial Static Calculation --
if isfield(Calc.Opt,'beamInitSta')
    Sol.Beam.U.value_DOFt(:,1) = Beam.SysM.K\F(:,1);
end % if isfield(Calc.Opt,'beamInitSta')

% -- Step by step calculation --
for t = 1:Calc.Solver.num_t_beam-1

    % ---- Beam System ----
    % Newmark-beta scheme
    % Note A ~= Sol.Beam.A; A = auxiliary variable; Sol.Beam.A = Beam Accelerations
    A = Sol.Beam.U.value_DOFt(:,t)/(Calc.Solver.NewMark.beta*Calc.Solver.dt^2) + ...
            Sol.Beam.V.value_DOFt(:,t)/(Calc.Solver.NewMark.beta*Calc.Solver.dt) + ...
            Sol.Beam.A.value_DOFt(:,t)*(1/(2*Calc.Solver.NewMark.beta)-1);
    B = (Calc.Solver.NewMark.delta/(Calc.Solver.NewMark.beta*Calc.Solver.dt)*Sol.Beam.U.value_DOFt(:,t) - ...
            (1-Calc.Solver.NewMark.delta/Calc.Solver.NewMark.beta)*Sol.Beam.V.value_DOFt(:,t) - ...
            (1-Calc.Solver.NewMark.delta/(2*Calc.Solver.NewMark.beta))*Calc.Solver.dt*Sol.Beam.A.value_DOFt(:,t));
    Sol.Beam.U.value_DOFt(:,t+1) = eff_K_beam\(F(:,t+1) + Beam.SysM.M*A + Beam.SysM.C*B);
    Sol.Beam.V.value_DOFt(:,t+1) = Calc.Solver.NewMark.delta/(Calc.Solver.NewMark.beta*Calc.Solver.dt)*Sol.Beam.U.value_DOFt(:,t+1) - B;
    Sol.Beam.A.value_DOFt(:,t+1) = Sol.Beam.U.value_DOFt(:,t+1)/(Calc.Solver.NewMark.beta*Calc.Solver.dt^2) - A;

end % for t

% ---- End of script ----