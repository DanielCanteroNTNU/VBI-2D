% Script (Not a function) peforming auxiliary calculations needed for some 
% of the plots in the FI procedure

% *************************************************************************
% *** Script part of VBI-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% *************************************************************************

if Calc.Proc.code == 1
    if Calc.Proc.Iter.num == 0

        % Initialize some plot variables

        if isfield(Calc.Plot,'P7_Veh_U_iter')
            for veh_num = 1:Veh(1).Event.num_veh
                Sol.Proc.Iter.Veh(veh_num).U = zeros(Veh(veh_num).DOF(1).num_independent,Calc.Solver.num_t_beam,2);
            end % for veh_num = 1:Veh(1).Event.num_veh
        end % if isfield(Calc.Plot,'P7_Veh_U_iter')
        if isfield(Calc.Plot,'P8_Veh_Urel_iter')
            for veh_num = 1:Veh(1).Event.num_veh
                Sol.Proc.Iter.Veh(veh_num).Wheels.Urel = zeros(Veh(veh_num).Prop.num_wheels,Calc.Solver.num_t_beam,2);
            end % for veh_num = 1:Veh(1).Event.num_veh
        end % if isfield(Calc.Plot,'P8_Veh_Urel_iter')
        if isfield(Calc.Plot,'P9_Beam_U_under_Veh_iter')
            for veh_num = 1:Veh(1).Event.num_veh
                Sol.Proc.Iter.Veh(veh_num).def_under = zeros(Veh(veh_num).Prop.num_wheels,Calc.Solver.num_t_beam,2);
            end % for veh_num = 1:Veh(1).Event.num_veh
        end % if isfield(Calc.Plot,'P9_Beam_U_under_Veh_iter')
        if isfield(Calc.Plot,'P14_Interaction_Force_iter')
            for veh_num = 1:Veh(1).Event.num_veh
                Sol.Proc.Iter.Veh(veh_num).onBeamF = zeros(Veh(veh_num).Prop.num_wheels,Calc.Solver.num_t_beam,2);
            end % for veh_num = 1:Veh(1).Event.num_veh
        end % if isfield(Calc.Plot,'P14_Interaction_Force_iter')
        if isfield(Calc.Plot,'P21_Beam_MidSpan_BM_iter')
            Sol.Proc.Iter.BM05_iter = zeros(Calc.Solver.num_t_beam,2);
        end % if isfield(Calc.Plot,'P21_Beam_MidSpan_BM_iter')

    else

        % Variables for particular plots

        if isfield(Calc.Plot,'P7_Veh_U_iter')
            for veh_num = 1:Veh(1).Event.num_veh
                Sol.Proc.Iter.Veh(veh_num).U(:,:,Calc.Proc.Iter.num) = Sol.Veh(veh_num).U(:,Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam);
            end % for veh_num = 1:Veh(1).Event.num_veh
        end % if isfield(Calc.Plot,'P7_Veh_U_iter')
        if isfield(Calc.Plot,'P8_Veh_Urel_iter')
            for veh_num = 1:Veh(1).Event.num_veh
                Sol.Proc.Iter.Veh(veh_num).Wheels.Urel(:,:,Calc.Proc.Iter.num) = Sol.Veh(veh_num).Wheels.Urel(:,Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam);
            end % for veh_num = 1:Veh(1).Event.num_veh
        end % if isfield(Calc.Plot,'P8_Veh_U_iter')
        if isfield(Calc.Plot,'P9_Beam_U_under_Veh_iter')
            for veh_num = 1:Veh(1).Event.num_veh
                Sol.Proc.Iter.Veh(veh_num).def_under(:,:,Calc.Proc.Iter.num) = Sol.Veh(veh_num).Under.def;
            end % for veh_num = 1:Veh(1).Event.num_veh
        end % if isfield(Calc.Plot,'P9_Beam_U_under_Veh_iter')
        if isfield(Calc.Plot,'P14_Interaction_Force_iter')
            for veh_num = 1:Veh(1).Event.num_veh
                Sol.Proc.Iter.Veh(veh_num).onBeamF(:,:,Calc.Proc.Iter.num) = Sol.Veh(veh_num).Under.onBeamF;
            end % for veh_num = 1:Veh(1).Event.num_veh
        end % if isfield(Calc.Plot,'P14_Interaction_Force_iter')
        if and(isfield(Calc.Plot,'P21_Beam_MidSpan_BM_iter'),Calc.Proc.Iter.criteria == 2)
            if ~isempty(Beam.Mesh.Node.at_mid_span)
                Sol.Proc.Iter.BM05_iter(:,Calc.Proc.Iter.num) = Sol.Beam.BM.value_xt(Beam.Mesh.Node.at_mid_span,:);
            end % if ~isempty(Beam.Mesh.Node.at_mid_span)
        end % if and(isfield(Calc.Plot,'P21_Beam_MidSpan_BM_iter'),Calc.Proc.Iter.criteria == 2)

    end % if Calc.Proc.Iter.num == 0
end % if Calc.Proc.code == 1

clear veh_num

% ---- End of script ----