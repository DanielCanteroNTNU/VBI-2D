function [Sol] = B17_Calc_U_at(Calc,Veh,Beam,Sol)

% Function to calculate the vertical displacement of the beam under the
% vehicle(s) wheels

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

% Method selection
%k_method = 1;   % Using linear interpolation (Faster)
k_method = 2;   % Theoretically correct approach, using shape functions (Slower)

% Vehicle loop
for veh_num = 1:Veh(1).Event.num_veh

    if k_method == 1
        
        % -- Faster Alternative -- (Using linear interpolation; command interp2)
        for wheel_num = 1:Veh(veh_num).Prop.num_wheels
            Sol.Veh(veh_num).Under.def(wheel_num,:) = ...
                interp2(Beam.Mesh.Ele.acum,Calc.Solver.t_beam,Sol.Beam.U.value_DOFt(1:2:end,:)',...
                Calc.x_path(wheel_num,Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam), ...
                Calc.Solver.t_beam,'linear',0);
            Sol.Veh(veh_num).Under.vel(wheel_num,:) = ...
                interp2(Beam.Mesh.Ele.acum,Calc.Solver.t_beam,Sol.Beam.V.value_DOFt(1:2:end,:)',...
                Calc.x_path(wheel_num,Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam), ...
                Calc.Solver.t_beam,'linear',0);
        end % for wheel_num = 1:Veh(veh_num).Prop.num_wheels

    elseif k_method == 2
        
        % -- Theoretically correct approach -- (Slower)
        % Calculation of deformation using shape functions and nodal displacements

        % Initialize
        Sol.Veh(veh_num).Under.def = zeros(Veh(veh_num).Prop.num_wheels,Calc.Solver.num_t_beam);
        Sol.Veh(veh_num).Under.vel = zeros(Veh(veh_num).Prop.num_wheels,Calc.Solver.num_t_beam);

        for wheel_num = 1:Veh(veh_num).Prop.num_wheels
            
            beam_x_path_i = Veh(veh_num).Pos.wheels_x(wheel_num,Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam);
            is_on_beam = Veh(veh_num).Pos.wheels_on_beam(wheel_num,Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam);
            beam_x_path_i(beam_x_path_i<0) = 0;
            beam_x_path_i(beam_x_path_i>Beam.Prop.Lb) = Beam.Prop.Lb;

            % Element to which each of x_path belong to
            ele_num = sum(((ones(Beam.Mesh.Node.num,1)*beam_x_path_i) - ...
                (Beam.Mesh.Ele.acum'*ones(1,Calc.Solver.num_t_beam)))>0);
            ele_num(ele_num == 0) = 1;
            ele_num(ele_num > Beam.Mesh.Ele.num) = Beam.Mesh.Ele.num;

            % Distance from each x_path to its left node
            x = beam_x_path_i - Beam.Mesh.Ele.acum(ele_num);

            % Element shape functions at x
            a = Beam.Mesh.Ele.a(ele_num);
            shape_fun_at_x = Beam.Mesh.Ele.shape_fun(x,a)';
            shape_fun_at_x_p = Beam.Mesh.Ele.shape_fun_p(x,a)'*Calc.Opt.include_coriolis;

            % Time loop
            for t = 1:Calc.Solver.num_t_beam
                Sol.Veh(veh_num).Under.def(wheel_num,t) = ...
                    shape_fun_at_x(t,:)*Sol.Beam.U.value_DOFt(Beam.Mesh.Ele.DOF(ele_num(t),:),t);
                Sol.Veh(veh_num).Under.vel(wheel_num,t) = ...
                    shape_fun_at_x(t,:)*Sol.Beam.V.value_DOFt(Beam.Mesh.Ele.DOF(ele_num(t),:),t) + ...
                    shape_fun_at_x_p(t,:)*Sol.Beam.U.value_DOFt(Beam.Mesh.Ele.DOF(ele_num(t),:),t)*...
                    Veh(veh_num).Pos.vel_values_t(Calc.Solver.t0_ind_beam-1+t);
            end % for t = 1:Calc.Solver.num_t_beam
            
            % Removing results when wheel is not on beam
            Sol.Veh(veh_num).Under.def(wheel_num,:) = ...
                Sol.Veh(veh_num).Under.def(wheel_num,:).*is_on_beam;
            Sol.Veh(veh_num).Under.vel(wheel_num,:) = ...
                Sol.Veh(veh_num).Under.vel(wheel_num,:).*is_on_beam;
            
        end % for wheel_num = 1:Veh(veh_num).Prop.num_wheels
    end % if k_method == 1

end % for veh_num = 1:Veh(1).Event.num_veh

% ---- End of function ----