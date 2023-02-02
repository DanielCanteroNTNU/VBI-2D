function [Calc,Sol] = B48_SSI_solver(Calc,Veh,Beam,Sol)

% Solving the Vehicle-Bridge interaction by the Step-by-Sstep iteration (SSI) procedure
% These script basically gathers and arrenges the indivual functions
%   B13 = Beam dynamic analysis
%   B12 = Vehicle dynamic analysis
%   B29 = Force on beam
%   B17 = Deformation under vehicle
%   B36 = Iteration criteria

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

% -- Vehciles Effective stiffness matrix --
for veh_num = 1:Veh(1).Event.num_veh
    Veh(veh_num).SysM.K_eff = Veh(veh_num).SysM.K + Veh(veh_num).SysM.M/(Calc.Solver.NewMark.beta*Calc.Solver.dt^2) + ...
        Calc.Solver.NewMark.delta/(Calc.Solver.NewMark.beta*Calc.Solver.dt)*Veh(veh_num).SysM.C;
end % for veh_num = 1:Veh(1).Event.num_veh

% -- Beam Effective Stiffness Matrix --
Beam.SysM.K_eff = Beam.SysM.K + Beam.SysM.M/(Calc.Solver.NewMark.beta*Calc.Solver.dt^2) + ...
    Calc.Solver.NewMark.delta/(Calc.Solver.NewMark.beta*Calc.Solver.dt)*Beam.SysM.C;

% -- Initialize variables --
Sol.Beam.U.value_DOFt = zeros(Beam.Mesh.DOF.num,Calc.Solver.num_t_beam);
Sol.Beam.A.value_DOFt = Sol.Beam.U.value_DOFt; 
Sol.Beam.V.value_DOFt = Sol.Beam.A.value_DOFt;
for veh_num = 1:Veh(1).Event.num_veh
    Sol.Veh(veh_num).Under.def = zeros(Veh(veh_num).Prop.num_wheels,Calc.Solver.num_t_beam);
    Aux.Veh(veh_num).Under.def_old = Sol.Veh(veh_num).Under.def;
    Sol.Veh(veh_num).Under.vel = Sol.Veh(veh_num).Under.def;
    Sol.Veh(veh_num).Under.onBeamF = Sol.Veh(veh_num).Under.def;
    Sol.Veh(veh_num).Under.onBeamF(:,1) = ...
        Sol.Veh(veh_num).Wheels.Urel(:,Calc.Solver.t0_ind_beam).*Veh(veh_num).Prop.kTk' + ...
        Sol.Veh(veh_num).Wheels.Vrel(:,Calc.Solver.t0_ind_beam).*Veh(veh_num).Prop.cTk' + ...
        + Veh(veh_num).Static.load;
end % for veh_num = 1:Veh(1).Event.num_veh
Calc.Proc.Iter.num_t_bri = zeros(1,Calc.Solver.num_t_beam-1);
Calc.Proc.Iter.max_iter_reached_t_bri = zeros(1,Calc.Solver.num_t_beam-1);
% -- Auxiliary variables
def_diff = zeros(1,Veh(1).Event.num_veh);
BM_value_x = zeros(Beam.Mesh.Node.num,1);
old_BM_value_x = BM_value_x;
F_on_Beam = zeros(Beam.Mesh.DOF.num,1);
show_progress_next = Calc.Opt.show_progress_every_s;

% -- Step by step calculation --
tic;
for t = Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam-1

    t_bri = t-Calc.Solver.t0_ind_beam+1;
    
    % Progress display
    if and(toc>show_progress_next,Calc.Opt.show_progress_every_s>0)
        disp(['step ',num2str(t_bri),' of ',num2str(Calc.Solver.num_t_beam-1),' (',num2str(round(t_bri/(Calc.Solver.num_t_beam-1)*100,2)),'%)']);
        show_progress_next = show_progress_next + Calc.Opt.show_progress_every_s;
    end % if and(toc>show_progress_next,Calc.Opt.show_progress_every_s>0)

    % Iteration loop
    continue_while = 1;
    while continue_while == 1
    
        Calc.Proc.Iter.num_t_bri(t_bri) = Calc.Proc.Iter.num_t_bri(t_bri) + 1;

        % ---- Vehicle System ----
        for veh_num = 1:Veh(1).Event.num_veh
            if t<Veh(veh_num).Pos.t_end_ind_beam
                % Addition of previous deformation to original profile
                h = Veh(veh_num).Pos.wheels_h(:,t+1) + Sol.Veh(veh_num).Under.def(:,t_bri+1);
                hd = Veh(veh_num).Pos.wheels_hd(:,t+1) + Sol.Veh(veh_num).Under.vel(:,t_bri+1);
                % Force Vector (on vehicle)
                F_on_Veh = Veh(veh_num).Prop.kTk'.*h + Veh(veh_num).Prop.cTk'.*hd;
                F_on_Veh = Veh(veh_num).SysM.N2w'*F_on_Veh;
                % Newmark-beta scheme
                A = Sol.Veh(veh_num).U(:,t)/(Calc.Solver.NewMark.beta*Calc.Solver.dt^2) + ...
                        Sol.Veh(veh_num).V(:,t)/(Calc.Solver.NewMark.beta*Calc.Solver.dt) + ...
                        Sol.Veh(veh_num).A(:,t)*(1/(2*Calc.Solver.NewMark.beta)-1);
                B = (Calc.Solver.NewMark.delta/(Calc.Solver.NewMark.beta*Calc.Solver.dt)*Sol.Veh(veh_num).U(:,t) - ...
                        (1-Calc.Solver.NewMark.delta/Calc.Solver.NewMark.beta)*Sol.Veh(veh_num).V(:,t) - ...
                        (1-Calc.Solver.NewMark.delta/(2*Calc.Solver.NewMark.beta))*Calc.Solver.dt*Sol.Veh(veh_num).A(:,t));
                Sol.Veh(veh_num).U(:,t+1) = Veh(veh_num).SysM.K_eff\(F_on_Veh + Veh(veh_num).SysM.M*A + Veh(veh_num).SysM.C*B);
                Sol.Veh(veh_num).V(:,t+1) = Calc.Solver.NewMark.delta/(Calc.Solver.NewMark.beta*Calc.Solver.dt)*Sol.Veh(veh_num).U(:,t+1) - B;
                Sol.Veh(veh_num).A(:,t+1) = Sol.Veh(veh_num).U(:,t+1)/(Calc.Solver.NewMark.beta*Calc.Solver.dt^2) - A;
                % -- Additional Output generation --
                % Wheel displacements
                Sol.Veh(veh_num).Wheels.U(:,t+1) = Veh(veh_num).SysM.N2w*Sol.Veh(veh_num).U(:,t+1);
                % Relative wheel displacements
                Sol.Veh(veh_num).Wheels.Urel(:,t+1) = Sol.Veh(veh_num).Wheels.U(:,t+1) - h;
                % Wheel velocities
                Sol.Veh(veh_num).Wheels.V(:,t+1) = Veh(veh_num).SysM.N2w*Sol.Veh(veh_num).V(:,t+1); 
                % Relative wheel velocities
                Sol.Veh(veh_num).Wheels.Vrel(:,t+1) = Sol.Veh(veh_num).Wheels.V(:,t+1) - hd;
                % -- Force on beam  --
                % Due to relative wheel displacements (and velocities)
                Sol.Veh(veh_num).Under.onBeamF(:,t_bri+1) = Veh(veh_num).Static.load + ...
                    Sol.Veh(veh_num).Wheels.Urel(:,t+1).*Veh(veh_num).Prop.kTk' + ...
                    Sol.Veh(veh_num).Wheels.Vrel(:,t+1).*Veh(veh_num).Prop.cTk';
            end % if t<Veh(veh_num).Pos.t_end_ind_beam
        end % for veh_num = 1:Veh(1).Event.num_veh

        % ---- Beam System ----
        % Force vector (on beam)
        % Reset variable
        F_on_Beam = F_on_Beam*0;
        % Vehicles loop
        for veh_num = 1:Veh(1).Event.num_veh
            % Wheel loop
            for wheel_num = 1:Veh(veh_num).Prop.num_wheels
                if Veh(veh_num).Pos.wheels_on_beam(wheel_num,t+1) == 1
                    F_on_Beam_1wheel = F_on_Beam*0;
                    elexj = Veh(veh_num).Pos.elexj(wheel_num,t+1);
                    xj = Veh(veh_num).Pos.xj(wheel_num,t+1);
                    % Force vector for each individual wheel
                    if elexj > 0
                        elex = elexj; x = xj;
                        % DOFs for the element    
                        ele_DOF = Beam.Mesh.Ele.DOF(elex,:);
                        % Multiplication of nodal displacements by corresponding shape function value
                        ai = Beam.Mesh.Ele.a(elex); 
                        F_on_Beam_1wheel(ele_DOF) = Sol.Veh(veh_num).Under.onBeamF(wheel_num,t_bri+1)* ...
                            [ (ai+2*x)*(ai-x)^2/ai^3;       x*(ai-x)^2/ai^2;   x^2*(3*ai-2*x)/ai^3;      -x^2*(ai-x)/ai^2];
                    end % if elexj(t) > 0
                    F_on_Beam = F_on_Beam + F_on_Beam_1wheel;
                end % if Veh(veh_num).Pos.wheels_on_beam(wheel_num,t+1) == 1
            end % for wheel_num = 1:Veh(veh_num).Prop.num_wheels
        end % for veh_num = 1:Veh(1).Event.num_veh
        % Application of boundary conditoins to force vector
        F_on_Beam(Beam.BC.DOF_fixed,:) = 0;  % Vertical force = 0
        % Newmark-beta scheme
        % Note A ~= Sol.A; A = auxiliary variable; Sol.A = Beam Accelerations
        A = Sol.Beam.U.value_DOFt(:,t_bri)/(Calc.Solver.NewMark.beta*Calc.Solver.dt^2) + ...
                Sol.Beam.V.value_DOFt(:,t_bri)/(Calc.Solver.NewMark.beta*Calc.Solver.dt) + ...
                Sol.Beam.A.value_DOFt(:,t_bri)*(1/(2*Calc.Solver.NewMark.beta)-1);
        B = (Calc.Solver.NewMark.delta/(Calc.Solver.NewMark.beta*Calc.Solver.dt)*Sol.Beam.U.value_DOFt(:,t_bri) - ...
                (1-Calc.Solver.NewMark.delta/Calc.Solver.NewMark.beta)*Sol.Beam.V.value_DOFt(:,t_bri) - ...
                (1-Calc.Solver.NewMark.delta/(2*Calc.Solver.NewMark.beta))*Calc.Solver.dt*Sol.Beam.A.value_DOFt(:,t_bri));
        Sol.Beam.U.value_DOFt(:,t_bri+1) = Beam.SysM.K_eff\(F_on_Beam + Beam.SysM.M*A + Beam.SysM.C*B);
        Sol.Beam.V.value_DOFt(:,t_bri+1) = Calc.Solver.NewMark.delta/(Calc.Solver.NewMark.beta*Calc.Solver.dt)*Sol.Beam.U.value_DOFt(:,t_bri+1) - B;
        Sol.Beam.A.value_DOFt(:,t_bri+1) = Sol.Beam.U.value_DOFt(:,t_bri+1)/(Calc.Solver.NewMark.beta*Calc.Solver.dt^2) - A;
        % Displacements under vehicle
        for veh_num = 1:Veh(1).Event.num_veh
            for wheel_num = 1:Veh(veh_num).Prop.num_wheels
                beam_x_path_i = Veh(veh_num).Pos.wheels_x(wheel_num,t+1);
                is_on_beam = Veh(veh_num).Pos.wheels_on_beam(wheel_num,t+1);
                beam_x_path_i(beam_x_path_i<0) = 0;
                beam_x_path_i(beam_x_path_i>Beam.Prop.Lb) = Beam.Prop.Lb;
                % Element to which each of x_path belong to
                ele_num = sum(((ones(Beam.Mesh.Node.num,1)*beam_x_path_i) - Beam.Mesh.Ele.acum')>0);
                ele_num(ele_num == 0) = 1;
                ele_num(ele_num > Beam.Mesh.Ele.num) = Beam.Mesh.Ele.num;
                % Distance from each x_path to its left node
                x = beam_x_path_i - Beam.Mesh.Ele.acum(ele_num);
                % Element shape functions at x
                a = Beam.Mesh.Ele.a(ele_num);
                shape_fun_at_x = Beam.Mesh.Ele.shape_fun(x,a)';
                shape_fun_at_x_p = Beam.Mesh.Ele.shape_fun_p(x,a)'*Calc.Opt.include_coriolis;
                % Time loop
                Sol.Veh(veh_num).Under.def(wheel_num,t_bri+1) = ...
                    shape_fun_at_x*Sol.Beam.U.value_DOFt(Beam.Mesh.Ele.DOF(ele_num,:),t_bri+1)*is_on_beam;
                Sol.Veh(veh_num).Under.vel(wheel_num,t_bri+1) = ...
                    shape_fun_at_x*Sol.Beam.V.value_DOFt(Beam.Mesh.Ele.DOF(ele_num,:),t_bri+1)*is_on_beam + ... 
                    shape_fun_at_x_p*Sol.Beam.U.value_DOFt(Beam.Mesh.Ele.DOF(ele_num,:),t_bri+1)*...
                    Veh(veh_num).Pos.vel_values_t(t)*is_on_beam;
            end % for wheel_num = 1:Veh(veh_num).Prop.num_wheels
        end % for veh_num = 1:Veh(1).Event.num_veh
        
        % VBI option
        if Calc.Opt.VBI == 1
            
            % ---- Bending Moment ----
            if Calc.Proc.Iter.criteria == 2
                BM_value_x = BM_value_x*0;
                % ---- NO average nodal values ----
                if Calc.Opt.calc_mode_BM == 0
                    for ix = 1:Beam.Mesh.Node.num-1
                        aux1 = B30_Beam_ele_H(Beam.Mesh.Ele.a(ix),Beam.Prop.E_n(ix),Beam.Prop.I_n(ix));
                        BM_value_x(ix) = aux1(1) * Sol.Beam.U.value_DOFt((ix*2-1):(ix*2-1)+3,t_bri+1);
                    end % for ix
                    ix = Beam.Mesh.Node.num;
                    aux1 = B30_Beam_ele_H(Beam.Mesh.Ele.a(ix-1),Beam.Prop.E_n(ix-1),Beam.Prop.I_n(ix-1));
                    BM_value_x(ix) = aux1(2) * Sol.Beam.U.value_DOFt(((ix-1)*2-1):((ix-1)*2-1)+3,t_bri+1);
                % ---- AVERAGE nodal values ----
                elseif Calc.Opt.calc_mode_BM == 1
                    for ix = 1:Beam.Mesh.Node.num-1
                        BM_value_x([1,2]+(ix-1)) = BM_value_x([1,2]+(ix-1)) + ...
                            B30_Beam_ele_H(Beam.Mesh.Ele.a(ix),Beam.Prop.E_n(ix),Beam.Prop.I_n(ix)) * ... 
                            Sol.Beam.U.value_DOFt((ix*2-1):(ix*2-1)+3,t_bri+1);
                    end % for ix
                    % Average of nodes with multiple calculations
                    BM_value_x(2:end-1,:) = BM_value_x(2:end-1,:)/2;
                end % Calc.Opt.calc_mode_BM
            end % if Calc.Proc.Iter.criteria == 2

            % ---- Iteration criteria ----
            % Depending on what iteration criteria
            if Calc.Proc.Iter.criteria == 1
                % -- Deformation under wheels --
                % Difference with previous iteration
                for veh_num = 1:Veh(1).Event.num_veh
                    def_diff(veh_num) = max(abs(Sol.Veh(veh_num).Under.def(:,t_bri+1) - Aux.Veh(veh_num).Under.def_old(:,t_bri+1)));
                end % for veh_num = 1:Veh(1).Event.num_veh
                % Action depending on values
                if or(max(def_diff) < Calc.Proc.Iter.tol,Calc.Proc.Iter.num_t_bri(t_bri) >= Calc.Proc.Iter.max_num)
                    continue_while = 0;
                    if Calc.Proc.Iter.num_t_bri(t_bri) >= Calc.Proc.Iter.max_num
                        Calc.Proc.Iter.max_iter_reached_t_bri(t_bri) = 1;
                        %disp(['Maximum number of iterations reached !!! (',num2str(Calc.Proc.Iter.max_num),' iterations in time step ',num2str(t),')']);
                    end % if Calc.Proc.Iter.num_t_bri(t_bri) >= Calc.Proc.Iter.max_num
                else
                    for veh_num = 1:Veh(1).Event.num_veh
                        Aux.Veh(veh_num).Under.def_old(:,t_bri+1) = Sol.Veh(veh_num).Under.def(:,t_bri+1);
                    end % for veh_num = 1:Veh(1).Event.num_veh
                end % if Calc.Proc.Iter.def_diff < Calc.Proc.Iter.tol
            elseif Calc.Proc.Iter.criteria == 2
                % -- Bending Moment of whole beam --
                % Difference with previous iteration
                BM_diff = max(abs(BM_value_x - old_BM_value_x));
                % Action depending on values
                if or(BM_diff < Calc.Proc.Iter.tol,Calc.Proc.Iter.num_t_bri(t_bri) >= Calc.Proc.Iter.max_num)
                    continue_while = 0;
                    if Calc.Proc.Iter.num_t_bri(t_bri) >= Calc.Proc.Iter.max_num
                        Calc.Proc.Iter.max_iter_reached_t_bri(t_bri) = 1;
                        %disp(['Maximum number of iterations reached !!! (',num2str(Calc.Proc.Iter.max_num),' iterations in time step ',num2str(t),')']);
                    end % if Calc.Proc.Iter.num_t_bri(t_bri) >= Calc.Proc.Iter.max_num
                else
                    old_BM_value_x = BM_value_x;
                end % if Calc.Proc.Iter.BM_diff < Calc.Proc.Iter.tol
                %pause(0.1); disp(['Step: ',num2str(t),' - Iter: ',num2str(Calc.Proc.Iter.num_t_bri(t_bri)),' - diff: ',num2str(BM_diff)]);
            end % if Calc.Proc.Iter.criteria == 1
            
        else
            continue_while = 0;
        end % if Calc.Opt.VBI == 1
        
    end % while k_cont_while == 1
    
end % for t

% Additional output generation
Calc.Proc.Iter.mean_num = mean(Calc.Proc.Iter.num_t_bri);
Calc.Proc.Iter.max_reached_num = sum(Calc.Proc.Iter.max_iter_reached_t_bri);
Calc.Proc.Iter.max_reached_num_per = Calc.Proc.Iter.max_reached_num/(Calc.Solver.num_t_beam-1)*100;

% Displaying information about the procedure
disp('-- Information about SSI procedure --');
disp('Average number of iterations:');
disp([blanks(8),num2str(Calc.Proc.Iter.mean_num)]);
disp('Steps where maximum number of iterations has been reached:');
disp([blanks(8),num2str(Calc.Proc.Iter.max_reached_num),' (',num2str(Calc.Proc.Iter.max_reached_num_per),'%)']);

% ---- End of function ----