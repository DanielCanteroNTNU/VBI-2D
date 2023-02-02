function [Sol] = B49_Coup_solver(Calc,Veh,Beam)

% Solving the Vehicle-Bridge interaction establishing the coupled system at
% every time step (The coupled procedure).

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

% ---- Auxiliary variables definition ----
% Number of DOF and vehicle DOF index
Coup.DOF.num = Beam.Mesh.DOF.num;
aux1 = 0;
for veh_num = 1:Veh(1).Event.num_veh
    Coup.DOF.num = Coup.DOF.num + Veh(veh_num).DOF(1).num_independent;
    Veh(veh_num).global_ind = aux1 + (1:Veh(veh_num).DOF(1).num_independent);
    aux1 = Veh(veh_num).global_ind(end);
end % for veh_num = 1:Veh(1).Event.num_veh

% ---- Initialize variables ----
UnCoup.SysM.K = sparse(Coup.DOF.num,Coup.DOF.num);
UnCoup.SysM.C = UnCoup.SysM.K;
UnCoup.SysM.M = UnCoup.SysM.K;
UnCoup.SysM.F = sparse(Coup.DOF.num,1);
Coup.Sol.U = zeros(Coup.DOF.num,Calc.Solver.num_t);
Coup.Sol.V = Coup.Sol.U;
Coup.Sol.A = Coup.Sol.U;

% ---- Coupled equations BC ----
Coup.BC.DOF_fixed = Veh(end).global_ind(end) + Beam.BC.DOF_fixed;
Coup.BC.num_DOF_fixed = length(Coup.BC.DOF_fixed);

% ---- Auxiliary variables ----
Aux.disp_every_t = Calc.Opt.show_progress_every_s;
Aux.PCtime_start = clock;
Aux.last_display_time = Aux.disp_every_t;
ele_DOF = Beam.Mesh.Ele.DOF;
global_ind_end = Veh(end).global_ind(end);
NB_cte(1) = 1/(Calc.Solver.NewMark.beta*Calc.Solver.dt^2);
NB_cte(2) = Calc.Solver.NewMark.delta/(Calc.Solver.NewMark.beta*Calc.Solver.dt);
NB_cte(3) = 1/(Calc.Solver.NewMark.beta*Calc.Solver.dt);
NB_cte(4) = (1/(2*Calc.Solver.NewMark.beta)-1);
NB_cte(5) = (1-Calc.Solver.NewMark.delta/Calc.Solver.NewMark.beta);
NB_cte(6) = (1-Calc.Solver.NewMark.delta/(2*Calc.Solver.NewMark.beta))*Calc.Solver.dt;

% ---- Initial vehicle deformations ----
for veh_num = 1:Veh(1).Event.num_veh
    % Self-weight contribution
    Coup.Sol.U(Veh(veh_num).global_ind,1) = Veh(veh_num).SysM.K\(Veh(veh_num).Static.F_vector_no_grav*Calc.Constant.grav)';
    % Profile contribution
    aux1 = Veh(veh_num).SysM.N2w'*(Veh(veh_num).Prop.kTk'.*Veh(veh_num).Pos.wheels_h(:,1));
    Veh(veh_num).U0_prof = Veh(veh_num).SysM.K\aux1;
    Coup.Sol.U(Veh(veh_num).global_ind,1) = Coup.Sol.U(Veh(veh_num).global_ind,1) + Veh(veh_num).U0_prof;
end % for veh_num = 1:Veh(1).Event.num_veh

% ---- Uncoupled Block System Matrices ----
% These are the parts (blocks) of the system matrices that are independent of 
%   the vehicle's position.

% Vehicles contributions
for veh_num = 1:Veh(1).Event.num_veh
    
    % Vehicle's Diagonal block matrices
    UnCoup.SysM.K(Veh(veh_num).global_ind,Veh(veh_num).global_ind) = Veh(veh_num).SysM.K;
    UnCoup.SysM.C(Veh(veh_num).global_ind,Veh(veh_num).global_ind) = Veh(veh_num).SysM.C;
    UnCoup.SysM.M(Veh(veh_num).global_ind,Veh(veh_num).global_ind) = Veh(veh_num).SysM.M;

    % Force vector
    UnCoup.SysM.F(Veh(veh_num).global_ind) = Veh(veh_num).Static.F_vector_no_grav*Calc.Constant.grav;
    
end % for veh_num = 1:Veh(1).Event.num_veh

% Beam contribution
UnCoup.SysM.K(global_ind_end+1:end,global_ind_end+1:end) = Beam.SysM.K;
UnCoup.SysM.C(global_ind_end+1:end,global_ind_end+1:end) = Beam.SysM.C;
UnCoup.SysM.M(global_ind_end+1:end,global_ind_end+1:end) = Beam.SysM.M;

% % Symmetry check
% [sum(sum(abs(UnCoup.SysM.K-UnCoup.SysM.K')))==0,sum(sum(abs(UnCoup.SysM.C-UnCoup.SysM.C')))==0,sum(sum(abs(UnCoup.SysM.M-UnCoup.SysM.M')))==0]

% **** With VBI ***
if Calc.Opt.VBI == 1

    % --------------------------- Time Step Loop ------------------------------
    % Time Step Loop
    for t = 1:Calc.Solver.num_t-1

        % Progress display
        Aux = F01_Progress_Display(Aux,t,Calc.Solver.num_t);

        % ---- Time-dependent system matrices ----

        % Invariant block matrices
        Coup.SysM.K = UnCoup.SysM.K;
        Coup.SysM.C = UnCoup.SysM.C;
        Coup.SysM.M = UnCoup.SysM.M;
        Coup.SysM.F = UnCoup.SysM.F;

        % Reseting of auxiliary arrays
        vals_Kg = [];
        vals_Cg = [];
        vals_Kg_off = [];
        vals_Cg_off = [];
        vals_F = [];
        eq_num1 = [];
        eq_num2 = [];
        eq_num1_off = [];
        eq_num2_off = [];
        eq_numF = [];

        % Vehicles contributions
        for veh_num = 1:Veh(1).Event.num_veh

            kTk = Veh(veh_num).Prop.kTk;
            cTk = Veh(veh_num).Prop.cTk;
            ele_num_t_1 = Veh(veh_num).Pos.elexj(:,t+1);
            N2w_wheels = Veh(veh_num).SysM.N2w;
            h_path_t_1 = Veh(veh_num).Pos.wheels_h(:,t+1);
            hd_path_t_1 = Veh(veh_num).Pos.wheels_hd(:,t+1);
            rows = Veh(veh_num).global_ind;

            for wheel_num = 1:Veh(veh_num).Prop.num_wheels

                % Element to which each wheel belongs to
                ele_num = ele_num_t_1(wheel_num);
                if ele_num > 0
                    % Distance from each x_path to its left node
                    x = Veh(veh_num).Pos.xj(wheel_num,t+1);
                    % Element dimension
                    a = Beam.Mesh.Ele.a(ele_num);
                    % Element shape functions at x
                    shape_fun_at_x = Beam.Mesh.Ele.shape_fun(x,a);
                    shape_fun_at_x_p = Beam.Mesh.Ele.shape_fun_p(x,a)*Calc.Opt.include_coriolis;
                    % Auxiliary matrices
                    NN = shape_fun_at_x*shape_fun_at_x';
                    NNp = shape_fun_at_x*shape_fun_at_x_p';
                    % Addition of suspension porperties to beam
                    cols = global_ind_end+ele_DOF(ele_num,:);
                    eq_num_aux1 = repmat(cols',4,1);
                    eq_num1 = [eq_num1;eq_num_aux1];
                    eq_num_aux2 = repmat(cols,4,1);
                    eq_num2 = [eq_num2;eq_num_aux2(:)];
                    vals_Kg_aux = NN*kTk(wheel_num) + cTk(wheel_num)*Veh(veh_num).Pos.vel_values_t(t)*NNp;
                    vals_Cg_aux = NN*cTk(wheel_num);

                    vals_Kg = [vals_Kg; vals_Kg_aux(:)];
                    vals_Cg = [vals_Cg; vals_Cg_aux(:)];

                    % Off-diagonal block matrices
                    % Vehicle's Node to wheel displacements
                    N2w = N2w_wheels(wheel_num,:);
                    % Off diagonal block matrix
                    OffDiagBlockMat = -(shape_fun_at_x*N2w);
                    OffDiagBlockMat_d = -(shape_fun_at_x_p*N2w)*Veh(veh_num).Pos.vel_values_t(t);

                    % Addition to Coupled stiffness matrix
                    eq_num_aux1 = repmat(rows',4,1);
                    eq_num_aux2 = repmat(cols,Veh(veh_num).DOF(1).num_independent,1);
                    eq_num1_off = [eq_num1_off;eq_num_aux1;eq_num_aux2(:)];
                    eq_num2_off = [eq_num2_off;eq_num_aux2(:);eq_num_aux1];
                    vals_Kg_aux = (OffDiagBlockMat*kTk(wheel_num) + OffDiagBlockMat_d*cTk(wheel_num))';
                    vals_Kg_aux2 = (OffDiagBlockMat*kTk(wheel_num))';
                    % Addition to Coupled damping matrix
                    vals_Cg_aux = (OffDiagBlockMat*cTk(wheel_num))';
                    
                    vals_Kg_off = [vals_Kg_off; vals_Kg_aux(:); vals_Kg_aux2(:)];
                    vals_Cg_off = [vals_Cg_off; vals_Cg_aux(:); vals_Cg_aux(:)];

                    % Force vector
                    vals_F = [vals_F; 0*shape_fun_at_x; ...
                        (kTk(wheel_num)*h_path_t_1(wheel_num) + cTk(wheel_num)*hd_path_t_1(wheel_num))*[N2w';-shape_fun_at_x]];

                    eq_numF = [eq_numF,cols,rows,cols];

                elseif ele_num == 0
                    rows = Veh(veh_num).global_ind;
                    N2w = N2w_wheels(wheel_num,:);
                    % Force vector
                    vals_F = [vals_F; ...
                        (kTk(wheel_num)*h_path_t_1(wheel_num) + cTk(wheel_num)*hd_path_t_1(wheel_num))*N2w'];
                    eq_numF = [eq_numF,rows];
                end % if ele_num > 0
            end % for wheel_num = 1:Veh(veh_num).Prop.num_wheels

        end % for veh_num = 1:Veh(1).Event.num_veh

        % Adding the coupling terms as sparse elements according to their position in the whole matrix
        Coup.SysM.K = Coup.SysM.K + sparse(eq_num1,eq_num2,vals_Kg,Coup.DOF.num,Coup.DOF.num) + ...
            sparse(eq_num1_off,eq_num2_off,vals_Kg_off,Coup.DOF.num,Coup.DOF.num);
        Coup.SysM.C = Coup.SysM.C + sparse(eq_num1,eq_num2,vals_Cg,Coup.DOF.num,Coup.DOF.num) + ...
            sparse(eq_num1_off,eq_num2_off,vals_Cg_off,Coup.DOF.num,Coup.DOF.num);
        Coup.SysM.F = Coup.SysM.F + sparse(eq_numF,1,vals_F,Coup.DOF.num,1);

        % Re-apply the boundary conditions
        Coup.SysM.C(Coup.BC.DOF_fixed,:) = 0; Coup.SysM.C(:,Coup.BC.DOF_fixed) = 0;
        Coup.SysM.K(Coup.BC.DOF_fixed,:) = 0; Coup.SysM.K(:,Coup.BC.DOF_fixed) = 0;
        for i = 1:Coup.BC.num_DOF_fixed
            Coup.SysM.K(Coup.BC.DOF_fixed(i),Coup.BC.DOF_fixed(i)) = Beam.BC.DOF_fixed_value;
        end % for i = 1:Coup.BC.num_DOF_fixed
        Coup.SysM.F(Coup.BC.DOF_fixed) = 0;

        % ---- Direct integraion ----

        % -- Newmark-Beta --
        % Effective Stiffness Matrix
        effKg = Coup.SysM.K + NB_cte(1)*Coup.SysM.M + NB_cte(2)*Coup.SysM.C;
        % Newmark-beta scheme
        A = Coup.Sol.U(:,t)*NB_cte(1) + Coup.Sol.V(:,t)*NB_cte(3) + Coup.Sol.A(:,t)*NB_cte(4);
        B = (NB_cte(2)*Coup.Sol.U(:,t) - NB_cte(5)*Coup.Sol.V(:,t) - NB_cte(6)*Coup.Sol.A(:,t));
        Coup.Sol.U(:,t+1) = effKg\(Coup.SysM.F + Coup.SysM.M*A + Coup.SysM.C*B);
        Coup.Sol.V(:,t+1) = NB_cte(2)*Coup.Sol.U(:,t+1) - B;
        Coup.Sol.A(:,t+1) = Coup.Sol.U(:,t+1)*NB_cte(1) - A;

    end % for t = 1:Calc.Solver.num_t-1
    
    % ---- Output generation ----
    % Dividing the results into Sol.Veh and Sol.Beam
    for veh_num = 1:Veh(1).Event.num_veh
        Sol.Veh(veh_num).U = Coup.Sol.U(Veh(veh_num).global_ind,:);
        Sol.Veh(veh_num).V = Coup.Sol.V(Veh(veh_num).global_ind,:);
        Sol.Veh(veh_num).A = Coup.Sol.A(Veh(veh_num).global_ind,:);
        % Removing initial deformation (and adding initial profile)
        Sol.Veh(veh_num).U = Sol.Veh(veh_num).U - Sol.Veh(veh_num).U(:,1)*ones(1,Calc.Solver.num_t) + Veh(veh_num).U0_prof;
    end % for veh_num = 1:Veh(1).Event.num_veh
    Sol.Beam.U.value_DOFt = Coup.Sol.U(global_ind_end+1:end,Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam);
    Sol.Beam.V.value_DOFt = Coup.Sol.V(global_ind_end+1:end,Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam);
    Sol.Beam.A.value_DOFt = Coup.Sol.A(global_ind_end+1:end,Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam);

% **** No VBI ****
elseif Calc.Opt.VBI == 0

    disp('No VBI!!!');
    % -- Solving the vehicles --
    [Sol] = B28_Veh_DynamicCalc_Approach(Calc,Veh,1);
    % -- Interaction force --
    [Sol] = B29_ForceOnBeam(Calc,Veh,Sol);
    % -- Beam Dynamic Simulation --
    [Sol] = B13_Beam_DynamicCalc(Calc,Veh,Beam,Sol);
    
end % if Calc.Opt.VBI == 1

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

function [Aux] = F01_Progress_Display(Aux,t,num_t)

% Display the progress of the calculation

% -- Inputs --
% Aux = Auxiliary structure with fields:
%   .disp_every_t = Display information every X seconds
%   .PCtime_start = Time of start of calculation
%   .last_display_time = When was the information displayed last time

% -- Output --
% Aux = The same structure but with updated values

Aux.PCtime = etime(clock,Aux.PCtime_start);

if Aux.PCtime > Aux.last_display_time
    disp_text = ['Time step ',num2str(t-1),' of ',num2str(num_t),...
        ' (',num2str(round((t-1)/num_t*100,2)),'%)'];
    disp(disp_text);
    Aux.last_display_time = Aux.last_display_time + Aux.disp_every_t;
end % if Aux.PCtime > Aux.last_display_time

% ---- End of script ----