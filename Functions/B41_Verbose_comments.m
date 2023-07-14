function [] = B41_Verbose_comments(Calc,Veh,Beam)

% Displays comments on the command window if the Verbose option is selected

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
% No output. Information is displayed on command window (if requested)
% -------------------------------------------------------------------------

if Calc.Opt.verbose == 1

disp('---- Beam ----');    

% Beam damping
disp(['Selected beam damping is: ',num2str(Beam.Prop.damp_per),'%']);

% Printing type of Bc
disp(['Number of supports = ',num2str(Beam.BC.supp_num)]);
for i = 1:Beam.BC.supp_num
    if Beam.BC.vert_stiff(i) == Inf
        aux1 = 'Fixed';
    elseif Beam.BC.vert_stiff(i) == 0
        aux1 = 'Free';
    else
        aux1 = [num2str(Beam.BC.vert_stiff(i)),' N/m'];
    end % if Beam.BC.vert_stiff(i) == Inf
    if Beam.BC.rot_stiff(i) == Inf
        aux2 = 'Fixed';
    elseif Beam.BC.rot_stiff(i) == 0
        aux2 = 'Free';
    else
        aux2 = [num2str(Beam.BC.rot_stiff(i)),' N/rad'];
    end % if Beam.BC.vert_stiff(i) == Inf
    disp([blanks(8),'Support ',num2str(i),' at ',num2str(Beam.BC.loc(i))]);
    disp([blanks(16),'Vertical disp. ',aux1]);
    disp([blanks(16),'Rotation ',aux2]);        
end % for i = 1:Beam.BC.supp_num

% Beam frequencies
if Calc.Opt.beam_frq == 1
    disp('First frequency of beam:');
    disp([blanks(8),num2str(Beam.Modal.w(1)),' (',num2str(Beam.Modal.f(1)),' Hz)']);    
    disp('Second frequency of beam:');    
    disp([blanks(8),num2str(Beam.Modal.w(2)),' (',num2str(Beam.Modal.f(2)),' Hz)']);    
    disp(['Last frequency of beam (Mode ',num2str(Beam.Mesh.DOF.num-Beam.BC.num_DOF_fixed),'):']);
    disp([blanks(8),num2str(Beam.Modal.w(end)),' (',num2str(Beam.Modal.f(end)),' Hz)']);
end % if Calc.Opt.beam_frq == 1

disp('---- Profile ----');

% Length of generated profile
disp(['Length of generated profile = ',num2str(Calc.Profile.L),' m']);

% Road profile type and properties
if Calc.Profile.type == 0
    disp('SMOOTH road profile'); 
elseif Calc.Profile.type == 2
    disp(['ISO class ',Calc.Profile.Info.class,' road profile']); 
    disp([blanks(8),'Gd = ',num2str(Calc.Profile.Info.Gd),' m^3'])
    if Calc.Profile.Opt.class_var == 0
        disp([blanks(8),'No variability']);
    else
        disp([blanks(8),'With variability']);
    end % if Calc.Profile.Opt.class_var == 0
end % if Calc.Profile.type

% Moving average on the profile
if myIsfield(Calc.Profile,{'Opt',1,'MovAvg',1,'on'},1)
    disp('Road with Moving Average Filter');
    disp([blanks(8),'for a window of length ',num2str(round(Calc.Profile.Opt.MovAvg.window_L,2))]);
end % if myIsfield(Calc.Profile,{'Opt',1,'MovAvg',1,'on'},1)

disp('---- Event ----');

% Number of vehicles in the event
disp(['Number of vehicles in event = ',num2str(Veh(1).Event.num_veh)]);

for veh_num = 1:Veh(1).Event.num_veh
    
    disp(['---- Vehicle ',num2str(veh_num),' ----']);
    
    % Vehicle model
    disp(['Vehicle model = ',Veh(veh_num).Model.type]);
    
    % Number of wheels
    disp([blanks(8),'Number of wheels = ',num2str(Veh(veh_num).Prop.num_wheels)]);

    % Vehicle frequencies
    if Veh(veh_num).DOF(1).num_independent == 1
        disp('Vehicle-s natural frequency is:');
        disp([blanks(8),num2str(Veh(veh_num).Modal.w),' (',num2str(Veh(veh_num).Modal.f),' Hz)']);
    elseif Veh(veh_num).DOF(1).num_independent == 2
        disp('Vehicle-s 2 natural frequencies are:');
        disp([blanks(8),num2str(Veh(veh_num).Modal.w(1)),' (',num2str(Veh(veh_num).Modal.f(1)),' Hz)']);
        disp([blanks(8),num2str(Veh(veh_num).Modal.w(2)),' (',num2str(Veh(veh_num).Modal.f(2)),' Hz)']);
    else
        disp(['The Vehicle has ',num2str(Veh(veh_num).DOF(1).num_independent),' frequencies. The first one is:']);
        disp([blanks(8),num2str(Veh(veh_num).Modal.w(1)),' (',num2str(Veh(veh_num).Modal.f(1)),' Hz)']);
    end % if Veh(veh_num).DOF(1).num_independent == 1

    % Vehicle position, velocity and acceleration
    disp('Vehicle position overview');
    disp([blanks(8),'It starts and ends moving at:']);
    disp([blanks(16),'x0 = ',num2str(Veh(veh_num).Pos.x0_values(1)),' m']);
    disp([blanks(16),'x_end = ',num2str(Veh(veh_num).Pos.wheels_x(1,end)),' m']);
    if Veh(veh_num).Pos.var_vel_flag == 0
        disp([blanks(8),'With constant velocity:']);
        disp([blanks(16),'v0 = ',num2str(Veh(veh_num).Pos.vel_values(1)),' m/s']);
    elseif Veh(veh_num).Pos.var_vel_flag == 1
        disp([blanks(8),'With variable velocity:']);
        disp([blanks(16),'v0 = ',num2str(Veh(veh_num).Pos.vel_values),' m/s']);
        disp([blanks(16),'v_end = ',num2str(Veh(veh_num).Pos.vel_values_t(end)),' m/s']);
    end % if Veh(veh_num).Pos.var_vel_flag == 0

end % for veh_num = 1:Veh(1).Event.num_veh

disp('---- Calculation Options ----');

% No VBI 
if Calc.Opt.VBI == 0
    disp('No VBI is considered !!!');
end % if Calc.Opt.VBI == 0

% Printing Type of Newmark-Beta scheme
if Calc.Solver.NewMark.damp == 0
    aux1 = 'No';
elseif Calc.Solver.NewMark.damp == 1
    aux1 = 'Considering';
end % if Calc.Solver.NewMark.damp == 0
disp(['Newmark-Beta integration (',aux1,' numerical damping)']);

% Interaction Solution Procedure
if Calc.Proc.code == 1
    aux1 = 'Full model solution iteration';
elseif Calc.Proc.code == 2
    aux1 = 'Step-by-step iteration';
elseif Calc.Proc.code == 3
    aux1 = 'Coupled solution';
end % if Calc.Proc.code == 1
disp(['Interaction Solution Procedure: ',aux1]);

if any(Calc.Proc.code == [1,2])
    % Iteration criteria
    disp(['Selected iteration criteria is: ',Calc.Proc.Iter.criteria_text]);
    % Numerical and Iteration tolerance
    disp(['Numeric tolerance = ',num2str(Calc.Constant.tol)]);
    disp(['Iterative process tolerance = ',num2str(Calc.Proc.Iter.tol)]);
    % Maximum number of iterations
    disp(['Maximum number of iterations = ',num2str(Calc.Proc.Iter.max_num)]);
elseif Calc.Proc.code == 3
    
end % if Calc.Proc.code == 1

% Calculation of Vehicle frequencies
if Calc.Opt.veh_frq == 0
    disp('Vehicle frequences are NOT calculated');
elseif Calc.Opt.veh_frq == 1
    disp('Vehicle frequences are calculated');
end % if Calc.Opt.veh_frq == 0

% Calcuation of Beam frequencies
if Calc.Opt.beam_frq == 0
    disp('Beam frequences are NOT calculated');
elseif Calc.Opt.beam_frq == 1
    disp('Beam frequences are calculated');
end % if Calc.Opt.beam_frq == 0

% Calculation of Modes of vibration
if Calc.Opt.beam_modes == 0
    disp('Beam Modes of vibration are NOT calculated');
elseif Calc.Opt.beam_modes == 1
    disp('Beam Modes of vibration are calculated');
end % if Calc.Opt.beam_modes == 0

% BM calculation mode
if Calc.Opt.calc_mode_BM == 1
    disp('BM: The average nodal result is considered');
else
    disp('BM: NO average nodal result is considered');
end % if Calc.Opt.calc_mode_BM == 1

% Shear calculation mode
if Calc.Opt.calc_mode_Shear == 1
    disp('Shear: The average nodal result is considered');
else
    disp('Shear: NO average nodal result is considered');
end % if Calc.Opt.calc_mode_Shear == 1

% Selected time step
disp(['Problem solved using ',num2str(Calc.Solver.t_steps_per_second),' steps/second']);
if Calc.Solver.t_steps_criteria == 1
    max_frq = max(Veh(1).Modal.f)*2;
    for veh_num = 2:Veh(1).Event.num_veh
        max_frq = max(max_frq,max(Veh(veh_num).Modal.f)*2);
    end % for veh_num = 2:Veh(1).Event.num_veh
    disp([blanks(8),'to accurately represent the highest Vehicle Frequency (',...
        num2str(round(max_frq,2)),' Hz)']);
elseif Calc.Solver.t_steps_criteria == 2
    disp([blanks(8),'to accurately represent up to Beam mode ',...
        num2str(Calc.Solver.min_Beam_modes_considered),' (',...
        num2str(round(Beam.Modal.f(Calc.Solver.min_Beam_modes_considered),2)),' Hz)']);
elseif Calc.Solver.t_steps_criteria == 3
    disp([blanks(8),'which is the user-defined minimum number']);
elseif Calc.Solver.t_steps_criteria == 4
    disp([blanks(8),'to accurately represent the highest Profile Frequency (',...
        num2str(round(Calc.Profile.Spatial_frq.max*Veh(1).Event.max_vel,2)),' Hz)']);
elseif Calc.Solver.t_steps_criteria == 5
    disp([blanks(8),'to accurately represent the user-defined frequency (',...
        num2str(round(Calc.Solver.max_accurate_frq,2)),' Hz)']);
end % if Calc.Solver.t_steps_criteria == 1

disp('----------------');

end % if Calc.verbose == 1 

% ---- End of function ----