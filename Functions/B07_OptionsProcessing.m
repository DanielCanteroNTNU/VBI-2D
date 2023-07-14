function [Calc,Veh,Beam] = B07_OptionsProcessing(Calc,Veh,Beam)

% Processing of input variables, and generating new or changing auxiliary 
% variables accordingly. Definition of default values. Performing some checks 
% on the inputs.

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

% --------------------------------
% ---- Veh structure variable ----
% --------------------------------

% Number of vehicle in the event
Veh(1).Event.num_veh = size(Veh,2);

% Variable velocity flag and checking definitions
for veh_num = 1:Veh(1).Event.num_veh

    Veh(veh_num).Pos.var_vel_flag = 0;
    
    % Definition options:
    % Optoin 1) Constant speed: Defined by .x0 and .vel
    % Optoin 2) Variable speed: Defined by .vel_values and .x0_values
    % Optoin 3) Variable speed: Defined by [.x0, .vel, .a]
    %   Constant speed at approach (.vel) and starts acceleration (.a)
    %   when reaching the bridge
    
    Veh(veh_num).Pos.definition_option = 1;
    if or(isfield(Veh(veh_num).Pos,'vel_values'),isfield(Veh(veh_num).Pos,'x0_values'))
        Veh(veh_num).Pos.definition_option = 2;
    elseif isfield(Veh(veh_num).Pos,'a')
        if Veh(veh_num).Pos.a ~= 0
            Veh(veh_num).Pos.definition_option = 3;
        end % if Veh(veh_num).Pos.a ~= 0
    end % or
    
    % In terms of vel_values and x0_values
    if Veh(veh_num).Pos.definition_option == 2
        Veh(veh_num).Pos.var_vel_flag = 1;
        if or(~isfield(Veh(veh_num).Pos,'vel_values'),~isfield(Veh(veh_num).Pos,'x0_values'))
            error(['Error: Vehicle ',num2str(veh_num),' -> need to define vel_values and x0_values']);
        end % if or
    end % if

    % In terms of a
    if Veh(veh_num).Pos.definition_option == 3
        Veh(veh_num).Pos.var_vel_flag = 1;
        if any([~isfield(Veh(veh_num).Pos,'x0'),~isfield(Veh(veh_num).Pos,'vel'),~isfield(Veh(veh_num).Pos,'a')])
            error(['Error: Vehicle ',num2str(veh_num),' -> need to define x0, v0 and a']);
        end % if any
    end % if
    
    % Checking conflicting definitioins
    if and(isfield(Veh(veh_num).Pos,'vel'),isfield(Veh(veh_num).Pos,'vel_values'))
        error(['Error: Vehicle ',num2str(veh_num),' -> velocity definition either vel or vel_values']);
    end % if and
    if and(isfield(Veh(veh_num).Pos,'x0'),isfield(Veh(veh_num).Pos,'x0_values'))
        error(['Error: Vehicle ',num2str(veh_num),' -> x0 definition either x0 or x0_values']);
    end % if and
    
    % Definition of x0_values and vel_values (if not already)
    if ~isfield(Veh(veh_num).Pos,'x0_values')
        Veh(veh_num).Pos.x0_values = Veh(veh_num).Pos.x0;
    end % if ~isfield(Veh(veh_num).Pos,'x0_values')
    if ~isfield(Veh(veh_num).Pos,'vel_values')
        Veh(veh_num).Pos.vel_values = Veh(veh_num).Pos.vel;
    end % if ~isfield(Veh(veh_num).Pos,'x0_values')

    % Velocity sign
    Veh(veh_num).Pos.vel_sign = sign(Veh(veh_num).Pos.vel_values(1));
    % All velocities should be the same sign
    if ~or(all(sign(Veh(veh_num).Pos.vel_values)>0),all(sign(Veh(veh_num).Pos.vel_values)<0))
        error(['Error: Vehicle ',num2str(veh_num),' -> all vel_values should have the same sign']);
    end % if ~or
    
end % for veh_num = 1:Veh(1).Event.num_veh

% Signs of velocity and x0 should be opposite
for veh_num = 1:Veh(1).Event.num_veh
    if sign(Veh(veh_num).Pos.x0_values(1))==Veh(veh_num).Pos.vel_sign
        if Veh(veh_num).Pos.var_vel_flag == 0
            error(['Error: Vehicle ',num2str(veh_num),' -> the sign of either ''x0'' or ''vel'' is wrong']);
        elseif Veh(veh_num).Pos.var_vel_flag == 1
            error(['Error: Vehicle ',num2str(veh_num),' -> the sign of either ''x0_values(1)'' or ''vel_values(1)'' is wrong']);
        end % if Veh(veh_num).Pos.var_vel_flag == 0
    end % if sign(Veh(veh_num).Pos.x0_values(1))==Veh(veh_num).Pos.vel_sign
end % for veh_num = 1:Veh(1).Event.num_veh

% Default path name for vehicle functions
if ~myIsfield(Veh(1),{'Model',1,'function_path'})
    Veh(1).Model.function_path = 'Vehicle_equations';
end % if ~myIsfield(Veh(1),{'Model',1,'function_path'})

% ---------------------------------
% ---- Beam structure variable ----
% ---------------------------------

% Area [m^2]
Beam.Prop.A = 1;    % Definition not necessary. Beam defined in terms of mass per unit length

% Beam type
if ~myIsfield(Beam,{'Prop',1,'type'})
    Beam.Prop.type = 0;       % Custom definition of beam properties
end % if ~myIsfield(Beam,{'Prop',1,'type'})

% Beam Damping
if isfield(Beam.Prop,'damp_per')
    Beam.Prop.damp_xi = Beam.Prop.damp_per/100;
else
    Beam.Prop.damp_per = 0;
    Beam.Prop.damp_xi = 0;
end % if isfield(Beam,'damp_per')

% Definition of mid-span index
Beam.Mesh.Node.at_mid_span = [];
if mod(Beam.Mesh.Ele.num+1,2) == 1
    Beam.Mesh.Node.at_mid_span = (Beam.Mesh.Ele.num)/2+1;
end % if mod(Beam.Mesh.Ele.num,2) == 0

% Beam Damage Type
if ~myIsfield(Beam,{'Damage',1,'type'})
    Beam.Damage.type = 0;       % No beam damage considered
end % if ~myIsfield(Beam,{'Damage',1,'type'})

% ---------------------------------
% ---- Calc structure variable ----
% ---------------------------------

% ---- Profile ----

% Loading profile option
if Calc.Profile.type == 1
    Calc.Profile.Load.on = 1;
else
    Calc.Profile.Load.on = 0;
end % if Calc.Profile.type == 1

% Checking definition of Profile filename to load
if Calc.Profile.Load.on == 1
    if ~isfield(Calc.Profile.Load,'file_name')
        error('Error: Profile definition -> Profile is to be loaded but no file name is defined')
    end % ~isfield(Calc.Profile.Load,'file_name')
end % if Calc.Profile.Load.on == 1

% Default loading path
if Calc.Profile.Load.on == 1
    if ~isfield(Calc.Profile.Load,'path')
        Calc.Profile.Load.path = 'Profiles\';
    end % if ~isfield(Calc.Profile.Load,'path')
end % if Calc.Profile.Load.on == 1

% Default value of Save.on
if ~myIsfield(Calc.Profile,{'Save',1,'on'})
    Calc.Profile.Save.on = 0;
end % if ~myIsfield(Calc.Profile,{'Save',1,'on'})

% Default saving path
if Calc.Profile.Save.on == 1
    if ~isfield(Calc.Profile.Save,'path')
        Calc.Profile.Save.path = 'Profiles\';
    end % if ~isfield(Calc.Profile.Save,'path')
end % if Calc.Profile.Save.on == 1

% Checking definition of Profile filename to save
if Calc.Profile.Save.on == 1
    if ~isfield(Calc.Profile.Save,'file_name')
        error('Error: Profile definition -> Profile is to be saved but no file name is defined')
    end % ~isfield(Calc.Profile.Save,'file_name')
end % if Calc.Profile.Save.on == 1

% Checking incompatibility of "Save.on" and "Load.on"
if and(Calc.Profile.Load.on == 1, Calc.Profile.Save.on == 1)
	error('Error: Profile load/save -> Profile should not be loaded and saved in the same simulation');
end % if and(Calc.Profile.Load.on == 1, Calc.Profile.Save.on == 1)

% Default profile sampling distance (m)
if ~isfield(Calc.Profile,'dx')
    Calc.Profile.dx = 0.01;
end % if ~isfield(Calc.Profile,'dx')

% For ISO profile generation
if Calc.Profile.type == 2
    
    % ISO 8608:1995 profiles classification limits
    % Limits between [minA, A-B, B-C, C-D, D-E, E-F, F-G, G-H]
    Calc.Profile.Info.Gd_limits = (2.^(3:2:17))*1e-6;
    % Limits as defined in ISO 8608:1995:
    % A = Very Good; (8e-6 <=) Gd < 32e-6
    % B = Good; 32e-6 <= Gd < 128e-6
    % C = Average; 128e-6 <= Gd < 512e-6
    % D = Poor; 512e-6 <= Gd < 2048e-6
    % E = Very Poor; 2048e-6 <= Gd < 8192e-6
    % F = 8192e-6 <= Gd < 32768e-6
    % G = 32768e-6 <= Gd < 131072e-6
    % H = 131072e-6 <= Gd

    % Default value for Calc.Profile.class_var = 0;
    if ~myIsfield(Calc.Profile,{'Opt',1,'class_var'})
        Calc.Profile.Opt.class_var = 0;
    end % if ~myIsfield(Calc.Profile,{'Opt',1,'class_var'})
    
end % if Calc.Profile.type == 2

% Profile spatial frequencies 
if Calc.Profile.type == 2
    Calc.Profile.Spatial_frq.min = 0.01;    % Minimum spatial frequency to consider
    Calc.Profile.Spatial_frq.max = 4;       % Maximum spatial frequency to consider
    Calc.Profile.Spatial_frq.ref = 0.1;     % Reference spatial frequency as defined in ISO 8606
else
    Calc.Profile.Spatial_frq.max = 0;
end % if Calc.Profile.type == 2

% Default window length for moving average filter
if myIsfield(Calc.Profile,{'Opt',1,'MovAvg',1,'on'},1)
    if ~myIsfield(Calc.Profile,{'Opt',1,'MovAv',1,'window_L'})
        Calc.Profile.Opt.MovAvg.window_L = 0.24; % [m]
    end % if ~myIsfield(Calc.Profile,{'Opt',1,'MovAv',1,'window_L'})
end % if myIsfield(Calc.Profile,{'Opt',1,'MovAvg',1,'on'},1)

% Default profile length
if ~isfield(Calc.Profile,'L')
    Calc.Profile.L = 1000; % [m]
end % if ~isfield(Calc.Profile,'L')

% ---- Constants ----

% Default value of gravity
if ~myIsfield(Calc,{'Constant',1,'grav'})
    Calc.Constant.grav = -9.81;          % Gravity [m/s^2]
end % if ~myIsfield(Calc,{'Constant',1,'grav'})

% Default value of numerical tolerance
if ~isfield(Calc.Constant,'tol')
    Calc.Constant.tol = 1e-6;
end % if ~isfield(Calc.Constant,'tol')

% ---- Calculation options ----

% Default value of solver's maximum accurate frequency
if ~myIsfield(Calc,{'Solver',1,'max_accurate_frq'})
    Calc.Solver.max_accurate_frq = 0;  % [Hz]
end % if ~myIsfield(Calc,{'Solver',1,max_accurate_frq})

% Definition of minimum time steps per second
if ~isfield(Calc.Solver,'min_t_steps_per_second')
    Calc.Solver.min_t_steps_per_second = 0;
end % if ~isfield(Calc.Solver,'min_t_steps_per_second')

% Definition of accurate mode inclusion
if ~myIsfield(Calc,{'Solver',1,'min_Beam_modes_considered'})
    Calc.Solver.min_Beam_modes_considered = 1;
end % if ~myIsfield(Calc,{'Solver',1,'min_Beam_modes_considered'})

% Procedure for interaction solution
% Default procedure
if ~myIsfield(Calc,{'Proc',1,'name'})
    Calc.Proc.name = 'Coupled';
end % if ~myIsfield(Calc,{'Proc',1,'name'})
% Procedure code
if strcmp(Calc.Proc.name,'Full_Sim_Iter')
    Calc.Proc.short_name = 'FI';
    Calc.Proc.code = 1;
elseif strcmp(Calc.Proc.name,'StepByStep_Iter')
    Calc.Proc.short_name = 'SSI';
    Calc.Proc.code = 2;
elseif strcmp(Calc.Proc.name,'Coupled')
    Calc.Proc.short_name = 'Coup';
    Calc.Proc.code = 3;
end % if strcmp(Calc.Proc.name,'Full_Sim_Iter');

% FI default Iteration variables
if Calc.Proc.code == 1
    % Reset logical flag
    if ~myIsfield(Calc,{'Proc',1,'Iter',1,'continue'})
        Calc.Proc.Iter.continue = 1;
    end % if ~myIsfield(Calc,{'Proc',1,'Iter',1,'continue'})
end % if Calc.Proc.code == 1

% FI and SSI default Iteration variables
if or(Calc.Proc.code == 1,Calc.Proc.code == 2)

    % Creation of Iter subfield
    if ~isfield(Calc.Proc,'Iter')
        Calc.Proc.Iter = struct;
    end % if ~isfield(Calc.Proc,'Iter')
    
    % Default maximum number of iterations
    if ~isfield(Calc.Proc.Iter,'max_num')
        Calc.Proc.Iter.max_num = 10;
    end % if ~isfield(Calc.Proc.Iter,'max_num');

    % Default Iterative criteria
    if ~isfield(Calc.Proc.Iter,'criteria')
        %Calc.Proc.Iter.criteria = 1;         % Iteration criteria based on deformation under wheels
        Calc.Proc.Iter.criteria = 2;         % Iteration criteria based on BM of whole beam (Recommended. It is a more restrictive condition)
    end % if ~isfield(Calc.Proc.Iter,'criteria')

    % Iteration criteria text label
    if Calc.Proc.Iter.criteria == 1
        Calc.Proc.Iter.criteria_text = 'Beam deformation under wheels';
    elseif Calc.Proc.Iter.criteria == 2
        Calc.Proc.Iter.criteria_text = 'Whole beam BM';
    end % if Calc.Proc.Iter.criteria == 1

    % Default Iterative process tolerance for stopping criteria
    if ~isfield(Calc.Proc.Iter,'tol')
        Calc.Proc.Iter.tol = Calc.Constant.tol;
    end % if ~isfield(Calc.Proc.Iter,'tol')

% Coupled default values
elseif Calc.Proc.code == 3

    % No additional options/variables needed
    
end % if or(Calc.Proc.code == 1,Calc.Proc.code == 2)

% Initialize iteration counter
if Calc.Proc.code == 1
    if ~isfield(Calc.Proc.Iter,'num')
        Calc.Proc.Iter.num = 0;
    end % if ~isfield(Calc.Proc.Iter,'num')
% elseif Calc.Proc.code == 2    % Done in B48
%     if ~isfield(Calc.Proc.Iter,'num_t_bri')
%         Calc.Proc.Iter.num_t_bri = 0;
%     end % if ~isfield(Calc.Proc.Iter,'num')
end % if Calc.Proc.code == 1

% Initialize Options fields
if ~isfield(Calc,'Opt')
    Calc.Opt = struct;
end % if ~isfield(Calc,'Opt')

% Verbose On/Off
if ~isfield(Calc.Opt,'verbose')
    Calc.Opt.verbose = 0;
end % if isfield(Calc,'verbose')

% Show progress
if ~isfield(Calc.Opt,'show_progress_every_s')
    Calc.Opt.show_progress_every_s = 2;
end % if ~isfield(Calc.Opt,'show_progress_every_s')

% Switch ON/OFF VBI
if ~isfield(Calc.Opt,'VBI')
    Calc.Opt.VBI = 1;   % VBI is ON
elseif Calc.Opt.VBI == 0
    Calc.Proc.Iter.max_num = 0;
end % if ~isfield(Calc.Opt,'VBI')

% Additional free vibration seconds to calculate beam response
if ~isfield(Calc.Opt,'free_vib_seconds')
    Calc.Opt.free_vib_seconds = 0;
end % if ~isfield(Calc.Opt,'free_vib_seconds')

% ---- Other options not mentioned in A01 ----

% Default calculation of Vehicle natural frequencies
if ~isfield(Calc.Opt,'veh_frq')
    Calc.Opt.veh_frq = 1;
end % if ~isfield(Calc.Opt,'veh_frq')

% Default calculation of Beam natural frequencies
if ~myIsfield(Calc,{'Opt',1,'beam_frq'})
    Calc.Opt.beam_frq = 1;
end % if ~myIsfield(Calc,{'Opt',1,'beam_frq'})

% Default calculation of Beam modes of vibration
if ~myIsfield(Calc,{'Opt',1,'beam_modes'})
    Calc.Opt.beam_frq = 1;
    Calc.Opt.beam_modes = 1;
end % if ~myIsfield(Calc,{'Opt',1,'beam_modes'})

% Newmark-Beta scheme
% Calc.Solver.NewMark.damp = 1;     % Damped Newmark-Beta scheme used in Vehicle and Beam solvers
% Default: Average acceleration method (Normal Newmark-beta)
if ~myIsfield(Calc.Solver,{'NewMark',1,'damp'})
    Calc.Solver.NewMark.damp = 0;      
end % if ~myIsfield(Calc.Solver,{'NewMark',1,'damp'})

% Newmark scheme parameters (for vehicle and beam solvers)
if Calc.Solver.NewMark.damp == 0
    % Default: Average (or constant) acceleration method
    Calc.Solver.NewMark.delta = 0.5; 
    Calc.Solver.NewMark.beta = 0.25;
elseif Calc.Solver.NewMark.damp == 1
    % Damped Newmark-Beta scheme
    Calc.Solver.NewMark.delta = 0.6; 
    Calc.Solver.NewMark.beta = 0.3025;
end % if Calc.Solver.NewmMark.damp

% Linear equivalent loads
if ~myIsfield(Calc.Opt,{'linear_eqv_loads',1})
    Calc.Opt.linear_eqv_loads = 0;
end % if ~myIsfield(Calc.Opt,{'linear_eqv_loads',1})

% Include Coriolis effect
% Include additional terms due to chain rule derivation (Default: 1)
if ~isfield(Calc.Opt,'include_coriolis')
    Calc.Opt.include_coriolis = 1;
end % if ~isfield(Calc.Opt,'include_coriolis')

% Default BM calculation method
if ~isfield(Calc.Opt,'calc_mode_BM')
    Calc.Opt.calc_mode_BM = 1;      % The average nodal result is considered
    %Calc.Opt.calc_mode_BM = 0;      % No average value
end % if ~isfield(Calc.Opt,'calc_mode_BM')

% Default Shear calculation method
if ~isfield(Calc.Opt,'calc_mode_Shear')
    Calc.Opt.calc_mode_Shear = 1;      % The average nodal result is considered
    %Calc.Opt.calc_mode_Shear = 0;      % No average value
end % if ~isfield(Calc.Opt,'calc_mode_Shear')

% Default value for calculation of vehicles initial static deformations
if ~isfield(Calc.Opt,'vehInitSta')
    Calc.Opt.vehInitSta = 1;
end % if ~isfield(Calc.Opt,'vehInitSta')

% Default value for calculation of initial beam static deformation (Default: 1)
if ~isfield(Calc.Opt,'beamInitSta')
    Calc.Opt.beamInitSta = 1;      % Calculate Beam initial static deformation
end % if ~isfield(Calc.Opt,'beamInitSta')

% ---- Plotting Options ----

% If no plots are selected
if ~isfield(Calc,'Plot')
    Calc.Plot.NoPlot = 1;
end % if ~isfield(Calc,'Plot')

% ---- End of function ----