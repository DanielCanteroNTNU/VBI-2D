% Simulation of the validation examples using VBI-2D

% *************************************************************************
% *** Script part of VBI-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% *************************************************************************

% -------------------------------------------------------------------------
clear; clc; rng('shuffle'); addpath 'Functions'; tic

% ---- Validation ----
Validation.Save.on = 1;
Validation.Save.folder = 'Validation\';
%Validation.Save.name = 'Validation_Numerical_SprungVeh'; Opt.ramp_on = 0; Opt.damp_on = 0; Opt.acc_on = 0;
%Validation.Save.name = 'Validation_Numerical_RampOn'; Opt.ramp_on = 1; Opt.damp_on = 0; Opt.acc_on = 0;
%Validation.Save.name = 'Validation_Numerical_DampOn'; Opt.ramp_on = 1; Opt.damp_on = 1; Opt.acc_on = 0;
%Validation.Save.name = 'Validation_Numerical_AccOn'; Opt.ramp_on = 1; Opt.damp_on = 1; Opt.acc_on = 1;

% -------------------------------------------------------------------------
% ---------------------------- Vehicles -----------------------------------
% -------------------------------------------------------------------------

veh_num = 0;

% -- Sprung mass -- (1-DOF vehicle)
veh_num = veh_num + 1;
Veh(veh_num).Model.type = 'Sprung_mass';% Text with name of model to use
Veh(veh_num).Prop.mGj = 5750;           % Suspension mass [kg]
Veh(veh_num).Prop.kTk = 1595e3;         % Tyre stiffness [N/m]
Veh(veh_num).Prop.cTk = 40e4;           % Tyre viscous damping [N*s/m]
if Opt.damp_on == 0
    Veh(veh_num).Prop.cTk = 0;
end % if Opt.damp_on == 0

% -- Addition to define constant/variable vehicle speed --
Veh(veh_num).Pos.x0 = 0;        % Initial position of 1st axle of vehicle (from left bridge support)
Veh(veh_num).Pos.vel = 25;      % Velocity of vehicle [m/s] (negative value = vehicle moving from right to left)
% Vehicle acceleration
if Opt.acc_on == 1
    Veh(veh_num).Pos.x0 = -1;
    Veh(veh_num).Pos.a = 10;    % Uniform acceleration of the vehicle [m/s^2] (For deceleration use opposite sign of .vel)
end % if Opt.acc_on == 1

clear veh_num

% -------------------------------------------------------------------------
% ------------------------------- Beam ------------------------------------
% -------------------------------------------------------------------------

% -- Properties --
% Alternative definition: Based on tables for "typical" bridges
Beam.Prop.type = 'Y';         % Type of cross-section ('T', 'Y', or 'SY')
Beam.Prop.Lb = 25;            % Beam's length [m]
Beam.Prop.damp_per = 0;       % Damping percentage [%]

% -- Boundary conditions --
% Pinned-Pinned
Beam.BC.loc = [0,Beam.Prop.Lb]; Beam.BC.vert_stiff = [Inf,Inf]; Beam.BC.rot_stiff = [0,0];

% -- Mesh --
Beam.Mesh.Ele.num = 60;             % Number of elements (Should be an even number)

% -------------------------------------------------------------------------
% ----------------------------- Profile -----------------------------------
% -------------------------------------------------------------------------

% -- Type --
Calc.Profile.type = 0;               % Smooth profile
if Opt.ramp_on == 1
    Calc.Profile.type = 4;                          % Ramp profile
    Calc.Profile.ramp_loc_x_0 = Beam.Prop.Lb/3;     % Ramp start location in x-coordinate system [m]
    Calc.Profile.ramp_loc_x_end = Beam.Prop.Lb/2;   % Ramp end location in x-coordinate system [m]    
    Calc.Profile.ramp_height = 0.01;                % Ramp height in [m]
end % if Opt.ramp_on == 1

% -------------------------------------------------------------------------
% ------------------------- Calculation options ---------------------------
% -------------------------------------------------------------------------

% -- Solver step --
% It is posible to define multiple criteria to define the time step duration
Calc.Solver.max_accurate_frq = 500;             % [Hz]
%Calc.Solver.min_t_steps_per_second = 5;         % Minimum time steps per second
%Calc.Solver.min_Beam_modes_considered = 5;      % Number of beam modes to be considered for time step selection
% Alternatively, the user can define the desired time step
%Calc.Solver.t_steps_per_second = 100;

% -- Procedure to solve Interaction --
%Calc.Proc.name = 'Full_Sim_Iter';       % 1) Iterative procedure for whole time-history solution (Full simulation iteration)
%Calc.Proc.name = 'StepByStep_Iter';     % 2) Step-by-Step iteration 
%Calc.Proc.name = 'Coupled';             % 3) Coupled system solution (Default and recommended)

% -- Options for Full Simulation iteration (FI) or Step-by-Step Iteration (SSI)  --
%Calc.Proc.Iter.max_num = 40;         % Maximum number of iterations (Default: 10)
%Calc.Proc.Iter.criteria = 1;         % Iteration criteria based on deformation under wheels (Default)
%Calc.Proc.Iter.criteria = 2;         % Iteration criteria based on BM of whole beam
%Calc.Proc.Iter.tol = 1e-3;           % Tolerance for iteration stopping criteria (Default: Calc.Constant.tol)

% -- Calculation options --
%Calc.Opt.VBI = 0;                   % Disconnect VBI (Default: 1 = VBI on)
%Calc.Opt.free_vib_seconds = 0.5;    % Additional free vibration to calculate bridge response [s] (Default: 0)

% -- Display options --
Calc.Opt.verbose = 1;               % Display of comments
Calc.Opt.show_progress_every_s = 1; % Show progress every X seconds (Default: 2s)

% -------------------------------------------------------------------------
% ------------------------- Calculations ----------------------------------

A02_VBI_calculations;

% ---- Saving ----
if myIsfield(Validation,{'Save',1,'on'},1)
    % Data to save
    Data.Time.values = Calc.Solver.t_beam - Calc.Solver.t_beam(1);
    Data.MidSpan.Disp.values = Sol.Beam.Disp.value_xt(Beam.Mesh.Node.at_mid_span,:);
    % Saving
    save([Validation.Save.folder,Validation.Save.name],'Data');
    % Report
    disp('Results have been saved in:');
    disp([Validation.Save.folder,Validation.Save.name]);
end % if myIsfield(Bench,{'Save',1,'on'},1)

% ---- End of script ----    