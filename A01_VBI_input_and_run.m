% Main script to run VBI-2D model

% *************************************************************************
% *** Script part of VBI-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% *************************************************************************

clear; clc; close all
addpath 'Functions';

% --------------------------- Definitions ---------------------------------

% -------------------------------------------------------------------------
% ---------------------------- Vehicles -----------------------------------
% -------------------------------------------------------------------------

veh_num = 0;

% -- Sprung mass -- (1-DOF vehicle)
veh_num = veh_num + 1;
Veh(veh_num).Model.type = 'Sprung_mass';% Text with name of model to use
Veh(veh_num).Prop.mGj = 1*1e3;          % Suspension mass [kg]
Veh(veh_num).Prop.kTk = 1*1e6;          % Tyre stiffness [N/m]
Veh(veh_num).Prop.cTk = 1*1e4;          % Tyre viscous damping [N*s/m]

% % -- Quater-car -- (2-DOF vehicle)
% veh_num = veh_num + 1;
% Veh(veh_num).Model.type = 'Vehicle_1';  % Text with name of model to use1
% Veh(veh_num).Prop.mBi = 10000;          % Body masses [kg]
% Veh(veh_num).Prop.kSj = 1*1e6;          % Suspension stiffness [N/m]
% Veh(veh_num).Prop.cSj = 1*1e4;          % Suspension viscous damping [N*s/m]
% Veh(veh_num).Prop.mGj = 1*1e3;          % Axle/group mass [kg]
% Veh(veh_num).Prop.kTk = 1*1e6;          % Tyre stiffness [N/m]
% Veh(veh_num).Prop.cTk = 1*1e4;          % Tyre viscous damping [N*s/m]

% % -- Vehicle 2 -- (Vehicle with 2 axles)
% veh_num = veh_num + 1;
% Veh(veh_num).Model.type = 'Vehicle_2';  % Text with name of model to use
% Veh(veh_num).Prop.mBi = 10000;          % Body masses [kg]
% Veh(veh_num).Prop.IBi = 20000;          % Body moments of inertia [kg*m^2]
% Veh(veh_num).Prop.kSj = [1,1]*1e6;      % Suspension stiffness [N/m]
% Veh(veh_num).Prop.cSj = [1,1]*1e4;      % Suspension viscous damping [N*s/m]
% Veh(veh_num).Prop.mGj = [1,1]*1e3;      % Axle/group mass [kg]
% Veh(veh_num).Prop.kTk = [1,1]*1e6;      % Tyre stiffness [N/m]
% Veh(veh_num).Prop.cTk = [1,1]*1e4;      % Tyre viscous damping [N*s/m]
% Veh(veh_num).Prop.dj = [-2, 2];         % Coordinate of axle (group) to centre of gravity of its corresponding body [m]

% % -- Vehicle 2A3 -- (2-axle body + Articulation + 3-axle body)
% veh_num = veh_num + 1;
% Veh(veh_num).Model.type = 'Vehicle_2A3';                % Text with name of model to use
% Veh(veh_num).Prop.mBi = [4500, 31450];                  % Body masses [kg]
% Veh(veh_num).Prop.IBi = [4604, 16302];                  % Body moments of inertia [kg*m^2]
% Veh(veh_num).Prop.kSj = [400,1000,750,750,750]*1e3;     % Suspension stiffness [N/m]
% Veh(veh_num).Prop.cSj = [1,1,1,1,1]*1e3;                % Suspension viscous damping [N*s/m]
% Veh(veh_num).Prop.mGj = [700,1100,750,750,750];         % Axle/group mass [kg]
% Veh(veh_num).Prop.kTk = [1750,3500,3500,3500,3500]*1e3; % Tyre stiffness [N/m]
% Veh(veh_num).Prop.cTk = [1,1,1,1,1]*1e4;                % Tyre viscous damping [N*s/m]
% Veh(veh_num).Prop.ai = [0, 6.15];                       % Front distances from bodies centre of gravity
% Veh(veh_num).Prop.bi = 2.15;                            % Back distances from bodies centre of gravity
% Veh(veh_num).Prop.dj = [-0.5, 2.5, 1.8, 2.9, 4];        % Coordinate of axle (group) to centre of gravity of its corresponding body [m]

% % -- Vehicle 2A3_G_1_1_3 -- (2-axle body with separate axles + Articulation + 3-axle body with all axles grouped into one)
% veh_num = veh_num + 1;
% Veh(veh_num).Model.type = 'Vehicle_2A3_G_1_1_3';        % Text with name of model to use
% Veh(veh_num).Prop.mBi = [4500, 31450];                  % Body masses [kg]
% Veh(veh_num).Prop.IBi = [4604, 16302];                  % Body moments of inertia [kg*m^2]
% Veh(veh_num).Prop.kSj = [400,1000,750*3]*1e3;           % Suspension stiffness [N/m]
% Veh(veh_num).Prop.cSj = [1,1,1*3]*1e3;                  % Suspension viscous damping [N*s/m]
% Veh(veh_num).Prop.mGj = [700,1100,750*3];               % Axle/group mass [kg]
% Veh(veh_num).Prop.IGj = [0,0,200];                      % Axle/group moments of inertia [kg*m^2]
% Veh(veh_num).Prop.kTk = [1750,3500,3500,3500,3500]*1e3; % Tyre stiffness [N/m]
% Veh(veh_num).Prop.cTk = [1,1,1,1,1]*1e4;                % Tyre viscous damping [N*s/m]
% Veh(veh_num).Prop.ai = [0, 6.15];                       % Front distances from bodies centre of gravity
% Veh(veh_num).Prop.bi = 2.15;                            % Back distances from bodies centre of gravity
% Veh(veh_num).Prop.dj = [-0.5, 2.5, 1.9];                % Coordinate of axle/group to centre of gravity of its corresponding body [m]
% Veh(veh_num).Prop.ek = [0, 0, -1.1, 0, 1.1];            % Coordinate of wheel to centre of gravity of its corresponding axle/group [m]

% -- Addition to define constant/variable vehicle speed --
% Option 1) Constant speed: Defined by [.x0, .vel]
Veh(veh_num).Pos.x0 = -10;                        % Initial position of 1st axle of vehicle (from left bridge support)
Veh(veh_num).Pos.vel = 20;                        % Velocity of vehicle [m/s] (negative value = vehicle moving from right to left)
% % Option 2) Variable speed: Defined by [.vel_values, .x0_values]
% Veh(veh_num).Pos.x0_values = [0,25+10];             % Array specifiying the 1st wheel positions for corresponding vel_values
% Veh(veh_num).Pos.vel_values = [20,30];              % Array of velocity values at x0 positions [m/s] (negative value = vehicle moving from right to left)
% % Option 3) Variable speed: Defined by [.x0, .vel, .a]
% Veh(veh_num).Pos.x0 = -10;                         % Initial position of 1st axle of vehicle (from left bridge support)
% Veh(veh_num).Pos.vel = 20;                         % Velocity of vehicle [m/s] (negative value = vehicle moving from right to left)
% Veh(veh_num).Pos.a = 10;                           % Uniform acceleration of the vehicle [m/s^2] (For deceleration use opposite sign of .vel)

clear veh_num

% -------------------------------------------------------------------------
% ----------------------------- Profile -----------------------------------
% -------------------------------------------------------------------------

% -- Type --
Calc.Profile.type = 0;               % Smooth profile
%Calc.Profile.type = 1 ;                             % To load an existing proile 
%    %Calc.Profile.Load.path = 'Profiles\';          % Optional: Custom loading path. Default subfolder: 'Profiles\
%    Calc.Profile.Load.file_name = 'Test_Profile';   % Name of file to load
% Calc.Profile.type = 2;               % ISO random profile 
%     Calc.Profile.Info.class = 'A';   % ISO class in text format
%     %Calc.Profile.Opt.class_var = 0;  % Consider variation of roughness within the profile class (0 = No, 1 = Yes) (Default: 0)
% Calc.Profile.type = 3;               % Step profile
%    Calc.Profile.step_loc_x = 5;      % Step location in x-coordinate system [m]
%    Calc.Profile.step_height = 0.05;  % Step height in [m]
% Calc.Profile.type = 4;               % Ramp profile
%    Calc.Profile.ramp_loc_x_0 = 5;    % Ramp start location in x-coordinate system [m]
%    Calc.Profile.ramp_loc_x_end = 7;  % Ramp end location in x-coordinate system [m]    
%    Calc.Profile.ramp_height = 0.01;  % Ramp height in [m]
% Calc.Profile.type = 5;               % Sinewave profile
%    Calc.Profile.sine_wavelength = 20;% Wavelength of the sinusoidal [m]
%    Calc.Profile.sin_Amp = 0.01;      % Amplitude of the sinusoidal [m]
%    Calc.Profile.sin_phase = 0;       % Phase of sinusoidal at x=0 [rad]

% -- Saving Profile --
%Calc.Profile.Save.on = 1;                      % To save the generated profile (Default: off = 0)
%Calc.Profile.Save.file_name = 'Test_Profile';  % Name of file to save
%Calc.Profile.Load.path = 'Profiles\';          % Optional: Custom  savingpath. Default subfolder: 'Profiles\

% -- Other Options --
%Calc.Profile.Opt.MovAvg.on = 1;            % Moving Average filter for profile (Default: off)
%Calc.Profile.Opt.MovAvg.window_L = 0.24;   % Window length [m] (Default: 0.24 m)
%Calc.Profile.L = 1000;                     % Profile length [m] (longer than necessary) (Default: 1000)

% -------------------------------------------------------------------------
% ------------------------------- Beam ------------------------------------
% -------------------------------------------------------------------------

% -- Properties --
Beam.Prop.Lb = 25;              % Beam's length [m]
Beam.Prop.rho = 18358;          % Mass per unit length [kg/m]
Beam.Prop.E = 3.5e10;           % Elastic modulus [N/m^2]
Beam.Prop.I = 1.3901;           % Second moment of area [m^4]
Beam.Prop.damp_per = 3;         % Damping percentage [%]

% Alternative definition: Based on tables for "typical" bridges
% Beam.Prop.type = 'T';         % Type of cross-section ('T', 'Y', or 'SY')
% Beam.Prop.Lb = 15;            % Beam's length [m]
% Beam.Prop.damp_per = 3;       % Damping percentage [%]

% -- Boundary conditions --
Beam.BC.loc = [0,Beam.Prop.Lb]; % Location of supports
Beam.BC.vert_stiff = [Inf,Inf]; % Vertical stiffness (0 = free, Inf = fixed, other = vertical stiffness)
Beam.BC.rot_stiff = [0,0];      % Rotational stiffness (0 = free, Inf = fixed, other = rotational stiffness)
% % Examples for other configurations
% % Pinned-Pinned
% Beam.BC.loc = [0,Beam.Prop.Lb]; Beam.BC.vert_stiff = [Inf,Inf]; Beam.BC.rot_stiff = [0,0];
% % Fixed-Fixed
% Beam.BC.loc = [0,Beam.Prop.Lb]; Beam.BC.vert_stiff = [Inf,Inf]; Beam.BC.rot_stiff = [Inf,Inf];
% % With some rotational stiffness
% Beam.BC.loc = [0,Beam.Prop.Lb]; Beam.BC.vert_stiff = [Inf,Inf]; Beam.BC.rot_stiff = [1,1]*1e10;
% % Two span continuous bridge (pin supports)
% Beam.BC.loc = [0,Beam.Prop.Lb/2,Beam.Prop.Lb]; Beam.BC.vert_stiff = [Inf,Inf,Inf]; Beam.BC.rot_stiff = [0,0,0];

% -- Mesh --
Beam.Mesh.Ele.num = 40;         % Number of elements

% -- Damage --
% Beam.Damage.type = 0;          % No damage (Default)
% Beam.Damage.type = 1;          % 1 element damage
%   Beam.Damage.loc_per = 50;   % Location of the damage in percentage of beam length
%   Beam.Damage.E.per = 20;     % Magnitude of Stiffnes reduction in percentage
%   %Beam.Damage.I.per = 20;     % Magnitude of Inertia reduction in percentage
%   %Beam.Damage.rho.per = 20;   % Magnitude of Mass per unit length reduction in percentage
% Beam.Damage.type = 2;          % Global damage
%   Beam.Damage.E.per = 20;     % Magnitude of Stiffnes reduction in percentage
%   %Beam.Damage.I.per = 20;     % Magnitude of Inertia reduction in percentage
%   %Beam.Damage.rho.per = 20;   % Magnitude of Mass per unit length reduction in percentage
%Beam.Damage.type = 3;          % Change in BC
%   Beam.Damage.BC.loc = Beam.BC.loc;      % Location of BC changes (Default: Beam.BC.loc)
%   Beam.Damage.BC.rot_stiff = [1,1]*1e10; % Additional stiffness due to damage (Default: 0 values)
% Beam.Damage.type = 4;          % Function in space of the factor to apply to property E
%   Beam.Damage.Plot.on = 1;     % Switch on/off graphical representation of the factor applied to E (Default: 0 = off)
%   % Example of parabolic variation of E
%   %Beam.Damage.E.Inputs.values = [0.1, Beam.Prop.Lb];   % Inputs to the user defined function
%   %Beam.Damage.E.fun = @(x,Inputs) -(1-Inputs.values(1))*4/Inputs.values(2)^2*x.^2 + (1-Inputs.values(1))*4/Inputs.values(2)*x + Inputs.values(1);
%   % Example of reduction in two elements near mid-span
%   %Beam.Damage.E.Inputs.values = ones(1,Beam.Mesh.Ele.num);
%   %Beam.Damage.E.Inputs.values(round(Beam.Mesh.Ele.num*50/100)+[0,1]) = 1 - 20/100;
%   %Beam.Damage.E.fun = @(x,Inputs) Inputs.values;

% -------------------------------------------------------------------------
% ------------------------- Calculation options ---------------------------
% -------------------------------------------------------------------------

% -- Solver step --
% It is posible to define multiple criteria to define the time step duration
Calc.Solver.max_accurate_frq = 500;              % [Hz]
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
%Calc.Proc.Iter.criteria = 1;         % Iteration criteria based on deformation under wheels
%Calc.Proc.Iter.criteria = 2;         % Iteration criteria based on BM of whole beam (Default)
%Calc.Proc.Iter.tol = 1e-3;           % Tolerance for iteration stopping criteria (Default: Calc.Constant.tol)

% -- Calculation options --
%Calc.Opt.VBI = 0;                   % Disconnect VBI (Default: 1 = VBI on)
%Calc.Opt.free_vib_seconds = 0.5;    % Additional free vibration to calculate bridge response [s] (Default: 0)

% -- Display options --
Calc.Opt.verbose = 1;               % Display of comments
Calc.Opt.show_progress_every_s = 1; % Show progress every X seconds (Default: 2s)

% -------------------------------------------------------------------------
% ------------------------- Plotting Options ------------------------------
% -------------------------------------------------------------------------

%Calc.Plot.P1_Beam_frq = 1;                 % Distribution of beam frequencies
%Calc.Plot.P2_Beam_modes = 10;              % First n modes of vibration (of Beam)
%Calc.Plot.P3_VehPos = 1;                   % Vehicle position (velocity and acceleration)
%Calc.Plot.P4_Profile = 1;                  % Profiles and 1st derivative
%Calc.Plot.Profile_original = 1;            % Full generated/loaded profile (Inside function B19)
%Calc.Plot.P5_Beam_U = 1;                   % Contourplot of Beam deformation
%Calc.Plot.P6_Beam_U_under_Veh = 1;         % Deformation under the vehicle
%Calc.Plot.P7_Veh_U_iter = 1;               % Vehicle total displacement for each iteration (Only FI procedure)
%Calc.Plot.P8_Veh_Urel_iter = 1;            % Vehicle relative displacement for each iteration (Only FI procedure)
%Calc.Plot.P9_Beam_U_under_Veh_iter = 1;    % Deformation under the vehicle for each iteration (Only FI procedure)
%Calc.Plot.P10_diff_iter = 1;               % The difference between solutions (Iteration criteria) (Only FI procedure)
%Calc.Plot.P11_Beam_U_static = 1;           % Calculates the Static Deformation of Beam (Due to Interaction force)
%Calc.Plot.P13_Interaction_Force = 1;       % Final interaction force
%Calc.Plot.P14_Interaction_Force_iter = 1;  % Interaction force for each iteration (Only FI procedure)
Calc.Plot.P16_Beam_BM = 1;                 % Contourplot of Beam BM
%Calc.Plot.P17_Beam_BM_static = 1;          % Contourplot of Beam Static BM
%Calc.Plot.P18_Beam_Shear = 1;              % Contourplot of Beam Shear
Calc.Plot.P19_Beam_Shear_static = 1;       % Contourplot of Beam Static Shear
Calc.Plot.P20_Beam_MidSpan_BM = 1;         % Static and Dynamic BM at mid-span
%Calc.Plot.P21_Beam_MidSpan_BM_iter = 1;    % Static and Dynamic BM at mid-span for various iterations (Only FI procedure)
%Calc.Plot.P22_SSI_num_iterations = 1;      % Number of iterations for each time step (Only for SSI procedure)
%Calc.Plot.P23_vehicles_responses = 1;      % Vehicles DOF responses (Displacement, velocity and acceleration)
Calc.Plot.P24_vehicles_wheel_responses = 1;% Responses at the wheels of the vehicle
%Calc.Plot.P25_vehicles_under_responses = 1;% Responses under the wheel (Bridge response at the contact point)
%Calc.Plot.P26_PSD_Veh_A = 1;               % PSD of vehicle accelerations
Calc.Plot.P27_Veh_A = 1;                   % Time histories of vehicle accelerations

% -------------------------------------------------------------------------
% ------------------------- Calculations ----------------------------------
% -------------------------------------------------------------------------

A02_Calculations;

% -------------------------------------------------------------------------
% ---------------------------- Plotting -----------------------------------
% -------------------------------------------------------------------------

A03_Results_plotting;

% ---- End of script ----    
