% Use of VBI-2D to reproduce the example shown in Figure 6

% *************************************************************************
% *** Script part of VBI-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% *************************************************************************

clear; clc;

addpath 'Functions';

rng(20230109); disp('Fixed random state!!!');

% --------------------------- Definitions ---------------------------------

% -------------------------------------------------------------------------
% ------------------------------- Beam ------------------------------------
% -------------------------------------------------------------------------

% -- Properties --
Beam.Prop.type = 0;             % Custom definition of beam properties
Beam.Prop.Lb = 25;              % Beam's length
Beam.Prop.rho = 18358;          % Density per meter length
Beam.Prop.E = 3.5e10;           % E
Beam.Prop.I = 1.3901;           % I
Beam.Prop.A = 1;                % Area
Beam.Prop.damp_per = 3;         % Damping percentage

% -- Boundary conditions --
% Pinned-Pinned
Beam.BC.loc = [0,Beam.Prop.Lb]; Beam.BC.vert_stiff = [Inf,Inf]; Beam.BC.rot_stiff = [0,0];

% -- Mesh --
Beam.Mesh.Ele.num = 40;             % Number of elements (Should be an even number)

% -------------------------------------------------------------------------
% ---------------------------- Vehicles -----------------------------------
% -------------------------------------------------------------------------

veh_num = 0;

% -- Type 2 = 2-axle = Vehicle_2.m --
veh_num = veh_num + 1;
Veh(veh_num).Model.type = 'Vehicle_2';
% Vehicle properties
Veh(veh_num).Prop.mBi = 10500;
Veh(veh_num).Prop.IBi = 50000;
Veh(veh_num).Prop.kSj = [1,1]*6e6;
Veh(veh_num).Prop.cSj = [1,1]*1e4;
Veh(veh_num).Prop.mGj = [1,1]*900;
Veh(veh_num).Prop.kTk = [1,1]*1.75e6;
Veh(veh_num).Prop.cTk = [1,1]*0;
Veh(veh_num).Prop.dj = [-2.5, 2.5];
% Position
Veh(veh_num).Pos.x0 = Beam.Prop.Lb+5;
Veh(veh_num).Pos.vel = -70/3.6;
Veh(veh_num).Pos.a = -10;

% -- Type 5 = 5-axle = Vehicle_2A3.m --
veh_num = veh_num + 1;
Veh(veh_num).Model.type = 'Vehicle_2A3';
% Vehicle properties    
Veh(veh_num).Prop.mBi = [3100 20000];
Veh(veh_num).Prop.IBi = [5000 120000];
Veh(veh_num).Prop.kSj = [6000000 6000000 10000000 10000000 10000000];
Veh(veh_num).Prop.cSj = [10000 10000 20000 20000 20000];
Veh(veh_num).Prop.mGj = [750 750 1100 1100 1100];
Veh(veh_num).Prop.kTk = [1750000 1750000 3500000 3500000 3500000];
Veh(veh_num).Prop.cTk = [0 0 0 0 0];
Veh(veh_num).Prop.ai = [0, 5];
Veh(veh_num).Prop.bi = 3;
Veh(veh_num).Prop.dj = [-1, 3.5, 2, 3.2, 4.4];
% Position
Veh(veh_num).Pos.x0 = 0;
Veh(veh_num).Pos.vel = 90/3.6;

clear veh_num

% -------------------------------------------------------------------------
% ----------------------------- Profile -----------------------------------
% -------------------------------------------------------------------------

% -- Type --
%Calc.Profile.type = 0;
Calc.Profile.type = 2;               % ISO random profile 
    Calc.Profile.Info.class = 'A';   % ISO class in text format
%Calc.Profile.type = 1;                              % To load an existing proile
%    Calc.Profile.Load.file_name = 'Figure_6_Profile';   % Name of file to load

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
Calc.Plot.P5_Beam_U = 1;                   % Contourplot of Beam deformation
%Calc.Plot.P6_Beam_U_under_Veh = 1;         % Deformation under the vehicle
%Calc.Plot.P7_Veh_U_iter = 1;               % Vehicle total displacement for each iteration (Only FI procedure)
%Calc.Plot.P8_Veh_Urel_iter = 1;            % Vehicle relative displacement for each iteration (Only FI procedure)
%Calc.Plot.P9_Beam_U_under_Veh_iter = 1;    % Deformation under the vehicle for each iteration (Only FI procedure)
%Calc.Plot.P10_diff_iter = 1;               % The difference between solutions (Iteration criteria) (Only FI procedure)
%Calc.Plot.P11_Beam_U_static = 1;           % Calculates the Static Deformation of Beam (Due to Interaction force)
%Calc.Plot.P13_Interaction_Force = 1;       % Final interaction force
%Calc.Plot.P14_Interaction_Force_iter = 1;  % Interaction force for each iteration (Only FI procedure)
%Calc.Plot.P16_Beam_BM = 1;                 % Contourplot of Beam BM
%Calc.Plot.P17_Beam_BM_static = 1;          % Contourplot of Beam Static BM
%Calc.Plot.P18_Beam_Shear = 1;              % Contourplot of Beam Shear
%Calc.Plot.P19_Beam_Shear_static = 1;       % Contourplot of Beam Static Shear
Calc.Plot.P20_Beam_MidSpan_BM = 1;         % Static and Dynamic BM at mid-span
%Calc.Plot.P21_Beam_MidSpan_BM_iter = 1;    % Static and Dynamic BM at mid-span for various iterations (Only FI procedure)
%Calc.Plot.P22_SSI_num_iterations = 1;      % Number of iterations for each time step (Only for SSI procedure)
%Calc.Plot.P23_vehicles_responses = 1;      % Vehicles DOF responses (Displacement, velocity and acceleration)
%Calc.Plot.P24_vehicles_wheel_responses = 1;% Responses at the wheels of the vehicle
%Calc.Plot.P25_vehicles_under_responses = 1;% Responses under the wheel (Bridge response at the contact point)
%Calc.Plot.P26_PSD_Veh_A = 1;               % PSD of vehicle accelerations
%Calc.Plot.P27_Veh_A = 1;                   % Time histories of vehicle accelerations

% Small modification to reproduce the exact random state used in the generation of Figure 6
for i = 1:18; rand; end

% -------------------------------------------------------------------------
% ------------------------- Calculations ----------------------------------
% -------------------------------------------------------------------------

A02_Calculations;

% -------------------------------------------------------------------------
% ---------------------------- Plotting -----------------------------------
% -------------------------------------------------------------------------

A03_Results_plotting;

% -------------------------------------------------------------------------

%% Vehicle Position plot for an event for P103 (Based on P3 plot)
figure;
    hold on; box on;
for veh_num = 1:Veh(1).Event.num_veh
    h1 = plot(Veh(veh_num).Pos.wheels_x,Calc.Solver.t,'b');
end % for veh_num = 1:Veh(1).Event.num_veh
axis tight;
h4 = plot([0,0],ylim,'k--');
plot([1,1]*Beam.Prop.Lb,ylim,'k--');
xlabel('X-coordinate (m)');
ylabel('Time (s)');
title('Vehicle(s) position in time')
drawnow;

%% Tractor vertical accelerations
% veh_num = 1;
% figure;
%     plot(Calc.Solver.t,Sol.Veh(veh_num).A(1,:));
%     axis tight;
%     xlabel('Time (s)');
%     ylabel('Acc. (m/s^2)');
veh_num = 2;
figure; 
    plot(Calc.Solver.t,Sol.Veh(veh_num).A(1,:));
    axis tight;
    xlabel('Time (s)');
    ylabel('Acc. (m/s^2)');

%% Beam mid-span acceleration
figure;
    plot(Calc.Solver.t_beam,Sol.Beam.Disp_ddot.value_xt(Beam.Mesh.Node.at_mid_span,:))
    axis tight;
    xlabel('Time (s)');
    ylabel('Acc. (m/s^2)');

%% Contour Plot of Beam displacements
figure; hold on; box on;
    [~,h] = contourf(Beam.Mesh.Ele.acum,Calc.Solver.t_beam,Sol.Beam.U.value_xt'*1000,20);
    set(h,'EdgeColor','none'); colorbar;
    mycmap = colormap; colormap(flipud(mycmap));
    for veh_num = 1:Veh(1).Event.num_veh
        plot(Veh(veh_num).Pos.wheels_x(:,Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam),...
            Calc.Solver.t_beam,'k--');
    end % for veh_num = 1:Veh(1).Event.num_veh
    xlim([0,Beam.Prop.Lb]);
    xlabel('Distance (m)')
    ylabel('Time (s)')
    title('Beam displacements (mm)');
    clear h mycmap veh_num

%% Static and Dynamic BM at mid-span ----
figure; hold on; box on;
    % Static and Total
    plot(Calc.Solver.t_beam,Sol.Beam.BM.value_xt(Beam.Mesh.Node.at_mid_span,:)/1000)
    plot(Calc.Solver.t_beam,Sol.Beam.BM_static.value_xt(Beam.Mesh.Node.at_mid_span,:)/1000,'k--');
    % Dynamic component only (Total-Static)
    xlabel('Calculation time (s)');
    ylabel('Mid-span BM (kN*m)');
    title('BM at mid-span');
    %legend('Dynamic','Static'); 
    axis tight;
    
% ---- End of script ----    