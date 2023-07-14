% ------------------------- Calculations ----------------------------------

% *************************************************************************
% *** Script part of VBI-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% *************************************************************************

tic;

% -- Options Processing --
[Calc,Veh,Beam] = B07_OptionsProcessing(Calc,Veh,Beam);

% ---- Beam Model ----
% Beam properties
[Beam] = B51_BeamProperties(Beam);
% Elements and coordinates
[Beam] = B01_ElementsAndCoordinates(Beam);
% Boundary Conditions 
[Beam] = B02_BeamBoundaryConditions(Beam);
% Adding Damaged elements
[Beam] = B22_DamageBeamElements(Beam);
% Beam System Matrices (Mass and stiffness)
[Beam] = B03_Beam_Matrices(Beam);
% Beam frequencies 
[Beam] = B09_BeamFrq(Calc,Beam);
% Beam Damping Matrix
[Beam] = B24_Beam_Damping(Beam);

% ---- Vehicle Model ----
% Vehicle System matrices
[Veh] = B45_VehicleModel(Calc,Veh);
% Vehicle frequencies
[Calc,Veh] = B08_VehFreq(Calc,Veh);
% Vehicle position and Solver time array --
[Calc,Veh] = B43_ModelGeometry(Calc,Veh,Beam);

% ---- Solver time ----
% Solver time step
[Calc] = B42_TimeStep(Calc,Veh,Beam);
% Time and space discretization
[Calc,Veh] = B11_TimeSpace_discretization(Calc,Veh,Beam);

% -- Profile --
% Profile Loading / Generation / Save / Others
[Calc] = B40_Profile(Calc);
% Assigning profile to each wheel
[Veh] = B25_WheelProfiles(Calc,Veh);

% -- Other --
% Printing comments
B41_Verbose_comments(Calc,Veh,Beam);
% Some plotting related operations
B46_SomePlottingVariables;

% ---- Solving Interaction problem ----

if Calc.Proc.code == 1
    
    % -- Full model iteration procedure --
    disp('Full Iteration (FI) procedure started ...');
    % Vehicle Dynamic Simulation on Approach
    [Sol] = B28_Veh_DynamicCalc_Approach(Calc,Veh);
    while Calc.Proc.Iter.continue == 1
        disp(['Iteration ',num2str(Calc.Proc.Iter.num+1)]);
        % Profile update
        [Calc,Sol] = B37_ProfileUpdate(Calc,Veh,Beam,Sol);
        % Vehicle Dynamic Simulation 
        [Sol] = B12_Veh_DynamicCalc_OnBeam(Calc,Veh,Sol);
        % Interaction force
        [Sol] = B29_ForceOnBeam(Calc,Veh,Sol);
        % Beam Dynamic Simulation
        [Sol] = B13_Beam_DynamicCalc(Calc,Veh,Beam,Sol);
        % Bending Moment
        [Sol] = B31_Beam_BM_iter(Calc,Beam,Sol);
        % New Beam displacements under vehicle
        [Sol] = B17_Calc_U_at(Calc,Veh,Beam,Sol);
        % Iteration criteria
        [Calc,Sol] = B36_IterationCriteria(Calc,Sol);
        % Some plotting related operations
        B46_SomePlottingVariables;
    end % while Calc.Proc.Iter.continue == 1
    
elseif Calc.Proc.code == 2
    
    % -- Step-by-step iteration procedure --
    disp('Step-by-Step Iteration (SSI) procedure started ...');
    % Vehicle Dynamic Simulation on Approach
    [Sol] = B28_Veh_DynamicCalc_Approach(Calc,Veh);
    % SSI procedure 
    [Calc,Sol] = B48_SSI_solver(Calc,Veh,Beam,Sol);
    
elseif Calc.Proc.code == 3
    
    % -- Coupled procedure --
    disp('Coupled procedure started ...');
    % Coupled system solution
    [Sol] = B49_Coup_solver(Calc,Veh,Beam);
    % Contact force calculation
    [Calc,Sol] = B50_ContactForce(Calc,Veh,Beam,Sol);
    
end % if Calc.Proc.code == 1
fprintf('\b'); disp('  DONE');

% ---- Load effects ----
% Bending Moment
[Sol] = B31_Beam_BM(Calc,Beam,Sol);
% Shear Force
[Sol] = B33_Beam_Shear(Calc,Beam,Sol);
% Static Beam deformation
[Sol] = B15_Beam_Static_U(Calc,Veh,Beam,Sol);
% Static Bending Moment
[Sol] = B31_Beam_BM(Calc,Beam,Sol,1);
% Static Shear Force
[Sol] = B33_Beam_Shear(Calc,Beam,Sol,1);
% Displacements and Rotations
[Sol] = B52_Beam_Disp_and_Rot(Sol);
% Load effects maximum and minimum
[Sol] = B47_LoadEffectsMax(Beam,Sol);

% % Finished simulation
% disp('OK: End of script reached:');
% disp(datetime);
% disp(['Elapsed time is ',num2str(round(toc,3)),'s']);

% ---- End of script ----
