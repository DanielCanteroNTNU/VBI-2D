function [Veh] = B45_VehicleModel(Calc,Veh)

% Runs corresponding vehicle model script to obtain vehicle's system matrices
% Also calculates the static load of each vehicle

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

% ---- Vehicle system matrices ----

% Current directory
old_dir = cd;

% Move workspace to new directory
cd(Veh(1).Model.function_path)
copy_function_path = Veh(1).Model.function_path;

% Vehicle loop
for veh_num = 1:Veh(1).Event.num_veh
    
    % Running appropriate function
    if veh_num == 1 
        CopyVeh = feval(Veh(veh_num).Model.type,Veh(veh_num));
    else
        CopyVeh(veh_num) = feval(Veh(veh_num).Model.type,Veh(veh_num));
    end % if veh_num == 1 

end % for veh_num = 1:Veh(1).Event.num_veh

% Changing back to original path
cd(old_dir);

% Generating output
Veh = CopyVeh;
Veh(1).Model.function_path = copy_function_path;

% ---- Vehicle static load ----

% Vehicle loop
for veh_num = 1:Veh(1).Event.num_veh
    
    % Calculation of static deformation
    disp = (Veh(veh_num).Static.F_vector_no_grav*Calc.Constant.grav)/Veh(veh_num).SysM.K;

    % Contact Forces
    Veh(veh_num).Static.load = Veh(veh_num).Prop.kTk'.*(Veh(veh_num).SysM.N2w*disp');
    
    % Checks
    Veh(veh_num).Static.check = ...
        (sum(Veh(veh_num).Static.load)-((sum(Veh(veh_num).Prop.mBi)+sum(Veh(veh_num).Prop.mGj))*Calc.Constant.grav))<Calc.Constant.tol;
    if Veh(veh_num).Static.check==false
        disp(['Error: Vehicle ',num2str(veh_num),' -> in calculation of static contact force']);
    end % if Veh(veh_num).Static.check==false
    
    % Gross vehicle weight (GVW) in [N]
    Veh(veh_num).Static.GVW_N = sum(Veh(veh_num).Static.load);

end % for veh_num = 1:Veh(1).Event.num_veh

% ---- End of function ----