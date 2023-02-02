function [Veh] = B25_WheelProfiles(Calc,Veh)

% Calculates the profile under each wheel

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

% Checking that profile is long enough
checks = false(1,3);
checks(1) = Calc.Profile.x(1) > Calc.Profile.needed_x0;
checks(2) = Calc.Profile.x(end) < Calc.Profile.needed_x_end;
checks(3) = Calc.Profile.L < Calc.Profile.needed_L;
if any(checks)
    error('The profile is not long enough for the specified event');
end % if any(cheks)

% Profile for each wheel and vehicle
for veh_num = 1:Veh(1).Event.num_veh

    % Initialize
    Veh(veh_num).Pos.wheels_h = Veh(veh_num).Pos.wheels_x*0;
    Veh(veh_num).Pos.wheels_hd = Veh(veh_num).Pos.wheels_h;
    
    % Interpotlation from full length profile 
    interp_method = 'linear';   % Matlab's default
    %interp_method = 'pchip';    % Piecewise cubic interpolation
    for wheel_num = 1:Veh(veh_num).Prop.num_wheels
        Veh(veh_num).Pos.wheels_h(wheel_num,:) = ...
            interp1(Calc.Profile.x,Calc.Profile.h,Veh(veh_num).Pos.wheels_x(wheel_num,:),interp_method);
    end % for wheel_num = 1:Veh(veh_num).Prop.num_wheels
    Veh(veh_num).Pos.wheels_hd = [zeros(Veh(veh_num).Prop.num_wheels,1),...
        diff(Veh(veh_num).Pos.wheels_h,1,2)./...
        (ones(Veh(veh_num).Prop.num_wheels,1)*(diff(Veh(veh_num).Pos.wheels_x(1,:),1,2)./Veh(veh_num).Pos.vel_values_t(2:end)))];

end % for veh_num = 1:Veh(1).Event.num_veh

% ---- End of script ----