function [Calc,Veh] = B43_ModelGeometry(Calc,Veh,Beam)

% This function outputs some useful information about the model geometry 
% based on the provided values

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

% Initialize values
Calc.Profile.needed_x0 = 0;
Calc.Profile.needed_x_end = Beam.Prop.Lb;
Calc.Solver.t_end = 0;

% Calculations for each vehicle
for veh_num = 1:Veh(1).Event.num_veh

    % Vehicle Wheelbase and number of wheels
    Veh(veh_num).Prop.wheelbase = Veh(veh_num).Prop.ax_dist(end);
    Veh(veh_num).Prop.num_wheels = length(Veh(veh_num).Prop.ax_sp);
    
    % Profile start (x0) and end (x_end) for each vehicle
    Veh(veh_num).Pos.prof_x0 = -Veh(veh_num).Prop.wheelbase + Veh(veh_num).Pos.x0_values(1)*(Veh(veh_num).Pos.x0_values(1)<0);
    Veh(veh_num).Pos.prof_x_end = Beam.Prop.Lb + Veh(veh_num).Prop.wheelbase + ...
        (Veh(veh_num).Pos.x0_values(1)-Beam.Prop.Lb)*(Veh(veh_num).Pos.x0_values(1)>0);

    % Dealing with definition options 2 and 3
    if any(Veh(veh_num).Pos.definition_option == [2,3])
        % Extending vel_values and x0_value
        if Veh(veh_num).Pos.definition_option == 3
            % 1) to start of bridge
            Veh(veh_num).Pos.x0_values = [Veh(veh_num).Pos.x0_values, Beam.Prop.Lb*(Veh(veh_num).Pos.vel_sign==-1)];
            Veh(veh_num).Pos.vel_values = Veh(veh_num).Pos.vel_values*[1,1];
            % 2) With accelerated movement
            Veh(veh_num).Pos.x0_values = [Veh(veh_num).Pos.x0_values, Beam.Prop.Lb*(Veh(veh_num).Pos.vel_sign==1) + Veh(veh_num).Prop.wheelbase*Veh(veh_num).Pos.vel_sign];
            % From: v^2 = v0^2 + 2*a*x
            if Veh(veh_num).Pos.vel_sign == 1
                new_vel_value = sqrt(Veh(veh_num).Pos.vel_values(end)^2+2*Veh(veh_num).Pos.a*Veh(veh_num).Pos.x0_values(end));
            elseif Veh(veh_num).Pos.vel_sign == -1
                new_vel_value = -sqrt(Veh(veh_num).Pos.vel_values(end)^2+2*Veh(veh_num).Pos.a*diff(Veh(veh_num).Pos.x0_values([end-1,end])));
            end % Veh(veh_num).Pos.vel_sign == 1
            if isreal(new_vel_value)
                Veh(veh_num).Pos.vel_values = [Veh(veh_num).Pos.vel_values, new_vel_value];
            else
                disp('The decceleration of the vehicle is too big. The vehicle would stop on the bridge');
                disp('Change the sign or the value of the vehicle acceleration');
                error('The decceleration of the vehicle is too big. The vehicle would stop on the bridge');
            end % if isreal(new_vel_value)
        end % if Veh(veh_num).Pos.definition_option == 3
        % 3) with constant velocity
        Veh(veh_num).Pos.vel_values = [Veh(veh_num).Pos.vel_values, Veh(veh_num).Pos.vel_values(end)];
        if Veh(veh_num).Pos.vel_sign>0
            Veh(veh_num).Pos.x0_values = [Veh(veh_num).Pos.x0_values, max([Veh(veh_num).Pos.prof_x_end,Veh(veh_num).Pos.x0_values(end)]) + 0.01];
        elseif Veh(veh_num).Pos.vel_sign<0
            Veh(veh_num).Pos.x0_values = [Veh(veh_num).Pos.x0_values, min([Veh(veh_num).Pos.prof_x0,Veh(veh_num).Pos.x0_values(end)]) - 0.01];
        end % if Veh(veh_num).Pos.vel_sign>0
        % Removing possible duplicate values
        [Veh(veh_num).Pos.x0_values,IA] = unique(Veh(veh_num).Pos.x0_values,'stable');
        Veh(veh_num).Pos.vel_values = Veh(veh_num).Pos.vel_values(IA);
        % Constant acceleration for each interval
        Veh(veh_num).Pos.a_values = diff(Veh(veh_num).Pos.vel_values.^2)./(2*diff(Veh(veh_num).Pos.x0_values));
        % Times (based on equation x = (v_0 + v)/2*t )
        Veh(veh_num).Pos.t_values = [0, ...
            cumsum(diff(Veh(veh_num).Pos.x0_values)./((Veh(veh_num).Pos.vel_values(1:end-1)+Veh(veh_num).Pos.vel_values(2:end))/2))];
    else
        Veh(veh_num).Pos.a_values = 0;
    end % if any

    % Minimum time to simulate
    if Veh(veh_num).Pos.var_vel_flag == 0
        Veh(veh_num).Pos.min_t_end = (Veh(veh_num).Pos.prof_x_end-Veh(veh_num).Pos.prof_x0-Veh(veh_num).Prop.wheelbase)/abs(Veh(veh_num).Pos.vel_values);
    else % if Veh(veh_num).Pos.var_vel_flag == 1
        Veh(veh_num).Pos.min_t_end = Veh(veh_num).Pos.t_values(end);
    end % if Veh(veh_num).Pos.var_vel_flag == 1
    
    % Minimum and maximum values for x0 and x_end
    Calc.Profile.needed_x0 = min(Calc.Profile.needed_x0,Veh(veh_num).Pos.prof_x0);
    Calc.Profile.needed_x_end = max(Calc.Profile.needed_x_end,Veh(veh_num).Pos.prof_x_end);
    
    % Simulation time
    Calc.Solver.t_end = max(Calc.Solver.t_end,Veh(veh_num).Pos.min_t_end);
    
end % for veh_num = 1: Veh(1).Event.num_veh

% Maximum velocity in event
Veh(1).Event.max_vel = max(abs(Veh(1).Pos.vel_values));
for veh_num = 2:Veh(1).Event.num_veh
    Veh(1).Event.max_vel = max(Veh(1).Event.max_vel,max(abs(Veh(veh_num).Pos.vel_values)));
end % for veh_num = 2:Veh(1).Event.num_veh

% Addition of free vibration time
Calc.Solver.t_end = Calc.Solver.t_end + Calc.Opt.free_vib_seconds;

% Total length of profile
Calc.Profile.needed_L = Calc.Profile.needed_x_end - Calc.Profile.needed_x0;

% ---- End of function ----