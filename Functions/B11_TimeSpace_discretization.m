function [Calc,Veh] = B11_TimeSpace_discretization(Calc,Veh,Beam)

% Generates the uniformly spaced time discretization, correponding space 
% discretization, vehicle(s) locations and beam calculation times

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

% Time solver array
Calc.Solver.t = linspace(0,Calc.Solver.t_end,ceil(Calc.Solver.t_steps_per_second*Calc.Solver.t_end)+1);

% Solver sampling time 
Calc.Solver.dt = Calc.Solver.t(2)-Calc.Solver.t(1);

% Number of solver steps
Calc.Solver.num_t = length(Calc.Solver.t);

% Wheels' poition in time
for veh_num = 1:Veh(1).Event.num_veh
    % For constant velocity case
    if Veh(veh_num).Pos.var_vel_flag == 0
        x = (Calc.Solver.t*Veh(veh_num).Pos.vel_values+Veh(veh_num).Pos.x0_values);
        Veh(veh_num).Pos.vel_values_t = Veh(veh_num).Pos.vel_values*ones(size(Calc.Solver.t));
    % For variable velocity case
    elseif Veh(veh_num).Pos.var_vel_flag == 1
        % Values for each of the user-defined intervals
        t_values_t = Calc.Solver.t - interp1(Veh(veh_num).Pos.t_values,Veh(veh_num).Pos.t_values,Calc.Solver.t,'previous',Veh(veh_num).Pos.t_values(end));
        x0_values_t = interp1(Veh(veh_num).Pos.t_values,Veh(veh_num).Pos.x0_values,Calc.Solver.t,'previous',Veh(veh_num).Pos.x0_values(end));
        vel_values_t = interp1(Veh(veh_num).Pos.t_values,Veh(veh_num).Pos.vel_values,Calc.Solver.t,'previous',Veh(veh_num).Pos.vel_values(end));
        a_values_t = interp1(Veh(veh_num).Pos.t_values,[Veh(veh_num).Pos.a_values,Veh(veh_num).Pos.a_values(end)],Calc.Solver.t,'previous',Veh(veh_num).Pos.a_values(end));
        % Position
        x = x0_values_t + vel_values_t.*t_values_t + a_values_t/2.*t_values_t.^2;
        % Velocity
        Veh(veh_num).Pos.vel_values_t = vel_values_t + a_values_t.*t_values_t;
    end % if Veh(veh_num).Pos.var_vel_flag == 0
    Veh(veh_num).Pos.wheels_x = ones(Veh(veh_num).Prop.num_wheels,1)*x;
    Veh(veh_num).Pos.wheels_on_beam = ones(size(Veh(veh_num).Pos.wheels_x));
    for axle_num = 2:Veh(veh_num).Prop.num_wheels
        Veh(veh_num).Pos.wheels_x(axle_num,:) = Veh(veh_num).Pos.wheels_x(axle_num,:) ...
            - Veh(veh_num).Prop.ax_dist(axle_num)*Veh(veh_num).Pos.vel_sign;
    end % for axle_num = 2:Veh(veh_num).Prop.num_wheels
    Veh(veh_num).Pos.wheels_on_beam(Veh(veh_num).Pos.wheels_x<0) = 0;
    Veh(veh_num).Pos.wheels_on_beam(Veh(veh_num).Pos.wheels_x>Beam.Prop.Lb) = 0;
end % for veh_num = 1:Veh(1).Event.num_veh

% Initialize variables
Calc.Solver.t0_ind_beam = Calc.Solver.num_t;
Calc.Solver.t_end_ind_beam = 0;

% Beam calculation times (for each vehicle)
for veh_num = 1:Veh(1).Event.num_veh
    
    % Start of Beam calculation
    if Veh(veh_num).Pos.vel_values(1)>0
        [~,Veh(veh_num).Pos.t0_ind_beam] = min(abs(max(Veh(veh_num).Pos.wheels_x,[],1)));
    elseif Veh(veh_num).Pos.vel_values(1)<0
        [~,Veh(veh_num).Pos.t0_ind_beam] = min(abs(min(Veh(veh_num).Pos.wheels_x,[],1)-Beam.Prop.Lb));
    end % if Veh(veh_num).Pos.vel_values(1)>0

    % Making sure that Veh(veh_num).Pos.t0_ind_beam is before the beam
    if Veh(veh_num).Pos.t0_ind_beam > 1
        Veh(veh_num).Pos.t0_ind_beam = Veh(veh_num).Pos.t0_ind_beam - 1;
    end % if Veh(veh_num).Pos.t0_ind_beam > 1

    % End of Beam calculation
    if Veh(veh_num).Pos.vel_values(1)>0
        [~,Veh(veh_num).Pos.t_end_ind_beam] = min(abs(min(Veh(veh_num).Pos.wheels_x,[],1)-Beam.Prop.Lb));
    elseif Veh(veh_num).Pos.vel_values(1)<0
        [~,Veh(veh_num).Pos.t_end_ind_beam] = min(abs(max(Veh(veh_num).Pos.wheels_x,[],1)));
    end % if Veh(veh_num).Pos.vel_values(1)>0
    
    % Making sure that Veh(veh_num).Pos.t_end_ind_beam is after the beam
    if Veh(veh_num).Pos.t_end_ind_beam < Calc.Solver.num_t
        Veh(veh_num).Pos.t_end_ind_beam = Veh(veh_num).Pos.t_end_ind_beam + 1;
    end % if Veh(veh_num).Pos.t0_ind_beam < Calc.Solver.num_t

    % Values for Solver structure
    Calc.Solver.t0_ind_beam = min(Calc.Solver.t0_ind_beam,Veh(veh_num).Pos.t0_ind_beam);
    %Calc.Solver.t_end_ind_beam = max(Calc.Solver.t_end_ind_beam,Veh(veh_num).Pos.t_end_ind_beam);
    
    % Number of time steps
    Veh(veh_num).Pos.num_t_beam = Veh(veh_num).Pos.t_end_ind_beam - Veh(veh_num).Pos.t0_ind_beam + 1;
    
end % for veh_num = 1:Veh(1).Event.num_veh

% End of beam solver time equal to end of time discretization (which includes possible free vibration)
Calc.Solver.t_end_ind_beam = Calc.Solver.num_t;

Calc.Solver.num_t_beam = Calc.Solver.t_end_ind_beam - Calc.Solver.t0_ind_beam + 1;
Calc.Solver.t_beam = Calc.Solver.t(Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam);

% The element and relative position of each wheel for each vehicle in time
for veh_num = 1:Veh(1).Event.num_veh
    % Determination of element number
    for wheel = 1:Veh(veh_num).Prop.num_wheels
        aux1 = Veh(veh_num).Pos.wheels_x(wheel,:);
        aux1(aux1>Beam.Prop.Lb) = -1;
        aux1(aux1<0) = -1;
        elexj = aux1*0; xj = elexj;
        for j = Beam.Mesh.Ele.num:-1:1
            aux4 = aux1>=Beam.Mesh.Ele.acum(j);
            elexj(aux4) = j;
            xj(aux4) = aux1(aux4) - Beam.Mesh.Ele.acum(elexj(aux4));
            aux1(aux4) = -1;
        end % for j = length(Beam.Mesh.Ele.acum)-1:-1:1
        Veh(veh_num).Pos.elexj(wheel,:) = elexj;
        Veh(veh_num).Pos.xj(wheel,:) = xj;
    end % for wheel = 1:Veh(veh_num).Prop.num_wheels
end % for veh_num = 1:Veh(1).Event.num_veh

% ---- Plotting Results ----
if isfield(Calc.Plot,'P3_VehPos')
    figure; hold on; box on;
    for veh_num = 1:Veh(1).Event.num_veh
        h1 = plot(Veh(veh_num).Pos.wheels_x,Calc.Solver.t,'b');
    end % for veh_num = 1:Veh(1).Event.num_veh
    h2 = plot(xlim,[1,1]*Calc.Solver.t(Calc.Solver.t0_ind_beam),'r');
    plot(xlim,[1,1]*Calc.Solver.t(Calc.Solver.t_end_ind_beam),'r');
    for veh_num = 1:Veh(1).Event.num_veh
        h3 = plot(xlim,[1,1]*Calc.Solver.t(Veh(veh_num).Pos.t0_ind_beam),'k-.');
        plot(xlim,[1,1]*Calc.Solver.t(Veh(veh_num).Pos.t_end_ind_beam),'k-.');
    end % for veh_num = 1:Veh(1).Event.num_veh
    ylim([0,Calc.Solver.t_end]);
    h4 = plot([0,0],ylim,'k--');
    plot([1,1]*Beam.Prop.Lb,ylim,'k--');
    h5 = plot([1,1]*Calc.Profile.needed_x0,ylim,'k:');
    plot([1,1]*Calc.Profile.needed_x_end,ylim,'k:');
    legend([h1(1),h2(1),h3(1),h4(1),h5(1)],'Vehicle(s)','Beam solver times',...
        'Beam entry/exit times','bridge start/end','Profile start/end');
    xlabel('Distance from Bridge''s left support (m)');
    ylabel('Time (s)');
    title('Vehicle(s) position in time')
    drawnow;
end % if isfield(Calc.Plot,'P3_VehPos')

% ---- End of function ----