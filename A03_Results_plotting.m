% Script (not function) to generate the desired plots
% Define the desired plot in the main script of the model by defining the 
% appropriate variable name in "Calc.Plot.XXX"

% *************************************************************************
% *** Script part of VBI-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% *************************************************************************

% ---- Plot 4 = Road profile and 1st derivative ----
if isfield(Calc.Plot,'P4_Profile')
    for veh_num = 1:Veh(1).Event.num_veh
        figure; subplot(2,1,1);
            plot(Veh(veh_num).Pos.wheels_x',Veh(veh_num).Pos.wheels_h'); Ylim = ylim; hold on;
            plot(((Veh(veh_num).Pos.x0_values(1)-Veh(veh_num).Prop.ax_dist*Veh(veh_num).Pos.vel_sign)'*[1,1])',...
                (ones(Veh(veh_num).Prop.num_wheels,1)*Ylim)','k--');
            plot(((Veh(veh_num).Pos.x0_values(1)-Veh(veh_num).Prop.ax_dist*Veh(veh_num).Pos.vel_sign...
                +Veh(veh_num).Pos.wheels_h(:,end)')'*[1,1])',...
                (ones(Veh(veh_num).Prop.num_wheels,1)*Ylim)','k--');
            plot([0;Beam.Prop.Lb],(Ylim(1)+diff(Ylim)*0.1)*[1,1],'k','LineWidth',4);
            plot([1;1]*[0,Beam.Prop.Lb],[Ylim;Ylim]','k:');
            annotation('arrow',abs(sort([2/5;3/5]*Veh(veh_num).Pos.vel_sign)),2/3*[1,1],'Color',[0,0,0]);
            xlabel('Distance (m)'); ylabel('Elevation (m)');
            title(['Profile under each wheel (Vehicle ',num2str(veh_num),')']);
            axis tight;
        subplot(2,1,2);
            plot(Veh(veh_num).Pos.wheels_x',Veh(veh_num).Pos.wheels_hd'); Ylim = ylim; hold on;
            plot(((Veh(veh_num).Pos.x0_values(1)-Veh(veh_num).Prop.ax_dist*Veh(veh_num).Pos.vel_sign)'*[1,1])',...
                (ones(Veh(veh_num).Prop.num_wheels,1)*Ylim)','k--');
            plot(((Veh(veh_num).Pos.x0_values(1)-Veh(veh_num).Prop.ax_dist*Veh(veh_num).Pos.vel_sign...
                +Veh(veh_num).Pos.wheels_h(:,end)')'*[1,1])',...
                (ones(Veh(veh_num).Prop.num_wheels,1)*Ylim)','k--');
            plot([0;Beam.Prop.Lb],(Ylim(1)+diff(Ylim)*0.1)*[1,1],'k','LineWidth',4);
            plot([1;1]*[0,Beam.Prop.Lb],[Ylim;Ylim]','k:');
            xlabel('Distance (m)'); ylabel('1st Derivative m/s');
            axis tight;
    end % for veh_num = 1:Veh(1).Event.num_veh
    clear veh_num Ylim
end % if isfield(Calc.Plot,'p4_Profile')

% ---- Plot 5 = Contour Plot of Beam displacements ----
if isfield(Calc.Plot,'P5_Beam_U')
    figure; hold on; box on;
    [~,h] = contourf(Beam.Mesh.Ele.acum,Calc.Solver.t_beam,Sol.Beam.U.value_xt',20);
    set(h,'EdgeColor','none'); colorbar;
    mycmap = colormap; colormap(flipud(mycmap));
    for veh_num = 1:Veh(1).Event.num_veh
        plot(Veh(veh_num).Pos.wheels_x(:,Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam),...
            Calc.Solver.t_beam,'k--');
    end % for veh_num = 1:Veh(1).Event.num_veh
    xlim([0,Beam.Prop.Lb]);
    xlabel('Distance (m)');
    ylabel('Time (s)');
    title('Beam displacements (m)');
    clear h mycmap veh_num
end % if isfield(Calc.Plot,'P5_Beam_U')

% ---- Plot 6 = Plot of Beam displacements under the vehicle ----
if isfield(Calc.Plot,'P6_Beam_U_under_Veh')
    for veh_num = 1:Veh(1).Event.num_veh
        figure;
            plot(Calc.Solver.t_beam,Sol.Veh(veh_num).Under.def);
            xlim(Calc.Solver.t_beam([1,end]));
            xlabel('Time (s)'); ylabel('Displacement (m)');
            title(['Beam displacement under the Vehicle ',num2str(veh_num)]);
    end % for veh_num = 1:Veh(1).Event.num_veh
    clear veh_num
end % if isfield(Calc.Plot,'P6_Beam_U_under_Veh')

if Calc.Proc.code == 1

    % ---- Plot 7 = Plot Vehicle Displacement for every iteration ----
    if isfield(Calc.Plot,'P7_Veh_U_iter')
        for veh_num = 1:Veh(1).Event.num_veh
            figure; hold on; box on;
            if Calc.Proc.Iter.num>1
                for iter_num = 1:Calc.Proc.Iter.num
                    mylines1 = plot(Calc.Solver.t_beam,Sol.Proc.Iter.Veh(veh_num).U(:,:,iter_num),'k');
                    mygroup = hggroup; set(mylines1,'Parent',mygroup); set(get(get(mygroup,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
                end % for k
                mygroup = hggroup; set(mylines1,'Parent',mygroup); set(get(get(mygroup,'Annotation'),'LegendInformation'),'IconDisplayStyle','on');
                mylines2 = plot(Calc.Solver.t_beam,Sol.Proc.Iter.Veh(veh_num).U(:,:,end),'r');
                mygroup = hggroup; set(mylines2,'Parent',mygroup); set(get(get(mygroup,'Annotation'),'LegendInformation'),'IconDisplayStyle','on');
                legend('Consecutive Iterations','Final Solution');
            else
                plot(Calc.Solver.t_beam,Sol.Proc.Iter.Veh(veh_num).U(:,:,1),'r');
                legend('No iterations');
            end % if Calc.num_iter>1
            xlim(Calc.Solver.t_beam([1,end]));
            xlabel('Time (s)'); ylabel('Displacement (m)');
            title(['Vehicle ',num2str(veh_num),' - Vertical displacement (',num2str(Calc.Proc.Iter.num),' iterations)']);
        end % for veh_num = 1:Veh(1).Event.num_veh
        clear veh_num iter_num mylines1 mygroup mylines1 mylines2
    end % if isfield(Calc.Plot,'P7_Veh_U_iter')

    % ---- Plot 8 = Plot Vehicle Wheel Relative displacement for every iteration ----
    if isfield(Calc.Plot,'P8_Veh_Urel_iter')
        for veh_num = 1:Veh(1).Event.num_veh
            figure; hold on; box on;
            if Calc.Proc.Iter.num>1
                for iter_num = 1:Calc.Proc.Iter.num-1
                    mylines1 = plot(Calc.Solver.t_beam,Sol.Proc.Iter.Veh(veh_num).Wheels.Urel(:,:,iter_num),'k');
                    mygroup = hggroup; set(mylines1,'Parent',mygroup); set(get(get(mygroup,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
                end % for iter_num = 1:Calc.Proc.Iter.num-1
                mygroup = hggroup; set(mylines1,'Parent',mygroup); set(get(get(mygroup,'Annotation'),'LegendInformation'),'IconDisplayStyle','on');
                mylines2 = plot(Calc.Solver.t_beam,Sol.Proc.Iter.Veh(veh_num).Wheels.Urel(:,:,end),'r');
                mygroup = hggroup; set(mylines2,'Parent',mygroup); set(get(get(mygroup,'Annotation'),'LegendInformation'),'IconDisplayStyle','on');
                legend('Consecutive Iterations','Final Solution');
            else
                plot(Calc.Solver.t_beam,Sol.Proc.Iter.Veh(veh_num).Wheels.Urel(:,:,1),'r');
                legend('No iterations');
            end % if Calc.Proc.Iter.num>1
            xlim(Calc.Solver.t_beam([1,end]));
            xlabel('Time (s)'); ylabel('Relative Displacement (m)');
            title(['Vehicle ',num2str(veh_num),' - Wheel Relative vertical displacement (',...
                num2str(Calc.Proc.Iter.num),' iterations)']);
        end % for veh_num = 1:Veh(1).Event.num_veh
        clear veh_num iter_num mylines1 mygroup mylines1 mylines2
    end % if isfield(Calc.Plot,'P8_Veh_Urel_iter')

    % ---- Plot 9 =  Plot of Beam displacements under the vehicle for each iteration ----
    if isfield(Calc.Plot,'P9_Beam_U_under_Veh_iter')
        for veh_num = 1:Veh(1).Event.num_veh
            figure; hold on; box on;
            if Calc.Proc.Iter.num>1
                for iter_num = 1:Calc.Proc.Iter.num-1
                    mylines1 = plot(Calc.Solver.t_beam,Sol.Proc.Iter.Veh(veh_num).def_under(:,:,iter_num),'k');
                    mygroup = hggroup; set(mylines1,'Parent',mygroup); set(get(get(mygroup,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
                end % for iter_num = 1:Calc.Proc.Iter.num-1
                mygroup = hggroup; set(mylines1,'Parent',mygroup); set(get(get(mygroup,'Annotation'),'LegendInformation'),'IconDisplayStyle','on');
                mylines2 = plot(Calc.Solver.t_beam,Sol.Proc.Iter.Veh(veh_num).def_under(:,:,end),'r');
                mygroup = hggroup; set(mylines2,'Parent',mygroup); set(get(get(mygroup,'Annotation'),'LegendInformation'),'IconDisplayStyle','on');
                legend('Consecutive Iterations','Final Solution');
            else
                plot(Calc.Solver.t_beam,Sol.Proc.Iter.Veh(veh_num).def_under(:,:,1),'r');
                legend('No iterations');
            end % if Calc.Proc.Iter.num>1
            xlim(Calc.Solver.t_beam([1,end]));
            xlabel('Time (s)'); ylabel('Displacement (m)');
            title(['Vehicle ',num2str(veh_num),' - Under vehicle beam displacement (',num2str(Calc.Proc.Iter.num),' iterations)']);
        end % for veh_num = 1:Veh(1).Event.num_veh
        clear veh_num iter_num mylines1 mygroup mylines1 mylines2
    end % if isfield(Calc.Plot,'P9_Beam_U_under_Veh_iter')

    % ---- Plot 10 = Iteration Criteria (Displacement difference) ----
    if and(isfield(Calc.Plot,'P10_diff_iter'),Calc.Proc.Iter.num>1)
        if Calc.Proc.Iter.criteria == 1
            for veh_num = 1:Veh(1).Event.num_veh
                figure;
                semilogy((2:Calc.Proc.Iter.num),Calc.Proc.Iter.def_diff(2:end,veh_num),'.-');
                Xlim = xlim; hold on; plot(Xlim,Calc.Proc.Iter.tol*[1,1],'k--');
                legend('Difference','Iterations tolerance');
                xlabel('Iteration number'); ylabel('Solutions difference');
                title(['Vehicle ',num2str(veh_num),' - Criteria: ',Calc.Proc.Iter.criteria_text]);
            end % for veh_num = 1:Veh(1).Event.num_veh
        elseif Calc.Proc.Iter.criteria == 2
            figure;
            semilogy((2:Calc.Proc.Iter.num),Calc.Proc.Iter.BM_diff(2:end),'.-');
            Xlim = xlim; hold on; plot(Xlim,Calc.Proc.Iter.tol*[1,1],'k--');
            legend('Difference','Iterations tolerance');
            xlabel('Iteration number'); ylabel('Solutions difference');
            title(['Criteria: ',Calc.Proc.Iter.criteria_text]);
        end % if Calc.Proc.Iter.criteria == 1
        clear veh_num Xlim
    end % if and(isfield(Calc.Plot,'P10_diff_iter'),Calc.Proc.Iter.num>1)

end % if Calc.Proc.code == 1

% ---- Plot 11 = Static Beam deformation ----
if isfield(Calc.Plot,'P11_Beam_U_static')
    figure; hold on; box on;
    [~,h] = contourf(Beam.Mesh.Ele.acum,Calc.Solver.t_beam,Sol.Beam.U_static.value_xt',20);
    set(h,'EdgeColor','none'); colorbar;
    mycmap = colormap; colormap(flipud(mycmap));
    for veh_num = 1:Veh(1).Event.num_veh
        plot(Veh(veh_num).Pos.wheels_x(:,Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam),Calc.Solver.t_beam,'k--');
    end % for veh_num = 1:Veh(1).Event.num_veh
    xlim([0,Beam.Prop.Lb]);
    xlabel('Distance (m)'); ylabel('Time (s)');
    title('Beam Static displacements (m)');
    clear h mycmap veh_num
end % if isfield(Calc.Plot,'P11_Beam_U_static')

% ---- Plot 13 = Interaction Force ----
if isfield(Calc.Plot,'P13_Interaction_Force')
    for veh_num = 1:Veh(1).Event.num_veh
        figure;
            plot(Calc.Solver.t_beam,Sol.Veh(veh_num).Under.onBeamF);
            xlim(Calc.Solver.t_beam([1,end]));
            axis tight;
            ylim([min(0,min(min(Sol.Veh(veh_num).Under.onBeamF))),max(0,max(max(Sol.Veh(veh_num).Under.onBeamF)))]*1.2);
            xlabel('Time (s)'); ylabel('Force (N)');
            title(['Vehicle ',num2str(veh_num),' - Interaction force']);
    end % for veh_num = 1:Veh(1).Event.num_veh
    clear veh_num
end % if isfield(Calc.Plot,'P13_Interaction_Force')

if Calc.Proc.code == 1

    % ---- Plot 14 =  Plot of Interaction Force for each iteration ----
    if isfield(Calc.Plot,'P14_Interaction_Force_iter')
        for veh_num = 1:Veh(1).Event.num_veh
            figure; hold on; box on;
            if Calc.Proc.Iter.num>1
                for iter_num = 1:Calc.Proc.Iter.num-1
                    mylines1 = plot(Calc.Solver.t_beam,Sol.Proc.Iter.Veh(veh_num).onBeamF(:,:,iter_num),'k');
                    mygroup = hggroup; set(mylines1,'Parent',mygroup); set(get(get(mygroup,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
                end % for iter_num = 1:Calc.Proc.Iter.num-1
                mygroup = hggroup; set(mylines1,'Parent',mygroup); set(get(get(mygroup,'Annotation'),'LegendInformation'),'IconDisplayStyle','on');
                mylines2 = plot(Calc.Solver.t_beam,Sol.Proc.Iter.Veh(veh_num).onBeamF(:,:,end),'r');
                mygroup = hggroup; set(mylines2,'Parent',mygroup); set(get(get(mygroup,'Annotation'),'LegendInformation'),'IconDisplayStyle','on');
                legend('Consecutive Iterations','Final Solution');
            else
                plot(Calc.Solver.t_beam,Sol.Proc.Iter.Veh(veh_num).onBeamF(:,:,1),'r');
                legend('No iterations');
            end % if Calc.num_iter>1    
            xlim(Calc.Solver.t_beam([1,end]));
            xlabel('Time (s)'); ylabel('Force (N)');
            title(['Vehicle ',num2str(veh_num),' - Interaction Force (',num2str(Calc.Proc.Iter.num),' iterations)']);
        end % for veh_num = 1:Veh(1).Event.num_veh
        clear veh_num iter_num mygroup mylines1 mylines2
    end % if isfield(Calc.Plot,'P14_Interaction_Force_iter')

end % if Calc.Proc.code == 1

% ---- Plot 16 = Contour Plot of Beam BM ----
if isfield(Calc.Plot,'P16_Beam_BM')
    figure; hold on; box on;
    % Total BM
    [~,h] = contourf(Beam.Mesh.Ele.acum,Calc.Solver.t_beam,Sol.Beam.BM.value_xt',20);
    % Dynamic component only (Total-Static)
    %[~,h] = contourf(Beam.Mesh.Ele.acum,Calc.Solver.t_beam,Sol.Beam.BM.value_xt'-Sol.Beam.BM_static.value_xt',20);
    set(h,'EdgeColor','none'); colorbar;
    for veh_num = 1:Veh(1).Event.num_veh
        plot(Veh(veh_num).Pos.wheels_x(:,Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam),Calc.Solver.t_beam,'k--');
    end % for veh_num = 1:Veh(1).Event.num_veh
    xlim([0,Beam.Prop.Lb]);
    xlabel('Distance (m)'); ylabel('Time (s)');
    title('Beam BM (N*m)');
    clear h veh_num
end % if isfield(Calc.Plot,'P16_Beam_BM'

% ---- Plot 17 = Contour Plot of Beam Static BM ----
if isfield(Calc.Plot,'P17_Beam_BM_static')
    figure; hold on; box on;
    [~,h] = contourf(Beam.Mesh.Ele.acum,Calc.Solver.t_beam,Sol.Beam.BM_static.value_xt',20);
    set(h,'EdgeColor','none'); colorbar;
    for veh_num = 1:Veh(1).Event.num_veh
        plot(Veh(veh_num).Pos.wheels_x(:,Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam),Calc.Solver.t_beam,'k--');
    end % for veh_num = 1:Veh(1).Event.num_veh
    xlim([0,Beam.Prop.Lb]);
    xlabel('Distance (m)'); ylabel('Time (s)');
    title('Beam Static BM (N*m)');
    clear h veh_num
end % if isfield(Calc.Plot,'P17_Beam_BM_static')

% ---- Plot 18 = Contour Plot of Beam Shear ----
if isfield(Calc.Plot,'P18_Beam_Shear')
    figure; hold on; box on;
    [~,h] = contourf(Beam.Mesh.Ele.acum,Calc.Solver.t_beam,Sol.Beam.Shear.value_xt',20);
    set(h,'EdgeColor','none'); colorbar;
    for veh_num = 1:Veh(1).Event.num_veh
        plot(Veh(veh_num).Pos.wheels_x(:,Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam),Calc.Solver.t_beam,'k--');
    end % for veh_num = 1:Veh(1).Event.num_veh
    xlim([0,Beam.Prop.Lb]);
    xlabel('Distance (m)'); ylabel('Time (s)');
    title('Beam Shear (N)');
    clear h veh_num
end % if isfield(Calc.Plot,'P18_Beam_Shear'

% ---- Plot 19 = Contour Plot of Beam Static Shear ----
if isfield(Calc.Plot,'P19_Beam_Shear_static')
    figure; hold on; box on;
    [~,h] = contourf(Beam.Mesh.Ele.acum,Calc.Solver.t_beam,Sol.Beam.Shear_static.value_xt',20);
    set(h,'EdgeColor','none'); colorbar;
    for veh_num = 1:Veh(1).Event.num_veh
        plot(Veh(veh_num).Pos.wheels_x(:,Calc.Solver.t0_ind_beam:Calc.Solver.t_end_ind_beam),Calc.Solver.t_beam,'k--');
    end % for veh_num = 1:Veh(1).Event.num_veh
    xlim([0,Beam.Prop.Lb]);
    xlabel('Distance (m)'); ylabel('Time (s)');
    title('Beam Static Shear (N)');
    clear h veh_num
end % if isfield(Calc.Plot,'P19_Beam_Shear_static')

% ---- Plot 20 = Static and Dynamic BM at mid-span ----
if isfield(Calc.Plot,'P20_Beam_MidSpan_BM')
    if isempty(Beam.Mesh.Node.at_mid_span)
        disp('P20 cannot be plotted. The used mesh has no node at mid-span');
    else
        figure; hold on; box on;
        % Static and Total
        plot(Calc.Solver.t_beam,Sol.Beam.BM.value_xt(Beam.Mesh.Node.at_mid_span,:)/1000)
        plot(Calc.Solver.t_beam,Sol.Beam.BM_static.value_xt(Beam.Mesh.Node.at_mid_span,:)/1000,'k--');
        xlabel('Calculation time (s)');
        ylabel('Mid-span Bending Moment (kN*m)');
        xlabel('BM at mid-span');
        %legend('Dynamic','Static'); 
        axis tight;
    end % if isempty(Beam.Mesh.Node.at_mid_span)
end % if isfield(Calc.Plot,'P20_Beam_MidSpan_BM')

if Calc.Proc.code == 1

    % ---- Plot 21 = Static and Dynamic BM at mid-span for various iterations ----
    if and(isfield(Calc.Plot,'P21_Beam_MidSpan_BM_iter'),Calc.Proc.Iter.criteria == 2)
        if isempty(Beam.Mesh.Node.at_mid_span)
            disp('P21 cannot be plotted. The used mesh has no node at mid-span');
        else
            figure; hold on; box on;
            if Calc.Proc.Iter.num>1
                for num_iter = 1:Calc.Proc.Iter.num-1
                    mylines1 = plot(Calc.Solver.t_beam,Sol.Proc.Iter.BM05_iter(:,num_iter)/1000,'k');
                    mygroup = hggroup; set(mylines1,'Parent',mygroup); set(get(get(mygroup,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
                end % for num_iter = 1:Calc.Proc.Iter.num-1
                mygroup = hggroup; set(mylines1,'Parent',mygroup); set(get(get(mygroup,'Annotation'),'LegendInformation'),'IconDisplayStyle','on');
                mylines2 = plot(Calc.Solver.t_beam,Sol.Proc.Iter.BM05_iter(:,end)/1000,'r');
                mygroup = hggroup; set(mylines2,'Parent',mygroup); set(get(get(mygroup,'Annotation'),'LegendInformation'),'IconDisplayStyle','on');
                mylines3 = plot(Calc.Solver.t_beam,Sol.Beam.BM_static.value_xt(Beam.Mesh.Node.at_mid_span,:)/1000,'k--');
                mygroup = hggroup; set(mylines3,'Parent',mygroup); set(get(get(mygroup,'Annotation'),'LegendInformation'),'IconDisplayStyle','on');
                legend('Consecutive Iterations','Final Solution','Static','Location','South');
            else
                plot(Calc.Solver.t_beam,Sol.Proc.Iter.BM05_iter(:,1)/1000,'r');
                plot(Calc.Solver.t_beam,Sol.Beam.BM_static.value_xt(Beam.Mesh.Node.at_mid_span,:)/1000,'k--');
                legend('No iterations','Static');
            end % if Calc.num_iter>1    
            xlim(Calc.Solver.t_beam([1,end]));
            xlabel('Time (s)'); ylabel('Bending Moment (kN*m)');
            title(['Mid-span Bending Moments (',num2str(Calc.Proc.Iter.num),' iterations)']);
            clear k mygroup mylines1 mylines2 mylines3 num_iter
        end % if isempty(Beam.Mesh.Node.at_mid_span)
    end % if and(isfield(Calc.Plot,'P21_Beam_MidSpan_BM_iter'),Calc.Proc.Iter.criteria == 2)

end % if Calc.Proc.code == 1

if Calc.Proc.code == 2

    % ---- Plot 22 = Number of iterations for each time step (Only for SSI procedure) ----
    if isfield(Calc.Plot,'P22_SSI_num_iterations')
        xdata = 1:Calc.Solver.num_t_beam-1;
        figure; hold on; box on;
            plot(xdata,Calc.Proc.Iter.num_t_bri,'.','MarkerSize',6);
            inds = (Calc.Proc.Iter.max_iter_reached_t_bri==1);
            plot(xdata(inds),Calc.Proc.Iter.num_t_bri(inds),'r.','MarkerSize',6);
            plot(xdata([1,end]),Calc.Proc.Iter.max_num*[1,1],'k--');
            xlim(xdata([1,end]));
            ylim([0,Calc.Proc.Iter.max_num+1]);
            xlabel('Time step (on beam)');
            ylabel('Number of iterations');
            clear xdata inds
    end % if isfield(Calc.Plot,'P22_SSI_num_iterations')
    
end % if Calc.Proc.code == 1

% ---- Plot 23 = Vehicles DOF responses (Displacement, velocity and acceleration) ----
if isfield(Calc.Plot,'P23_vehicles_responses')
    xdata = Calc.Solver.t;
    for veh_num = 1:Veh(1).Event.num_veh
        figure; subplot(3,1,1); hold on; box on;
            ydata = Sol.Veh(1).U;
            plot(xdata,ydata);
            axis tight;
            xlabel('Time (s)'); ylabel('Displacements (m)');
            title(['Vehicle ',num2str(veh_num),' - DOF responses']);
        subplot(3,1,2); hold on; box on;
            ydata = Sol.Veh(1).V;
            plot(xdata,ydata);
            axis tight;
            xlabel('Time (s)'); ylabel('Velocities (m/s)');
        subplot(3,1,3); hold on; box on;
            ydata = Sol.Veh(1).A;
            plot(xdata,ydata);
            axis tight;
            xlabel('Time (s)'); ylabel('Accelerations (m/s^2)');
    end % for veh_num = 1:Veh(1).Event.num_veh
    clear xdata ydata veh_num
end % if isfield(Calc.Plot,'P23_vehicles_responses')

% ---- Plot 24 = Responses at the wheels of the vehicle ----
if isfield(Calc.Plot,'P24_vehicles_wheel_responses')
    xdata = Calc.Solver.t;
    for veh_num = 1:Veh(1).Event.num_veh
        figure; subplot(4,1,1);
            plot(xdata,Sol.Veh(veh_num).Wheels.U);
            xlabel('Time (s)'); ylabel('Displacements (m)');
            title(['Vehicle ',num2str(veh_num),': Wheel responses']);
            axis tight;
        subplot(4,1,2);
            plot(xdata,Sol.Veh(veh_num).Wheels.Urel);
            xlabel('Time (s)'); ylabel('Relative Disp. (m)');
            axis tight;
        subplot(4,1,3);
            plot(xdata,Sol.Veh(veh_num).Wheels.V);
            xlabel('Time (s)'); ylabel('Velocities (m/s)');
            axis tight;
        subplot(4,1,4);
            plot(xdata,Sol.Veh(veh_num).Wheels.Vrel);
            xlabel('Time (s)'); ylabel('Relative Vel. (m/s)');
            axis tight;
    end % for veh_num = 1:Veh(1).Event.num_veh
    clear xdata veh_num
end % if isfield(Calc.Plot,'P24_vehicles_wheel_responses')

% ---- Plot 25 = Responses under the wheel (Bridge response at the contact point) ----
if isfield(Calc.Plot,'P25_vehicles_under_responses')
    xdata = Calc.Solver.t_beam;
    for veh_num = 1:Veh(1).Event.num_veh
        figure; subplot(2,1,1);
            plot(xdata,Sol.Veh(veh_num).Under.def);
            xlabel('Time (s)'); ylabel('Deformation (m)');
            title(['Vehicle ',num2str(veh_num),': Responses Under wheels (Bridge response at the contact point)']);
            axis tight;
        subplot(2,1,2);
            plot(xdata,Sol.Veh(veh_num).Under.vel);
            xlabel('Time (s)'); ylabel('Velocity (m/s)');
            axis tight;
    end % for veh_num = 1:Veh(1).Event.num_veh
    clear xdata veh_num
end % if isfield(Calc.Plot,'P25_vehicles_under_responses') 

% ---- Plot 26 = PSD of vehicle accelerations ----
if isfield(Calc.Plot,'P26_PSD_Veh_A')
    for veh_num = 1:Veh(1).Event.num_veh
        figure;
        num_DOF = Veh(veh_num).DOF.num_independent;
        sub_num_cols = ceil(sqrt(num_DOF));
        sub_num_rows = ceil(num_DOF/sub_num_cols);
        for DOF_num = 1:num_DOF
            subplot(sub_num_rows,sub_num_cols,DOF_num);
            myPSDplot(Sol.Veh(veh_num).A(DOF_num,:),1/Calc.Solver.dt,4);
            title(['DOF ',num2str(DOF_num)]);
        end % for DOF_num = 1:Veh(veh_num).DOF.num_independent
    end % for veh_num = 1:Veh(1).Event.num_veh
    clear veh_num num_DOF sub_num_rows sub_num_cols DOF_num
end % if isfield(Calc.Plot,'P26_PSD_Veh_A')

% ---- Plot 27 = Vehicle Accelerations ----
if isfield(Calc.Plot,'P27_Veh_A')
    for veh_num = 1:Veh(1).Event.num_veh
        figure;
        num_DOF = Veh(veh_num).DOF.num_independent;
        for DOF_num = 1:num_DOF
            subplot(num_DOF,1,DOF_num)
            plot(Calc.Solver.t,Sol.Veh(veh_num).A(DOF_num,:));
            axis tight;
            ylabel(['DOF ',num2str(DOF_num)]);
            if DOF_num == 1
                title('Vehicle accelerations (m/s^2)');
            end % if DOF_num == 1
        end % for DOF_num = 1:num_DOF
        xlabel('Time (s)');
    end % for veh_num = 1:Veh(1).Event.num_veh
    clear veh_num num_DOF DOF_num 
end % if isfield(Calc.Plot,'P27_Veh_A')

% ---- End of script ----