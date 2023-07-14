% Plotting of validation results for VBI-2D

% *************************************************************************
% *** Script part of VBI-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% *************************************************************************

clear; clc;

% Definitions of files to load
Load.Num.subfolder = 'Validation\';
Load.Num.files2load = {...
    'Validation_Numerical_SprungVeh',...
    'Validation_Numerical_DampOn',...
    'Validation_Numerical_RampOn',...
    'Validation_Numerical_AccOn',...
    };
Load.Analytic.subfolder = 'Validation\';
Load.Analytic.files2load = {...
    'Validation_Analytical_SprungVeh',...
    'Validation_Analytical_DampOn',...
    'Validation_Analytical_RampOn',...
    'Validation_Analytical_AccOn',...
    };
Load.legends = {'Analytical: VBI','Analytical: VBI+Ramp','Analytical: VBI+Ramp+Damping','Analytical: VBI+Ramp+Damping+Acc','Numerical'};
% Auxiliary calculations
Load.Num.num_files2load = size(Load.Num.files2load,2);
Load.Analytic.num_files2load = size(Load.Analytic.files2load,2);

% ---- Plotting ----

% Plot options
LineWidth_1 = 2;
LineWidth_2 = 1.5;
myColors = {}; k = 0;
k=k+1; myColors{k} = [0.4660 0.6740 0.1880];
k=k+1; myColors{k} = [0.9290 0.6940 0.1250];
k=k+1; myColors{k} = [0.8500 0.3250 0.0980].*[1.1 1 1];
k=k+1; myColors{k} = [0.6350 0.0780 0.1840].*[1.3 1 1];
Xlims = zeros(1,2);

% Plot generation
figure; hold on; box on;
    for file_num = 1:Load.Num.num_files2load
        load([Load.Analytic.subfolder,Load.Analytic.files2load{file_num}]);
        h = plot(Data.Time.values,Data.MidSpan.Disp.values*1000,'LineWidth',LineWidth_1);
        set(h,'Color',myColors{file_num});
        aux1 = xlim;
        Xlims = [min(Xlims(1),aux1(1)),max(Xlims(2),aux1(2))];
    end % for file_num = 1:Load.Num.num_files2load
    for file_num = 1:Load.Num.num_files2load
        load([Load.Num.subfolder,Load.Num.files2load{file_num}]);
        h = plot(0,0,'--','Color',[1,1,1]*0.2,'LineWidth',LineWidth_2);
        dashline(Data.Time.values,Data.MidSpan.Disp.values*1000,2,2,2,2,'Color',[1,1,1]*0,'LineWidth',LineWidth_2);
    end % for file_num = 1:Num.num_files2load
    hLegend = legend(Load.legends,'Location','northwest');
    set(hLegend.BoxFace, 'ColorType','truecoloralpha', 'ColorData',uint8(255*[.9;.9;.9;.8]));  % [.5,.5,.5] is light gray; 0.8 means 20% transparent
    axis tight;
    xlim(Xlims);
    ylim(ylim+diff(ylim)*[-5,+5]/100);
    xlabel('Time (s)')
    ylabel('Vertical displacement (mm)')

% ---- End of script ----