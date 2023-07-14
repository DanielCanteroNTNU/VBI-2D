function [Calc] = B19_RoadProfile(Calc)

% Calculates the profile

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

% Profile space discretization
Calc.Profile.x = -Calc.Profile.L/2:Calc.Profile.dx:Calc.Profile.L/2;

% Number of samples
Calc.Profile.num_x = length(Calc.Profile.x);

% Random number generator seed
if myIsfield(Calc.Profile,{'Opt',1,'rng_seed'})
    rng(Calc.Profile.Opt.rng_seed);
end % if myIsfield(Calc.Profile,{'Opt',1,'rng_seed'})

% ---- Type of Road Profile ----

% -- Smooth --
if Calc.Profile.type == 0
    
    Calc.Profile.h = zeros(1,Calc.Profile.num_x);
    Calc.Profile.Gd = 0;
    
% -- ISO profile generator --    
elseif Calc.Profile.type == 2

    [~,Calc.Profile.h,Calc.Profile.Info.Gd,Calc.Profile.Info.PSD_x,Calc.Profile.Info.PSD_y] = ... 
       B21_ISO_Profile(Calc.Profile.L,Calc.Profile.dx,...
           Calc.Profile.Info.class,Calc.Profile.Opt.class_var,...
           Calc.Profile.Spatial_frq.min,Calc.Profile.Spatial_frq.max,...
           Calc.Profile.Spatial_frq.ref,Calc.Profile.Info.Gd_limits);

% -- Step profile --
elseif Calc.Profile.type == 3

    Calc.Profile.h = zeros(1,Calc.Profile.num_x);
    Calc.Profile.h(Calc.Profile.x>=Calc.Profile.step_loc_x) = Calc.Profile.step_height;

% -- Ramp profile --
elseif Calc.Profile.type == 4

    Calc.Profile.h = zeros(1,Calc.Profile.num_x);
    Calc.Profile.h = interp1([Calc.Profile.x(1),Calc.Profile.ramp_loc_x_0,Calc.Profile.ramp_loc_x_end,Calc.Profile.x(end)],...
        [0,0,Calc.Profile.ramp_height*[1,1]],Calc.Profile.x);

% -- Sinewave profile --
elseif Calc.Profile.type == 5

    Calc.Profile.h = Calc.Profile.sin_Amp*...
        sin((2*pi/Calc.Profile.sine_wavelength)*Calc.Profile.x+Calc.Profile.sin_phase);

end % if Calc.Profile.type

% ---- Moving Average for profile ---- (Only uniformly spaced)
if myIsfield(Calc.Profile,{'Opt',1,'MovAvg',1,'on'},1)
    
    % Copy of original profile
    original_h = Calc.Profile.h;
    % Moving average filter
    [Calc.Profile.h] = B20_MovAvg(Calc.Profile.x,Calc.Profile.h,Calc.Profile.Opt.MovAvg.window_L);

end % if myIsfield(Calc.Profile,{'Opt',1,'MovAvg',1,'on'},1)

% ---- Plotting profile ----
if isfield(Calc.Plot,'Profile_original')
    figure; hold on; box on;
        h{1} = plot(Calc.Profile.x,Calc.Profile.h);
        ylim(ylim+diff(ylim)*[-1,1]*0.2);
        plot([0,0],ylim,'k--');
        if myIsfield(Calc.Profile,{'Opt',1,'MovAvg',1,'on'},1)
            h{2} = plot(Calc.Profile.x,original_h);
            legend([h{2},h{1}],'Original','Moving Average','Location','Best');
        end % if myIsfield(Calc.Profile,{'Opt',1,'MovAvg',1,'on'},1)
        aux1 = get(gcf,'Position'); set(gcf,'Position',[aux1(1:2),500,250]);
        xlim([Calc.Profile.x(1),Calc.Profile.x(end)]);
        xlabel('Distance (m)'); ylabel('Profile Elevation');
    drawnow;
end % if isfield

% ---- End of function ----