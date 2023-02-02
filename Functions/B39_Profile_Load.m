function [Calc] = B39_Profile_Load(Calc,varargin)

% If selected, loads a previously saved road profile

% *************************************************************************
% *** Script part of VBI-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% *************************************************************************

% -------------------------------------------------------------------------
% ---- Input ----
% See A00 for description of input variables
% -- Optional --
% disp_op = Set to "1" to display message showing what profile was loaded
% ---- Output ----
% See A00 for description of output variables
% -------------------------------------------------------------------------

% Input processing
disp_on = 0;
if nargin > 1
    disp_on = varargin{1};
end % if nargin > 1

% Loading profile
load([Calc.Profile.Load.path,Calc.Profile.Load.file_name],'Profile');

% Displaying information
if disp_on == 1
    disp(['Loaded profile: ',Calc.Profile.Load.path,Calc.Profile.Load.file_name,'.mat']);
end % if disp_on == 1

% Gathering information
if Calc.Profile.needed_L < Profile.L
    Calc.Profile.dx = Profile.dx;
    Calc.Profile.L = Profile.L;
    Calc.Profile.x = Profile.x;
    Calc.Profile.num_x = Profile.num_x;
    Calc.Profile.h = Profile.h;
    Calc.Profile.Spatial_frq = Profile.Spatial_frq;    
    if isfield(Profile,'Info')
        Calc.Profile.Info = Profile.Info;
    end % if isfield(Profile,'Info')
    if isfield(Profile,'Opt')
        Calc.Profile.Opt = Profile.Opt;
    end % if isfield(Profile,'Opt')
else
    error('Error: Profile loading -> Loaded profile is too short for this event')
end % if Calc.Profile.needed_L < Profile.L


% ---- End of script ----