function [Calc] = B40_Profile(Calc)

% Gathers together all the functions related to the Profile:
%   Loading of a saved profile
%   Calculation of new profile
%   Saving of generated profile

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

if Calc.Profile.Load.on == 0
    
    % -- Profile Calculation --
    [Calc] = B19_RoadProfile(Calc);

    % -- Saving Profile --
    B38_Profile_Save(Calc);
    
elseif Calc.Profile.Load.on == 1

    % -- Loading Pofile --
    [Calc] = B39_Profile_Load(Calc,1);

end % if Calc.Profile.Load.on == 0

% ---- End of function ----