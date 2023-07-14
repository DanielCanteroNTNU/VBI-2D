function [Calc,Veh] = B08_VehFreq(Calc,Veh)

% Calculates the vehicle(s) frequencies given the system matrices

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

if Calc.Opt.veh_frq == 1

    for veh_num = 1:Veh(1).Event.num_veh
    
        % ---- Eigenvalue analysis ----
        aux1 = eig(Veh(veh_num).SysM.K,Veh(veh_num).SysM.M);
        Veh(veh_num).Modal.w = sqrt(aux1);                      % Vehicle circuar frequencies (rad/s)
        Veh(veh_num).Modal.f = Veh(veh_num).Modal.w/(2*pi);     % Vehicle frequecies (Hz)
    
    end % for veh_num = 1:Veh(1).Event.num_veh

end % if Calc.Opt.veh_frq == 1

% ---- End of function ----