function [Beam] = B24_Beam_Damping(Beam)

% Calculates the Beam damping matrix 
%   Rayleigh damping is addopted 
%   1st and 2nd beam frequencies are taken as reference (of non-rigid modes)

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

if Beam.Prop.damp_xi > 0
    
    % Reference frequencies
    ref_w = Beam.Modal.w((1:2)+Beam.Modal.num_rigid_modes);
    
    % Rayleigh's coefficients 'alpha' and 'beta'
    aux1 = (1/2)*[[1/ref_w(1) ref_w(1)];[1/ref_w(2) ref_w(2)]]\[Beam.Prop.damp_xi;Beam.Prop.damp_xi];

    % Damping matrix
    Beam.SysM.C = aux1(1)*Beam.SysM.M + aux1(2)*Beam.SysM.K;

else
    
    % No Damping case
    Beam.SysM.C = Beam.SysM.K*0;
    
end % if Beam.Prop.damp_xi > 0

% ---- End of script ----