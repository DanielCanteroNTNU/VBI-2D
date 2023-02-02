function He = B30_Beam_ele_H(L,E,I)

% Generation of beam element Moment-Displacement matrix

% *************************************************************************
% *** Script part of VBI-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% *************************************************************************

% % ---------------------------------------------------------------
% % ---- Input ----
% L = Length of Beam element
% E = Young's Modulus of the beam element
% I = Moment of inertia of the beam element
% % ---- Output ----
% He = Element Moment-Displacement matrix
% % ---------------------------------------------------------------

He = E*I * ...
    [[ -6/L^2, -4/L,  6/L^2, -2/L];...
    [  6/L^2,  2/L, -6/L^2,  4/L]];

% ---- End of function ----