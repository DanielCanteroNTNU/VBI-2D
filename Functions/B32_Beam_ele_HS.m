function HSe = B32_Beam_ele_HS(L,E,I)

% Generation of beam element Force-Displacement matrix

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
% HSe = Element Force-Displacement matrix
% % ---------------------------------------------------------------

HSe = E*I * ...
    [[  12/L^3,  6/L^2, -12/L^3,  6/L^2];...
    [ 12/L^3, 6/L^2,  -12/L^3, 6/L^2]];

% ---- End of function ----