function [prof_y] = B20_MovAvg(prof_x,prof_y,window_L)

% Applies the moving average filter on a given profile via convolution operation

% *************************************************************************
% *** Script part of VBI-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% *************************************************************************

% -------------------------------------------------------------------------
% % ---- Inputs ----
% prof_x = X coordenates of the profile (m)
% prof_y = Matrix containing Y coordenates of the profiles in rows (m)
% window_L = Window length in [m]
% % ---- Outputs ----
% prof_y = New smoother profile
% -------------------------------------------------------------------------

% Input processing
dx = prof_x(2) - prof_x(1);

% Window definition
window_size = round(window_L/dx+1);
window = ones(1,window_size)/window_size;

% Convolution of padded array
prof_y = conv([ones(1,window_size)*prof_y(1),prof_y,ones(1,window_size)*prof_y(end)],window,'same');

% Reduction
prof_y = prof_y(window_size+1:end-window_size);

% ---- End of script ----