function [] = myPSDplot(signal,Fs,varargin)

% Plots the PSD of a given signal
% Note that this function calls myPSD.m

% *************************************************************************
% *** Script part of VBI-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% *************************************************************************

% -------------------------------------------------------------------------
% ---- Input ----
% signal = Array with the signal to analyse
% Fs = sampling frequency, wich is the 1/dx or 1/dt where dx and dt is the
%   distance / time between two consecutive points
% -- Optional input --
% Plot_type = Selection of the scale of the plot axis
%       1 = [X,Y] = [Linear,Linear]
%       2 = [X,Y] = [Linear,Log]
%       3 = [X,Y] = [Log,Linear]
%       4 = [X,Y] = [Log,Log]
% approx_type = The method used to calculate the approximation of the PSD
%       1 = Periodogram (The squared magnitude of the FFT of the signal) DEFAULT
%       2 = Welch (Average of periodograms of several overlaping signal sections)
% ---- Output ----
% -------------------------------------------------------------------------

% ---- Getting figure's limits ----
Xlim = xlim;

% ---- Selecting default values ----
if length(varargin) >= 1
    Plot_type = varargin{1};
else
    Plot_type = 1;
end % if length(varargin) >= 1
    
if length(varargin) >= 2
    approx_type = varargin{2};
else
    approx_type = 1;
end % if length(varargin) >= 2

% ---- Calculation of PSD ----
[PSD_x,PSD_y] = myPSD(signal,Fs,approx_type);

% ---- Plot generation ----
if Plot_type == 1
    plot(PSD_x,PSD_y);
elseif Plot_type == 2
    semilogy(PSD_x,PSD_y);
elseif Plot_type == 3
    semilogx(PSD_x,PSD_y);
elseif Plot_type == 4
    loglog(PSD_x,PSD_y);
end

% ---- Axis Labels ----
xlabel('Frequency'); 
ylabel('PSD');

% ---- Axis limits ----
if all(Xlim==[0,1])
    xlim([PSD_x(1),PSD_x(end)]);
end % if all(Xlim==[0,1])

% ---- End of function ----