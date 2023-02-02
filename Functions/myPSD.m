function [PSD_x,PSD_y] = myPSD(signal,Fs,varargin)

% Generates the PSD of a given signal

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
% approx_type = The method used to calculate the approximation of the PSD
%       1 = Periodogram (The squared magnitude of the FFT of the signal) DEFAULT
%       2 = Welch (Average of periodograms of several overlaping signal sections)
% Property name = String with name of property to change
% Property value = New value of the mentioned property
% ---- Output ----
% PSD_x = X axis of the PSD
%   Units = The same as Fs
%   To obtain the circular frequency multiply by 2*pi
%       In space = Omega = 2*pi/m
%       In time = omega = 2*pi/s
% PSD_y = Y axis of the PSD
% -------------------------------------------------------------------------

% ---- Selecting default values ----
approx_type = 1;
if length(varargin) > 0
    if ~isempty(varargin{1})
        % approx_type
        approx_type = varargin{1};
    end % if ~isempty(varargin{1})
end % if length(varargin) > 0

% ---- Type of approximation ----
if approx_type == 1
    h = spectrum.periodogram;
elseif approx_type == 2
    h = spectrum.welch;
    % Changing other properties
    if length(varargin) > 2
        for k = 2:2:length(varargin)
            h.(varargin{k}) = varargin{k+1};
        end % for k = 2:2:length(varargin)
    end % if length(varargin) > 2
end % if approx_type == 1

PSD = psd(h,signal,'Fs',Fs);

% ---- Output Generation ----
PSD_x = PSD.Frequencies;
PSD_y = PSD.Data;

% ---- End of function ----