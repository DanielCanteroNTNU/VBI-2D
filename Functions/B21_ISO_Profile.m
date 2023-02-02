function [prof_x,prof_y,Gd,ISO_spaf,ISO_PSD] = ...
    B21_ISO_Profile(L,dx,Gd,class_var,spaf_min,spaf_max,spaf_ref,Gd_limits)

% Program to generate road profiles based on Spectra implied by
% ISO standard method of reporting profile data. ISO standard assumes
% PSD is of the form Gd(n0)*(n/n0)^-w where n0 = spaf_ref cycles/m (Input)
% This method assumes that the pavement is isotropic.

% Based on the specifications found in:
% International Organization for Standardization. Mechanical vibration – Road
%   surface profiles – Reporting of measured data. ISO 8608; 1995.

% *************************************************************************
% *** Script part of VBI-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% *************************************************************************

% % -------------------------------------------------------------------------
% % ---- Input ----
% L = Length of profile on X direcction (m)
% dx = Spatial sampling distance on X direction
% Gd = Roughness coeficient in ISO form [m^3]
%       If Gd = value, this is the value to be used
%       If Gd = A, random Gd value for class A
%       If Gd = B, random Gd value for class B
%       If Gd = C, random Gd value for class C
%       If Gd = D, random Gd value for class D
%       If Gd = E, random Gd value for class E
%       If Gd = F, random Gd value for class F
%       If Gd = G, random Gd value for class G
%       If Gd = H, random Gd value for class H
% class_var = 
%       0 = Centre value for specified Gd range selected
%       1 = Random value of Gd within the profile class selected
% spaf_min = Minimum spatial frequency to be considered
% spaf_max = Maximum spatial frequency to be considered
% spaf_ref = Reference spatial frequency for the definition of ISO PSD
% Gd_limits = As defined in code
% ---- Output ----
% prof_x = X values of the profile
% prof_y = Y values of the profile
% Gd = Roughness coeficient used
% ISO_spaf = X coordinates of ISO PSD (Spatial frequency)
% ISO_PSD = Y coordinates of ISO PSD [m^3]
% -------------------------------------------------------------------------

% --- Classification of road roughness proposed by ISO ---
if class_var == 0
    aux1 = 0.5;
elseif class_var == 1
    aux1 = rand;
end % class_var == 0

if Gd == 'A'      % ISO Class A
    Gd = Gd_limits(1) + diff(Gd_limits([1,2]))*aux1;
elseif Gd == 'B'  % ISO Class B
    Gd = Gd_limits(2) + diff(Gd_limits([2,3]))*aux1;
elseif Gd == 'C'  % ISO Class C
    Gd = Gd_limits(3) + diff(Gd_limits([3,4]))*aux1;
elseif Gd == 'D' % ISO Class D
    Gd = Gd_limits(4) + diff(Gd_limits([4,5]))*aux1;
elseif Gd == 'E' % ISO Class E
    Gd = Gd_limits(5) + diff(Gd_limits([5,6]))*aux1;
elseif Gd == 'F' % ISO Class F
    Gd = Gd_limits(6) + diff(Gd_limits([6,7]))*aux1;
elseif Gd == 'G' % ISO Class G
    Gd = Gd_limits(7) + diff(Gd_limits([7,8]))*aux1;
elseif Gd == 'H' % ISO Class H
    Gd = Gd_limits(8) + Gd_limits(8)/2*aux1;
end % if Gd == 'A'

% Default values
w = 2;                  % The unevenness coefficient

% Auxiliary variables
prof_x = (0:dx:L);      % X values of the profile (Output)
nx = length(prof_x);    % Number of profile sample points
N = 2^nextpow2(nx);     % Number of frequencies to calculate

% ---- ISO spectrum ----- (1 sided PSD)
ISO_spaf = (1/dx)*(0:N/2)/N;            % Output
ISO_PSD = Gd*(ISO_spaf/spaf_ref).^(-w); % Output

% Application of spatial frequency limits
ISO_PSD(ISO_spaf<spaf_min) = 0;
ISO_PSD(ISO_spaf>spaf_max) = 0;

% ---- 2-sided PSD ----
PSD2 = ISO_PSD/2;
PSD2([1,end]) = 0;
PSD2 = [PSD2,fliplr(PSD2(2:end-1))];

% ---- Fourier transform coefficients ----
Amplitude = sqrt(PSD2*N/dx);
phase = 2*pi*rand(1,N/2-1);
phase = [0,phase,0,-fliplr(phase)];
FFT_y = Amplitude .* exp(1i*phase);

% ---- Inverse Fourier Transform ----
iFFT_y = ifft(FFT_y,N);

% ---- Reducing results to desired output ----
prof_y = iFFT_y(1:nx);

% ---- End of script ----