function [Beam] = B51_BeamProperties(Beam)

% Beam properties given the type and span

% Values based on the tables in the Phd thesis by Yingyan Li, Appendix.A, page 235:
%   Yingyan Li, Factors Afiecting the Dynamic Interaction of Bridges and 
%   Vehicle Loads. PhD thesis, University College Dublin, 2006.

% Three types defined:
%   T beam type bridge for spans 9 to 21m
%   Y beam type bridge for spans 17 to 31m
%   Super-Y beam type bridge for spans 33 to 43m

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

if Beam.Prop.type ~= 0

    % Checking that variables have not been defined already
    has_been_defined = 0;
    if isfield(Beam.Prop,'E')
        has_been_defined = 1; has_been_defined_txt = 'E';
    elseif isfield(Beam.Prop,'rho')
        has_been_defined = 1; has_been_defined_txt = 'rho';
    elseif isfield(Beam.Prop,'I')
        has_been_defined = 1; has_been_defined_txt = 'I';
    end % if isfield(Beam.Prop,'E')
    if has_been_defined == 1
        error(['The beam parameter "',has_been_defined_txt,'" has been defined already!']);
    end % if has_been_defined == 1

    % Definition of common properties
    Beam.Prop.E = 3.5e10;           % Modulus of elasticity (N/m^2)

    % Definition of table values
    % (Values taken from table in appendix A. Columns: L, rho, I)
    if strcmp(Beam.Prop.type,'T')
        Table_values = [...
            9 16875 0.1139;
            11 20625 0.2080;
            13 24375 0.3433;
            15 28125 0.5273;
            17 31875 0.7677;
            19 35625 1.0717;
            21 39375 1.4470];
    elseif strcmp(Beam.Prop.type,'Y')
        Table_values = [...
            17 15002 0.4911;
            19 15741 0.6660;
            21 16530 0.8722;
            23 17419 1.1133;
            25 18358 1.3901;
            27 19372 1.7055;
            29 20486 2.0634;
            31 21650 2.4651];
    elseif strcmp(Beam.Prop.type,'SY')
        Table_values = [...
            33 20952 2.9327;
            35 21752 3.4162;
            37 22552 3.9425;
            39 23352 4.5132;
            41 24152 5.1305;
            43 24952 5.7957];
    end % if strcmp(Beam.Prop.type,'T')

    % Interpolation of parameters dependent on span length
    Beam.Prop.rho = interp1(Table_values(:,1),Table_values(:,2),Beam.Prop.Lb,'pchip',nan);
    Beam.Prop.I = interp1(Table_values(:,1),Table_values(:,3),Beam.Prop.Lb,'pchip',nan);

    % Checking of results
    if isnan(Beam.Prop.rho)
        error('The chosen span length is not valid for the chosen bridge type');
    end % if isnan(Beam.Prop.rho)
    
end % if Beam.Prop.type ~= 0

% ---- End of function ----