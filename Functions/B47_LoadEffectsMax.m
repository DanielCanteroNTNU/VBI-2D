function [Sol] = B47_LoadEffectsMax(Beam,Sol,varargin)

% To extract maximum/minimum of all the calculated beam load effects

% *************************************************************************
% *** Script part of VBI-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% *************************************************************************

% -------------------------------------------------------------------------
% ---- Input ----
% See A00 for description of input variables
% ---- Optional Inputs ----
% field_names = Cell array with the field names in Sol.Beam to consider
% ---- Output ----
% See A00 for description of output variables
% -------------------------------------------------------------------------

% Load effects to consider
field_names = fields(Sol.Beam);

% Optional input processing
if nargin > 2
    if ~isempty(varargin{1})
        field_names = varargin{1};
    end % if ~isempty(varargin{1})
end % if nargin > 2

% Load effect loop
for field_num = 1:length(field_names)
    
    % Field name
    field_name = field_names{field_num};

    % Create value_xt variable (if necessary)
    if isfield(Sol.Beam.(field_name),'value_DOFt')
        Sol.Beam.(field_name).value_xt = Sol.Beam.(field_name).value_DOFt(1:2:end,:);
    end % if isfield(Sol.Beam.(field_name),'value_DOFt')
    
    % Maximum Bending Moment
    [Sol.Beam.(field_name).Max.value,aux1] = max(Sol.Beam.(field_name).value_xt);
    [Sol.Beam.(field_name).Max.value,aux2] = max(Sol.Beam.(field_name).Max.value);
    Sol.Beam.(field_name).Max.COP = Beam.Mesh.Ele.acum(aux1(aux2));
    Sol.Beam.(field_name).Max.pCOP = Sol.Beam.(field_name).Max.COP/Beam.Prop.Lb*100;
    Sol.Beam.(field_name).Max.cri_t_ind = aux2;

    % Mid-span Maximum BM
    if ~isempty(Beam.Mesh.Node.at_mid_span)
        Sol.Beam.(field_name).Max.value05 = max(Sol.Beam.(field_name).value_xt(Beam.Mesh.Node.at_mid_span,:));
    end % if ~isempty(Beam.Mesh.Node.at_mid_span)

    % Minimum Bending Moment
    [Sol.Beam.(field_name).Min.value,aux1] = min(Sol.Beam.(field_name).value_xt);
    [Sol.Beam.(field_name).Min.value,aux2] = min(Sol.Beam.(field_name).Min.value);
    Sol.Beam.(field_name).Min.COP = Beam.Mesh.Ele.acum(aux1(aux2));
    Sol.Beam.(field_name).Min.pCOP = Sol.Beam.(field_name).Min.COP/Beam.Prop.Lb*100;
    Sol.Beam.(field_name).Min.cri_t_ind = aux2;

    % Mid-span Bending Moment
    if ~isempty(Beam.Mesh.Node.at_mid_span)
        Sol.Beam.(field_name).Min.value05 = min(Sol.Beam.(field_name).value_xt(Beam.Mesh.Node.at_mid_span,:));
    end % if ~isempty(Beam.Mesh.Node.at_mid_span)

end % for field_num = 1:length(field_names)

% ---- End of function ----