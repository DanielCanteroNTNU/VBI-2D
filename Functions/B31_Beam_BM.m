function [Sol] = B31_Beam_BM(Calc,Beam,Sol,varargin)

% Calculates the BM of the beam using the nodal displacements

% *************************************************************************
% *** Script part of VBI-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% *************************************************************************

% -------------------------------------------------------------------------
% ---- Input ----
% See A00 for description of input variables
% ---- Optional inputs ----
% static_flag = Calculate the static BM if this optional input is equal to 1
% ---- Output ----
% See A00 for description of output variables
% -------------------------------------------------------------------------

% Input processing
static_flag = 0;
if nargin > 3
    static_flag = varargin{1};
end % if nargin > 3

% Field name
if static_flag == 1
    BM_field_name = 'BM_static';
    U_field_name = 'U_static';
else
    BM_field_name = 'BM';
    U_field_name = 'U';
end % if static_flag == 1
U_subfield_name = 'value_DOFt';

% Initialize variables
Sol.Beam.(BM_field_name).value_xt = zeros(Beam.Mesh.Node.num,Calc.Solver.num_t_beam);

% ---- NO average nodal values ----
if Calc.Opt.calc_mode_BM == 0
    
    for ix = 1:Beam.Mesh.Node.num-1

        aux1 = B30_Beam_ele_H(Beam.Mesh.Ele.a(ix),Beam.Prop.E_n(ix),Beam.Prop.I_n(ix));
        Sol.Beam.(BM_field_name).value_xt(ix,:) = aux1(1,:) * Sol.Beam.(U_field_name).(U_subfield_name)((ix*2-1):(ix*2-1)+3,:);

    end % for ix

    ix = Beam.Mesh.Node.num;
    aux1 = B30_Beam_ele_H(Beam.Mesh.Ele.a(ix-1),Beam.Prop.E_n(ix-1),Beam.Prop.I_n(ix-1));
    Sol.Beam.(BM_field_name).value_xt(ix,:) = aux1(2,:) * Sol.Beam.(U_field_name).(U_subfield_name)(((ix-1)*2-1):((ix-1)*2-1)+3,:);
    
% ---- AVERAGE nodal values ----
elseif Calc.Opt.calc_mode_BM == 1
    
    for ix = 1:Beam.Mesh.Node.num-1

        Sol.Beam.(BM_field_name).value_xt([1,2]+(ix-1),:) = Sol.Beam.(BM_field_name).value_xt([1,2]+(ix-1),:) + ...
            B30_Beam_ele_H(Beam.Mesh.Ele.a(ix),Beam.Prop.E_n(ix),Beam.Prop.I_n(ix)) * ... 
            Sol.Beam.(U_field_name).(U_subfield_name)((ix*2-1):(ix*2-1)+3,:);

    end % for ix

    % Average of nodes with multiple calculations
    Sol.Beam.(BM_field_name).value_xt(2:end-1,:) = Sol.Beam.(BM_field_name).value_xt(2:end-1,:)/2;
    
end % Calc.Opt.calc_mode_BM

% ---- End of function ----