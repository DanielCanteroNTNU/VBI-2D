function [Sol] = B52_Beam_Disp_and_Rot(Sol)

% Extracts the Displacements and Rotations from the FEM nodal responses to
% separate sub-structures on the Sol variable

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

LEs_list = {'Disp','Rot'};
first_DOF = [1,2];
t_deriv_nums = [0,1,2];
t_deriv_list = {'U','V','A'};
t_deriv_add_names = {'','_dot','_ddot'};
static_option_txt = {'','_static'};

for LE_num = 1:length(LEs_list)
    for t_deriv_num = t_deriv_nums
        if t_deriv_num == 0
            static_options = [1,2];
        else
            static_options = 1;
        end % if t_deriv_num == 0
        for static_option = static_options
            Sol.Beam.([LEs_list{LE_num},t_deriv_add_names{t_deriv_num+1},static_option_txt{static_option}]).value_xt = ...
                Sol.Beam.([t_deriv_list{t_deriv_num+1},static_option_txt{static_option}]).value_DOFt(first_DOF(LE_num):2:end,:);
        end % for static_option = static_options
    end % for t_deriv_num = t_deriv_nums
end % for LE_num = 1:length(LEs_list)

% ---- End of function ----