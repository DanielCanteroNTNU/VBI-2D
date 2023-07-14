function [Calc,Sol] = B36_IterationCriteria(Calc,Sol)

% Checks the selected iteration criteria

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

% Depending on what iteration criteria
if Calc.Proc.Iter.criteria == 1

    % -- Deformation under wheels --
    % Difference with previous iteration
    for veh_num = 1:size(Sol.Veh,2)
        Calc.Proc.Iter.def_diff(Calc.Proc.Iter.num,veh_num) = max(max(abs(Sol.Veh(veh_num).Under.def - Sol.Veh(veh_num).Under.def_old)));
    end % for veh_num = 1:size(Sol.Veh,2)

    % Action depending on values
    if or(max(Calc.Proc.Iter.def_diff(Calc.Proc.Iter.num,:)) < Calc.Proc.Iter.tol,Calc.Proc.Iter.num >= Calc.Proc.Iter.max_num)
        Calc.Proc.Iter.continue = 0;
        if Calc.Proc.Iter.num == 1
            disp('No VBI!!!');
        elseif Calc.Proc.Iter.num >= Calc.Proc.Iter.max_num
            disp(['Maximum number of iterations reached !!! (',num2str(Calc.Proc.Iter.max_num),' iterations)']);
        end % if Calc.num_iter > Calc.max_iter
    else
        for veh_num = 1:size(Sol.Veh,2)
            Sol.Veh(veh_num).Under.def_old = Sol.Veh(veh_num).Under.def;
        end % for veh_num = 1:size(Sol.Veh,2)
    end % if Calc.Proc.Iter.def_diff < Calc.Proc.Iter.tol
    
elseif Calc.Proc.Iter.criteria == 2

    % -- Bending Moment of whole beam --
    % Difference with previous iteration
    Calc.Proc.Iter.BM_diff(Calc.Proc.Iter.num) = max(max(abs(Sol.Beam.BM.value_xt - Calc.Aux.old_BM_value_xt)));

    % Action depending on values
    if or(Calc.Proc.Iter.BM_diff(Calc.Proc.Iter.num) < Calc.Proc.Iter.tol,Calc.Proc.Iter.num >= Calc.Proc.Iter.max_num)
        Calc.Proc.Iter.continue = 0;
        if Calc.Proc.Iter.num == 1
            disp('No VBI!!!');
        elseif Calc.Proc.Iter.num >= Calc.Proc.Iter.max_num
            disp(['Maximum number of iterations reached !!! (',num2str(Calc.Proc.Iter.max_num),' iterations)']);
        end % if Calc.num_iter > Calc.max_iter
    else
        Calc.Aux.old_BM_value_xt = Sol.Beam.BM.value_xt;
    end % if Calc.Proc.Iter.BM_diff < Calc.Proc.Iter.tol
    
end % if Calc.Proc.Iter.criteria == 1

% ---- End of scritp ----