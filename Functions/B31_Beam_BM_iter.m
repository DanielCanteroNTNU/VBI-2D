function [Sol] = B31_Beam_BM_iter(Calc,Beam,Sol)

% It is essentially the same function as B31_Beam_BM 
% but it should be used inside the iterative procedure and it is called
% only if the iteration criteria depends on BM values.

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
% ---- Output ----
% See A00 for description of output variables
% -------------------------------------------------------------------------

% Calculations performed only if iteration criteria depends on BM results
if Calc.Proc.Iter.criteria == 2

    [Sol] = B31_Beam_BM(Calc,Beam,Sol);

end % if Calc.Proc.Iter.criteria == 2

% ---- End of function ----