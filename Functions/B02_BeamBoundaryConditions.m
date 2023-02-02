function [Beam] = B02_BeamBoundaryConditions(Beam)

% Definition of DOF with boundary conditions for different configurations

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

% Number of supports
Beam.BC.supp_num = length(Beam.BC.loc);

% Supports location index
[~,Beam.BC.loc_ind] = min(abs(ones(Beam.BC.supp_num,1)*Beam.Mesh.Ele.acum - ...
    Beam.BC.loc'*ones(1,Beam.Mesh.Node.num)),[],2);
Beam.BC.loc_ind = Beam.BC.loc_ind';

% Fixed vertical displacement DOF
Beam.BC.DOF_fixed = Beam.BC.loc_ind(Beam.BC.vert_stiff==Inf)*2-1;

% Fixed rotational DOF
Beam.BC.DOF_fixed = [Beam.BC.DOF_fixed,Beam.BC.loc_ind(Beam.BC.rot_stiff==Inf)*2];

% Sorting fixed DOF
Beam.BC.DOF_fixed = sort(Beam.BC.DOF_fixed);

% Vertical displacement DOF with stiffness values
inds = and(isfinite(Beam.BC.vert_stiff),Beam.BC.vert_stiff~=0);
Beam.BC.DOF_with_values = Beam.BC.loc_ind(inds)*2-1;
Beam.BC.DOF_stiff_values = Beam.BC.vert_stiff(inds);

% Fixed rotational DOF
inds = and(isfinite(Beam.BC.rot_stiff),Beam.BC.rot_stiff~=0);
Beam.BC.DOF_with_values = [Beam.BC.DOF_with_values, Beam.BC.loc_ind(inds)*2];
Beam.BC.DOF_stiff_values = [Beam.BC.DOF_stiff_values, Beam.BC.rot_stiff(inds)];

% Sorting fixed DOF
[Beam.BC.DOF_with_values,aux2] = sort(Beam.BC.DOF_with_values);
Beam.BC.DOF_stiff_values = Beam.BC.DOF_stiff_values(aux2);

% Auxiliary variables
Beam.BC.num_DOF_fixed = length(Beam.BC.DOF_fixed);
Beam.BC.num_DOF_with_values = length(Beam.BC.DOF_with_values);
Beam.Modal.num_rigid_modes = max([0,2 - Beam.BC.num_DOF_fixed]);

% Value to use in the diagonal element when the DOF is fixed
Beam.BC.DOF_fixed_value = 1;

% ---- End of script ----