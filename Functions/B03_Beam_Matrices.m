function [Beam] = B03_Beam_Matrices(Beam)

% Generates the FEM system matrices for the beam model

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

% Generate nodal connectivity for each elemnt
nodalconnec = [(1:Beam.Mesh.Ele.num);(1:Beam.Mesh.Ele.num)+1]';

% Initialize matrices
Beam.SysM.K = zeros(Beam.Mesh.DOF.num); 
Beam.SysM.M = Beam.SysM.K;

% Elemental matrices (In-line functions)
B04_Beam_ele_M = @(r,A,L) r*A*L/420*[[156,22*L,54,-13*L];[22*L,4*L^2,13*L,-3*L^2];...
    [54,13*L,156,-22*L];[-13*L,-3*L^2,-22*L,4*L^2]];
B05_Beam_ele_K = @(EI,L) EI/L^3*[[12,6*L,-12,6*L];[6*L,4*L^2,-6*L,2*L^2];...
    [-12,-6*L,12,-6*L];[6*L,2*L^2,-6*L,4*L^2]];

% ---- Mass Stiffness and Matrices ----
for iel = 1:Beam.Mesh.Ele.num

    index = [(nodalconnec(iel,1)-1)*2+(1:2),(nodalconnec(iel,2)-1)*2+(1:2)];
    
    % Element matrices
    Me = B04_Beam_ele_M(Beam.Prop.rho_n(iel),Beam.Prop.A_n(iel),Beam.Mesh.Ele.a(iel));
    Ke = B05_Beam_ele_K(Beam.Prop.E_n(iel)*Beam.Prop.I_n(iel),Beam.Mesh.Ele.a(iel));
    
    % Assembly of matrices
    Beam.SysM.K(index,index) = Beam.SysM.K(index,index) + Ke;
    Beam.SysM.M(index,index) = Beam.SysM.M(index,index) + Me;

end %for iel = 1:Beam.Mesh.Ele.num

% ---- Application of boundary conditions ----
% Diagonal elements equal Beam.BC.DOF_fixed_value, and columns and rows equal zero for bc DOF

% Fixed DOF
Beam.SysM.K(Beam.BC.DOF_fixed,:) = 0; Beam.SysM.K(:,Beam.BC.DOF_fixed) = 0;
Beam.SysM.M(Beam.BC.DOF_fixed,:) = 0; Beam.SysM.M(:,Beam.BC.DOF_fixed) = 0;
for i = 1:Beam.BC.num_DOF_fixed
    Beam.SysM.K(Beam.BC.DOF_fixed(i),Beam.BC.DOF_fixed(i)) = Beam.BC.DOF_fixed_value;
    Beam.SysM.M(Beam.BC.DOF_fixed(i),Beam.BC.DOF_fixed(i)) = Beam.BC.DOF_fixed_value;
end % for i = 1:Beam.BC.num_DOF_fixed

% Support DOF with values
for i = 1:Beam.BC.num_DOF_with_values
    Beam.SysM.K(Beam.BC.DOF_with_values(i),Beam.BC.DOF_with_values(i)) = ...
        Beam.SysM.K(Beam.BC.DOF_with_values(i),Beam.BC.DOF_with_values(i)) + ...
        Beam.BC.DOF_stiff_values(i);
end % for i = 1:Beam.BC.num_DOF_values

% ---- Making output sparse ----
Beam.SysM.K = sparse(Beam.SysM.K); 
Beam.SysM.M = sparse(Beam.SysM.M);

% ---- Beam element Shape function ----
Beam.Mesh.Ele.shape_fun = @(x,a) [(a+2*x).*(a-x).^2./a.^3; x.*(a-x).^2./a.^2; x.^2.*(3*a-2*x)./a.^3; -x.^2.*(a-x)./a.^2];
Beam.Mesh.Ele.shape_fun_p = @(x,a) [-(6*x.*(a - x))./a.^3; 1 - (x.*(4*a - 3*x))./a.^2; (6*x.*(a - x))./a.^3; -(x.*(2*a - 3*x))./a.^2];
Beam.Mesh.Ele.shape_fun_pp = @(x,a) [(12*x)./a.^3 - 6./a.^2; (6*x)./a.^2 - 4./a; 6./a.^2 - (12*x)./a.^3; (6*x)./a.^2 - 2./a];

% ---- End of function ----