function [Beam] = B01_ElementsAndCoordinates(Beam)

% Calculation of Node coordinates and associated Nodes (and DOF) to each element.
% Specifies the property values (E,I,rho and A) for each element
% Also generates additional and auxiliary variables needed in the FEM model
 
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

% Mesh definition 
Beam.Mesh.Ele.a = ones(1,Beam.Mesh.Ele.num)*Beam.Prop.Lb/Beam.Mesh.Ele.num;
Beam.Mesh.Ele.acum = [0 cumsum(Beam.Mesh.Ele.a)];

% -- Various definitions --
Beam.Mesh.Node.coord = Beam.Mesh.Ele.acum';
Beam.Mesh.Node.num_perEle = 2;
Beam.Mesh.DOF.num_perNode = 2;
Beam.Mesh.Ele.nodes = ones(length(Beam.Mesh.Ele.a),1)*(1:2);
Beam.Mesh.Ele.DOF = (1:2:(Beam.Mesh.Ele.num)*Beam.Mesh.Node.num_perEle)';
Beam.Mesh.Ele.DOF = [Beam.Mesh.Ele.DOF,Beam.Mesh.Ele.DOF+1,Beam.Mesh.Ele.DOF+2,Beam.Mesh.Ele.DOF+3];
Beam.Mesh.Node.num = size(Beam.Mesh.Node.coord,1);
Beam.Mesh.DOF.num = Beam.Mesh.Node.num*Beam.Mesh.Node.num_perEle;

% -- Element by element property definition Input processing --
% Beam Young's Modulus
if length(Beam.Prop.E) == 1
    Beam.Prop.E_n = ones(Beam.Mesh.Ele.num,1)*Beam.Prop.E;
end % if length(Beam.Prop.E) == 1
% Beam's section Second moment of Inertia product
if length(Beam.Prop.I) == 1
    Beam.Prop.I_n = ones(Beam.Mesh.Ele.num,1)*Beam.Prop.I;
end % if length(Beam.Prop.I) == 1
% Beam's mass per unit length
if length(Beam.Prop.rho) == 1
    Beam.Prop.rho_n = ones(Beam.Mesh.Ele.num,1)*Beam.Prop.rho;
end % if length(Beam.Prop.rho) == 1
% Beam's section Area
if length(Beam.Prop.A) == 1
    Beam.Prop.A_n = ones(Beam.Mesh.Ele.num,1)*Beam.Prop.A;
end % if length(Beam.Prop.A) == 1

% ---- End of function ----