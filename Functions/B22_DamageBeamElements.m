function [Beam] = B22_DamageBeamElements(Beam)

% Changes the values of element properties (or BC) according to the defined damage

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

% -- 1 element Damage --
if Beam.Damage.type == 1
    mid_ele_coords = Beam.Mesh.Ele.acum(1:end-1)+Beam.Mesh.Ele.a/2;
    [~,aux1] = min(abs(mid_ele_coords/Beam.Prop.Lb-Beam.Damage.loc_per/100));
    if isfield(Beam.Damage,'E')
        Beam.Prop.E_n(aux1) = Beam.Prop.E_n(aux1)*(1-Beam.Damage.E.per/100);
    end % if isfield(Beam.Damage,'E')
    if isfield(Beam.Damage,'I')
        Beam.Prop.I_n(aux1) = Beam.Prop.I_n(aux1)*(1-Beam.Damage.I.per/100);
    end % if isfield(Beam.Damage,'I')
    if isfield(Beam.Damage,'rho')
        Beam.Prop.rho_n(aux1) = Beam.Prop.rho_n(aux1)*(1-Beam.Damage.rho.per/100);
    end % if isfield(Beam.Damage,'rho')

% -- Global Damage --
elseif Beam.Damage.type == 2    
    if isfield(Beam.Damage,'E')
        Beam.Prop.E_n = Beam.Prop.E_n*(1-Beam.Damage.E.per/100);
    end % if isfield(Beam.Damage,'E')
    if isfield(Beam.Damage,'I')
        Beam.Prop.I_n = Beam.Prop.I_n*(1-Beam.Damage.I.per/100);
    end % if isfield(Beam.Damage,'I')
    if isfield(Beam.Damage,'rho')
        Beam.Prop.rho_n = Beam.Prop.rho_n*(1-Beam.Damage.rho.per/100);
    end % if isfield(Beam.Damage,'rho')
    
% -- BC change --
elseif Beam.Damage.type == 3
    
    % Default values
    if ~myIsfield(Beam.Damage,{'BC',1,'loc'})
        Beam.Damage.BC.loc = Beam.BC.loc;
    end % if ~myIsfield(Beam.Damage,{'BC',1,'loc'})
    if ~myIsfield(Beam.Damage,{'BC',1,'vert_stiff'})
        Beam.Damage.BC.vert_stiff = Beam.Damage.BC.loc*0;
    end % if ~myIsfield(Beam.Damage,{'BC',1,'vert_stiff'})
    if ~myIsfield(Beam.Damage,{'BC',1,'rot_stiff'})
        Beam.Damage.BC.rot_stiff = Beam.Damage.BC.loc*0;
    end % if ~myIsfield(Beam.Damage,{'BC',1,'rot_stiff'})

    % Checking input lengths
    if ~all(length(Beam.Damage.BC.loc)==[length(Beam.Damage.BC.vert_stiff),length(Beam.Damage.BC.rot_stiff)])
        error('Lengths of Beam.Damage.BC inputs are not the same')
    end % if ~all

    % New Beam structure
    Beam2 = rmfield(Beam,'BC');
    Beam2.BC = Beam.Damage.BC;
    
    % New BC
    [Beam2] = B02_BeamBoundaryConditions(Beam2);
    Beam.Damage.BC = Beam2.BC;

    % Merging Beam.Damage.BC to Beam.BC
    Beam.BC.DOF_fixed = sort([Beam.BC.DOF_fixed,Beam.Damage.BC.DOF_fixed]);
    Beam.BC.DOF_with_values = [Beam.BC.DOF_with_values,Beam.Damage.BC.DOF_with_values];
    Beam.BC.DOF_stiff_values = [Beam.BC.DOF_stiff_values,Beam.Damage.BC.DOF_stiff_values];
    [Beam.BC.DOF_with_values,sort_inds] = sort(Beam.BC.DOF_with_values);
    Beam.BC.DOF_stiff_values = Beam.BC.DOF_stiff_values(sort_inds);
    Beam.BC.DOF_with_values(Beam.BC.DOF_stiff_values==0) = [];
    Beam.BC.DOF_stiff_values(Beam.BC.DOF_stiff_values==0) = [];
    [Beam.BC.DOF_with_values,~,ic] = unique(Beam.BC.DOF_with_values);
    aux1 = zeros(1,max(ic));
    for i = 1:max(ic)
        aux1(i) = sum(Beam.BC.DOF_stiff_values(ic==i));
    end % for i
    Beam.BC.DOF_stiff_values = aux1;
    Beam.BC.num_DOF_fixed = length(Beam.BC.DOF_fixed);
    Beam.BC.num_DOF_with_values = length(Beam.BC.DOF_with_values);
    
    % Checking that not negative stiffness values are used
    if any(Beam.BC.DOF_stiff_values<0)
        error('Damage type 3 introduces negative stiffness values')
    end % if any(Beam.BC.DOF_stiff_values<0)
    
% -- Function in space --
elseif Beam.Damage.type == 4    

    % Default values
    if ~myIsfield(Beam.Damage,{'Plot',1,'on'})
        Beam.Damage.Plot.on = 0;
    end % if ~myIsfield(Beam.Damage,{'Plot',1,'on'})

    x = Beam.Mesh.Ele.acum(1:end-1) + Beam.Mesh.Ele.a/2;
    
    % Spatial variation of E
    if isfield(Beam.Damage,'E')
        x_factor = Beam.Damage.E.fun(x,Beam.Damage.E.Inputs);
        Beam.Prop.E_n = Beam.Prop.E*x_factor;
    end % if isfield(Beam.Damage,'E')

    % Graphical check
    if Beam.Damage.Plot.on == 1
        figure; 
            plot(x,x_factor,'.-'); axis tight;
            ylim(ylim.*[0,1]);
            xlabel('Distance'); ylabel('Factor for damage'); 
            title('Spatial variation for for E');
    end % if Beam.Damage.Plot.on == 1

end % if Beam.Damage.ype == 1


% ---- End of script ----
