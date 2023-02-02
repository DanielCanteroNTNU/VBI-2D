function [Fextnew] = B14_EqVertNodalForce(Calc,Veh,Beam,Sol)

% Distributee the forces to the appropriate DOFs
% For each time step and depending on the location of the force, the actual
% forces must be distributed to the degrees of freedom, this can be 
% accomplished through the shape functions.
% NOTE: Only Vertical forces calculated

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
% Fextnew = Equivalent nodal forces vector
% -------------------------------------------------------------------------

% Calculation options (Only for Point loads)
k_fun = 0;      % Calculate NOT using subfunction (A bit faster)
%k_fun = 1;      % Calculate using subfunction

% Linear equivalent load option
k_factors = [1,0]; 
if Calc.Opt.linear_eqv_loads == 1
    k_factors = [0,1];
end % if Calc.Opt.linear_eqv_loads == 1

% Initialize variable
Fextnew = zeros(Beam.Mesh.DOF.num,Calc.Solver.num_t_beam);

% Vehicles loop
for veh_num = 1:Veh(1).Event.num_veh

    % Wheel loop
    for wheel_num = 1:Veh(veh_num).Prop.num_wheels

        Fextnew1 = Fextnew*0;

        % Time loop
        for t = 1:Calc.Solver.num_t_beam

            elex = Veh(veh_num).Pos.elexj(wheel_num,Calc.Solver.t0_ind_beam-1+t);
            x = Veh(veh_num).Pos.xj(wheel_num,Calc.Solver.t0_ind_beam-1+t);
            if elex > 0

                % DOFs for the element    
                ele_DOF = Beam.Mesh.Ele.DOF(elex,:);

                % Multiplication of nodal displacements by corresponding shape function value
                if k_fun == 1
                    % --- Tidy alternative --- (A bit slower)
                    Fextnew1(ele_DOF,t) = Sol.Veh(veh_num).Under.onBeamF(wheel_num,t)*shapefunBeam(Beam.Mesh.Ele.a(elex),x,k_factors);
                elseif k_fun == 0
                    % --- Alternative without function --- (A bit faster)
                    ai = Beam.Mesh.Ele.a(elex); 
                    Fextnew1(ele_DOF,t) = Sol.Veh(veh_num).Under.onBeamF(wheel_num,t)*( ...
                        k_factors(1)*[ (ai+2*x)*(ai-x)^2/ai^3; x*(ai-x)^2/ai^2; x^2*(3*ai-2*x)/ai^3; -x^2*(ai-x)/ai^2] + ...
                        k_factors(2)*[ 1-x/ai; 0; x/ai; 0]);
                end % if k_fun == 1        
            end % if elex > 0
        end % for t = 1:Calc.Solver.num_t_beam

        Fextnew = Fextnew + Fextnew1;

    end % for wheel_num = 1:Veh(veh_num).Prop.num_wheels
end % for veh_num = 1:Veh(1).Event.num_veh

% Application of boundary conditoins to force vector
Fextnew(Beam.BC.DOF_fixed,:) = 0;  % Vertical force = 0

% Function for Tidy alternative
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
function vect = shapefunBeam(a,x,k_factors)
    % a = Element size in X direction
	% x = Position of force in X direction (local coordinates)
    % k_factors = [
    
    vect = k_factors(1)*[ (a+2*x)*(a-x)^2/a^3; x*(a-x)^2/a^2; x^2*(3*a-2*x)/a^3; -x^2*(a-x)/a^2] + ...
           k_factors(2)*[ 1-x/ai; 0; x/ai; 0];
       
%end % function shapefunBeam
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

%end % function B14
% ---- End of function ----