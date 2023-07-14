% Generation of Validation examples based on the analytical formulation

% *************************************************************************
% *** Script part of VBI-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% *************************************************************************

% This script includes implementation of the analytical formulation derived in:
% "Documents\Derivation of beam and 1-DOF moving vehicle coupled formulation.pdf"

clear; clc;

% ---- Validation ----
Validation.Save.on = 1;
Validation.Save.folder = 'Validation\';
%Validation.Save.name = 'Validation_Analytical_SprungVeh'; Opt.ramp_on = 0; Opt.damp_on = 0; Opt.acc_on = 0;
%Validation.Save.name = 'Validation_Analytical_RampOn'; Opt.ramp_on = 1; Opt.damp_on = 0; Opt.acc_on = 0;
%Validation.Save.name = 'Validation_Analytical_DampOn'; Opt.ramp_on = 1; Opt.damp_on = 1; Opt.acc_on = 0;
%Validation.Save.name = 'Validation_Analytical_AccOn'; Opt.ramp_on = 1; Opt.damp_on = 1; Opt.acc_on = 1;

% ---- Input definitions ----
L = 25;     % [m]
E = 3.5e10; % [N/m^2]
I = 1.3901; % [m^4] 
mB = 18358; % [kg/m]
mV = 5750;  % [kg]
kV = 1595e3;% [N/m]
cV = 40e4;  % [N*s/m]
v_0 = 25;   % [m/s]
acc = 0;    % [m/s^2]
g = -9.81;  % [m/s^2]
if Opt.damp_on == 0
    cV = 0;
end % if Opt.damp_on == 0

% Number of modes
num_modes = 100;
%num_modes = 10;

% Time step
dt = 0.001;
%dt = 0.0001/2;

% Ramp in profile
ramp_on = 0;
if Opt.ramp_on == 1
    ramp_on = 1;
    ramp_start = L/3;
    ramp_end = L/2;
    ramp_h = 0.01;
end % if Opt.ramp_on == 1

% Vehicle acceleration
if Opt.acc_on == 1
    acc = 10;   % [m/s^2]
end % if Opt.acc_on == 1

% ---------------------- Calculations -------------------------

% Mass Matrix
M_matrix = diag([mV,ones(1,num_modes)*mB]);

% Time
if acc == 0
    t_array = 0:dt:L/v_0;
else
    t_array = 0:dt:(sqrt(v_0^2+2*acc*L)-v_0)/acc;
end % if acc == 0
num_t = length(t_array);

% Instantaneous speed
v_t = v_0 + acc*t_array;
x_t = 0 + v_0.*t_array + 1/2*acc*t_array.^2;

% Beam circular frequency
i = 1:num_modes;
wB_i = (i*pi/L).^2*sqrt(E*I/mB);

% Profile
h = t_array*0;
if ramp_on == 1
    h = interp1([0,ramp_start,ramp_end,L],[0,0,ramp_h,ramp_h],x_t);
end % if ramp_on == 1

h_dot = [0,diff(h)]/dt;

% ---- Auxiliary variables ----
NewMark_delta = 0.5; 
NewMark_beta = 0.25;
NB_cte(1) = 1/(NewMark_beta*dt^2);
NB_cte(2) = NewMark_delta/(NewMark_beta*dt);
NB_cte(3) = 1/(NewMark_beta*dt);
NB_cte(4) = (1/(2*NewMark_beta)-1);
NB_cte(5) = (1-NewMark_delta/NewMark_beta);
NB_cte(6) = (1-NewMark_delta/(2*NewMark_beta))*dt;

% Initialize variables
Q = zeros(1+num_modes,num_t);
Q_dot = Q;
Q_ddot = Q;
F = zeros(1+num_modes,1);

% Time Step Loop
for t_ind = 1:num_t-1
    
    t = t_array(t_ind);

    % Stiffness matrix
    K_matrix = K_matrix_fun(kV,cV,mB,wB_i,num_modes,x_t(t_ind),v_t(t_ind),L);
    
    % Damping matrix
    C_matrix = C_matrix_fun(cV,num_modes,x_t(t_ind),L);
    
    % Force matrix
    F(1) = kV*h(t_ind+1) + cV*h_dot(t_ind+1);
    F(2:end) = (mV*g - kV*h(t_ind+1) - cV*h_dot(t_ind+1))*Phi_i_x_fun(1:num_modes,x_t(t_ind+1),L);
    
    % -- Newmark-Beta --
    % Effective Stiffness Matrix
    eff_K_matrix = K_matrix + NB_cte(1)*M_matrix + NB_cte(2)*C_matrix;
	% Newmark-beta scheme (As seen in [TCD.B014.p469])
    A = Q(:,t_ind)*NB_cte(1) + Q_dot(:,t_ind)*NB_cte(3) + Q_ddot(:,t_ind)*NB_cte(4);
    B = (NB_cte(2)*Q(:,t_ind) - NB_cte(5)*Q_dot(:,t_ind) - NB_cte(6)*Q_ddot(:,t_ind));
	Q(:,t_ind+1) = eff_K_matrix\(F + M_matrix*A + C_matrix*B);
    Q_dot(:,t_ind+1) = NB_cte(2)*Q(:,t_ind+1) - B;
    Q_ddot(:,t_ind+1) = Q(:,t_ind+1)*NB_cte(1) - A;
    
end % for t_ind = 1:num_t-1

% Beam mid-span displacement
y_beam_05 = Phi_i_x_fun(1:num_modes,L/2,L)*Q(2:end,:);

% ---- Saving ----
if myIsfield(Validation,{'Save',1,'on'},1)
    % Data to save
    Data.Time.values = t_array;
    Data.MidSpan.Disp.values = y_beam_05;
    % Saving
    save([Validation.Save.folder,Validation.Save.name],'Data');
    % Report
    disp('Results have been saved in:');
    disp([Validation.Save.folder,Validation.Save.name]);
end % if myIsfield(Bench,{'Save',1,'on'},1)

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
function [Phi_i_x] = Phi_i_x_fun(i,x,L)

Phi_i_x = sqrt(2/L)*sin(i*pi*x/L)*and(x>=0,x<=L);

end % function

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
function [Phi_prime_i_x] = Phi_prime_i_x_fun(i,x,L)

Phi_prime_i_x = sqrt(2/L)*(i*pi/L).*cos(i*pi*x/L)*and(x>=0,x<=L);

end % function

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
function [K_matrix] = K_matrix_fun(kV,cV,mB,wB_i,num_modes,x,v,L)

K_matrix_diag = diag([kV, mB*wB_i.^2 + kV*Phi_i_x_fun(1:num_modes,x,L).^2 + ...
    cV*Phi_i_x_fun(1:num_modes,x,L)*v.*Phi_prime_i_x_fun(1:num_modes,x,L)]);

K_matrix_offdiag = K_matrix_diag*0;
K_matrix_offdiag(1,2:end) = -kV*Phi_i_x_fun(1:num_modes,x,L);
for i = 1:num_modes
    for j = i+1:num_modes
        K_matrix_offdiag(i+1,j+1) = kV*Phi_i_x_fun(i,x,L)*Phi_i_x_fun(j,x,L) + ...
            cV*Phi_i_x_fun(i,x,L)*v*Phi_prime_i_x_fun(j,x,L);
    end % for j = i:num_modes
end % for i = 1:num_modes

K_matrix = K_matrix_diag + K_matrix_offdiag + K_matrix_offdiag';

K_matrix(1,2:end) = K_matrix(1,2:end) -cV*v*Phi_prime_i_x_fun(1:num_modes,x,L);

end % function

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
function [C_matrix] = C_matrix_fun(cV,num_modes,x,L)

C_matrix_diag = diag([cV, cV*Phi_i_x_fun(1:num_modes,x,L).^2]);

C_matrix_offdiag = C_matrix_diag*0;
C_matrix_offdiag(1,2:end) = -cV*Phi_i_x_fun(1:num_modes,x,L);
for i = 1:num_modes
    for j = i+1:num_modes
        C_matrix_offdiag(i+1,j+1) = cV*Phi_i_x_fun(i,x,L)*Phi_i_x_fun(j,x,L);
    end % for j = i:num_modes
end % for i = 1:num_modes

C_matrix = C_matrix_diag + C_matrix_offdiag + C_matrix_offdiag';

end % function

% ---- End of script ----