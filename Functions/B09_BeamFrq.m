function [Beam] = B09_BeamFrq(Calc,Beam)

% Calculates the beam frequencies given its system matrices

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

% Only natural frequencies calculation
if and(Calc.Opt.beam_frq == 1,Calc.Opt.beam_modes == 0)
    
    lambda = eig(full(Beam.SysM.K),full(Beam.SysM.M));
    Beam.Modal.w = sqrt(lambda);
    Beam.Modal.f = Beam.Modal.w/(2*pi);

    % Removing values associated to BC
    if Beam.Modal.f(1) < 0.1591549
        for k = 1:Beam.BC.num_DOF_fixed
            [~,aux1] = min(abs(Beam.Modal.f-0.1591549));
            Beam.Modal.f(aux1) = [];
            Beam.Modal.w(aux1) = [];
        end % for k = 1:Beam.BC.num_DOF_fixed
    else
        Beam.Modal.w = Beam.Modal.w(Beam.BC.num_DOF_fixed+1:end);
        Beam.Modal.f = Beam.Modal.f(Beam.BC.num_DOF_fixed+1:end);
    end % if Beam.Modal.f(1) < 0.1591549
    
% Natural frequencies and Modes of vibration calculation
elseif and(Calc.Opt.beam_frq == 1,Calc.Opt.beam_modes == 1)
    
    [V,lambda] = eig(full(Beam.SysM.K),full(Beam.SysM.M));
    [lambda,k] = sort(diag(lambda));
    V = V(:,k); 

    % Normaliztion of eigenvectors
    Factor = diag(V'*Beam.SysM.M*V); 
    Beam.Modal.modes = V/(sqrt(diag(Factor)));
    
    % EigenValues to Natural frequencies
    Beam.Modal.w = sqrt(lambda);
    Beam.Modal.f = Beam.Modal.w/(2*pi);
    
    % Removing values associated to BC
    Beam.Modal.w = Beam.Modal.w(Beam.BC.num_DOF_fixed+1:end);
    Beam.Modal.f = Beam.Modal.f(Beam.BC.num_DOF_fixed+1:end);
    Beam.Modal.modes(:,1:Beam.BC.num_DOF_fixed) = [];
    
end % if and(Calc.beam_frq == 1,Calc.beam_frq == 0)

% ------------ Plotting ------------

% -- Plotting of calculated Natural frequencies --
if myIsfield(Calc.Plot,{'P1_Beam_frq'},1)
    figure; plot((1:length(Beam.Modal.f)),Beam.Modal.f,'.'); axis tight;
    %figure; semilogy((1:length(Beam.Modal.f)),Beam.Modal.f,'.'); axis tight;
        xlabel('Mode number'); ylabel('Frequency (Hz)');
        title(['Beam Only (1st frq: ',num2str(round(Beam.Modal.f(1),2)),' Hz;',...
        blanks(1),'Last frq: ',num2str(round(Beam.Modal.f(end),2)),' Hz)']);
    drawnow;
end % if myIsfield(Calc.Plot,{'P1_Beam_frq'},1)

% -- Plotting Mode shapes --
if myIsfield(Calc.Plot,{'P2_Beam_modes'},1)
    if Calc.Plot.P2_Beam_modes > 0
        aux1 = ceil(Calc.Plot.P2_Beam_modes/2);
        figure; 
        for k = 1:Calc.Plot.P2_Beam_modes
            aux2 = max(abs(Beam.Modal.modes(1:2:end,k)));
            subplot(aux1,2,k)
            if exist('Beam_redux','var')
                Xdata = Beam_redux.acum;
            else
                Xdata = Beam.Mesh.Ele.acum;
            end % if exist('Beam_redux','var');
            plot(Xdata,Beam.Modal.modes(1:2:end,k)/aux2);
            hold on; plot(Xdata,-Beam.Modal.modes(1:2:end,k)/aux2,'r');
            plot(Beam.BC.loc,zeros(size(Beam.BC.loc)),'r.','MarkerSize',10);
            xlim([0,Beam.Prop.Lb]);
            title(['Mode ',num2str(k),' (',num2str(round(Beam.Modal.f(k),3)),' Hz)']);
        end % for k = 1:Calc.Plot.P2_Beam_modes
        drawnow;
    end % if Calc.Plot.P2_Beam_modes > 0
end % if myIsfield(Calc.Plot,{'P2_Beam_modes'},1)

% ---- End of function ----