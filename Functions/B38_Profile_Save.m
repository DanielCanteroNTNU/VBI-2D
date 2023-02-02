function [] = B38_Profile_Save(Calc)

% If selected, saves the generated profile in a given name

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
% No output. Relevant file saved in the specified folder
% -------------------------------------------------------------------------

if Calc.Profile.Save.on == 1
    
    % Removing unnecessary fields
    Calc.Profile = myrmfield(Calc.Profile,'needed_x0');
    Calc.Profile = myrmfield(Calc.Profile,'needed_x_end');
    Calc.Profile = myrmfield(Calc.Profile,'needed_L');
    
    % Checking existence and creating folder to save results
    aux1 = dir(Calc.Profile.Save.path);
    if isempty(aux1)
        mkdir(Calc.Profile.Save.path);
    end % if isempty(aux1)

    % Checking if already exists
    if exist([Calc.Profile.Save.path,Calc.Profile.Save.file_name,'.mat'],'file')
        disp('File already exists with this name:');
        disp([Calc.Profile.Save.path,Calc.Profile.Save.file_name,'.mat']);
        user_in = input('Do you want to REPLACE it? Type then "Y"  ','s');
        if user_in == 'Y'
            k_save = 1; 
        else
            k_save = 0; 
        end % if user_in == 'Y'
    else
        k_save = 1;
    end % if exist([Calc.Profile.Save.path,Calc.Profile.Save.file_name,'.mat'],'file')

    % Saving results
    if k_save == 1
        Profile = Calc.Profile;
        save([Calc.Profile.Save.path,Calc.Profile.Save.file_name],'Profile');
        disp(['Profile saved as: ',Calc.Profile.Save.path,Calc.Profile.Save.file_name,'.mat']);
    else
        disp('Results NOT saved');
    end % if k_save == 1

end % if Calc.Profile.k_save == 1

% ---- End of script ----