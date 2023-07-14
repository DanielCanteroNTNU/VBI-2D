function [InStruct] = myrmfield(InStruct,fields2remove)

% My variation of command rmfield()
% The difference is that if any of the fields to remove does not exist the
% command does not give an error.

% *************************************************************************
% *** Script part of VBI-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% *************************************************************************

% -------------------------------------------------------------------------

% If single input for field2remov
if ~iscell(fields2remove)
    fields2remove = {fields2remove};
end % if ~iscell(fields2remove)

for k = 1:size(fields2remove,2)
    
    if isfield(InStruct,fields2remove{k})
        
        InStruct = rmfield(InStruct,fields2remove{k});
        
    end % if isfield(InStruct,fields2remove{k})
    
end % for k = 1:size(fields2remove,2)

% ---- End of script ----