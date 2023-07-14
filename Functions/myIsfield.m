function [answer,varargout] = myIsfield(InputStruct,input_cell,varargin)

% Checks if the field exists. It does so by checking various levels of 
% substructure succesively

% *************************************************************************
% *** Script part of VBI-2D tool for Matlab environment.                ***
% *** Licensed under the GNU General Public License v3.0                ***
% *** Author: Daniel Cantero (daniel.cantero@ntnu.no)                   ***
% *** For help, modifications, and collaboration contact the author.    ***
% *************************************************************************

% ---- Input ----
% InputStruct = Original structure variable to check
% input_cell = Cell variable containing two thing for each substructure level
%   First a string with the name of the level
%   Then an integer indicating the element corresponding to that level
%   Repeat this pair of values as many subfields to check
%   The last subfield can be defined only with the string, and assumes then
%   that the element number is 1. This can then be used to check if particular variables exist
%   Example:
%       input_cell = {'NameField1',2,'NameField2',1,...]
%   or
%       input_cell = {'NameField1',2,'NameField2',1,...,'NameOfVariable']
% ---- Optional input ----
% equal_to = Additionally checks if the value of the sought variable is equal to
%   a particular value. The logic answer is updated accordingly.
%   Example
%   Test.test.value = 1;
%   answer = myIsfield(Test,{'Test',1,'value'},1);
%   answer = true
% ---- Output ---
% answer = true/false
% -- Optional Output --
% OutputStruct = Is the Substructure or variable at the end of the sequence
%   of subfields and field elements
% ---------------

% Input processing
fields_cell = {input_cell{1:2:end}};
fields_element = [input_cell{2:2:end}];
if length(fields_element) < length(fields_cell)
    fields_element = [fields_element, 1];
end % if length(fields_element) < length(fields_cell)
num_levels = size(fields_cell,2);

% Auxiliary variables
k_continue = 1;
level = 0;

% Substruture loop
while and(k_continue == 1,level<num_levels)
    
    level = level + 1;
    
    if isfield(InputStruct,fields_cell{level})
        
        size_subfield = size(InputStruct.(fields_cell{level}),2);
        
        if level == num_levels
            
            InputStruct = InputStruct.(fields_cell{level});
            answer = true;
            
        else
        
            if and(size_subfield > 0, size_subfield >= fields_element(level))

                InputStruct = InputStruct.(fields_cell{level})(fields_element(level));
                answer = true;

            else

                k_continue = 0;
                answer = false;

            end % if size(InputStruct.(fields_cell{level}),2) > 0
        end % if level == num_level
        
    else
        
        k_continue = 0;
        answer = false;
        
    end % if isfield(InputStruct,fields_cell{level})
    
end % while and(k_continue == 1,level<num_levels)

% Optional input
if nargin == 3
    if answer
        answer = (InputStruct==varargin{1});
    end % if answer
end % if nargin == 3

% Additional output generation
if nargout == 2
    if answer
        varargout{1} = InputStruct;
    else
        varargout{1} = [];
    end % if answer
end % if nargout == 2

% ---- End of script ----