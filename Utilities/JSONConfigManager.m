%%%%%%%%%%%%%%%%%%%%%%%%%
%% P4UCC Project
%% S. Ortega, A. Trujillo, J.M. Santana, C. Ramírez, J.P. Suárez
%% Universidad de Las Palmas de Gran Canaria, 2020
%%%%%%%%%%%%%%%%%%%%%%%%%

classdef JSONConfigManager
    properties(Constant)
        DEFAULT_CONFIG = 'config.json';
        
        % These values are not those of the final classification
        % but rather printing values in order to use the DrawStuff library
        % for debug purposes. The final value is assigned at saving time
        % by using the unlabel function.
        
        GROUND_CLASS_VALUE = 2; % yellow
        POLE_CLASS_VALUE = 16; % blue
        CAR_CLASS_VALUE = 17; % magenta
        TREE_CLASS_VALUE = 4; % dark green
        BUILDING_CLASS_VALUE = 6; % red
        TEMP_VALUE_1 = 30; %cyan
    end
    
    methods(Static)
    
        function generateConfig(inputArg2)
            JSONConfigManager.getConfig(inputArg2,1);
        end

        function generateDefaultConfig()
            JSONConfigManager.getConfig(JSONConfigManager.DEFAULT_CONFIG,1); 
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Asking for folders             %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function [dirInput, inputFiles, dirOutput] = getGeneralInformation()
            Config = JSONConfigManager.getConfig();    
            dirInput = Config.folders.input_folder;
            dirOutput = Config.folders.output_folder;
            inputFiles = Config.folders.input_files; 
        end
        
        %%%%%%% DO NOT USE OR EVEN TOUCH THIS FUNCTION AT ALL !
        function config = getConfig(inputArg2,op)
            if ~exist('inputArg2','var') || ~exist('op','var')
                op = 0;
            end

            persistent struct; 
            if op == 1
                struct = json2struct(inputArg2);
            end

            config = struct;
        end
    
    end
end



function val = json2struct(filename)
    fid = fopen(filename); 
    raw = fread(fid,inf); 
    str = char(raw'); 
    fclose(fid); 
    val = jsondecode(str);
end