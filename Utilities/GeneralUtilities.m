%%%%%%%%%%%%%%%%%%%%%%%%%
%% P4UCC Project
%% S. Ortega, A. Trujillo, J.M. Santana, C. Ramírez, J.P. Suárez
%% Universidad de Las Palmas de Gran Canaria, 2020
%%%%%%%%%%%%%%%%%%%%%%%%%

classdef GeneralUtilities
   methods(Static)
   
       function [maxX,maxY,minX,minY] = bbox2D(points)
            maxX = max(points(:,1));
            maxY = max(points(:,2));
            minX = min(points(:,1));
            minY = min(points(:,2));
       end
       
       function matrix = validatingPrintingAndSaving ...
               (data,validator,output,filename)

            %% Validation against the GT for this instance. Comment for no validation.
            summit = ismember(data(:,4),[6,16]);
            data(summit,4) = cleansing(data(summit,:));
            data(data(:,4) == 0,4) = 2;
            fprintf('-------------------------- \n Results for file %s :\n',filename);
            matrix = validator.validate(data(:,4));

            %% Graphical representation of the results of this instance. Comment for no plotting.
            DrawStuff.showData(data,false);

            %% Saving results of P4UCC for this instance. 
            saveResult(output,filename,unlabel(data(:,4)));
       end
       
   end
end

%% Private functions

function data2 = unlabel(data)
    %% This converts from printing values to SHREC values
    data2 = zeros(size(data));
    data2(data == JSONConfigManager.GROUND_CLASS_VALUE) = 3;
    data2(data == JSONConfigManager.POLE_CLASS_VALUE) = 4;
    data2(data == JSONConfigManager.CAR_CLASS_VALUE) = 2;
    data2(data == JSONConfigManager.TREE_CLASS_VALUE) = 5;
    data2(data == JSONConfigManager.BUILDING_CLASS_VALUE) = 1;
end

function saveResult(output,filename,data)
    s = sprintf('%s%s.txt',output,filename);
    dlmwrite(s,data,'newline','pc');
end

function classify = cleansing(p)
   
    pc = pointCloud(p(:,1:3));
    idx = pcsegdist(pc,0.5);
    
    count = accumarray(idx,ones(size(idx)),[],@sum);
    summit = accumarray(idx,p(:,4) == 16,[],@sum);
    isSummit = (summit ./ count) > 0.5; isSummit = isSummit(idx);
    
    classify = p(:,4);
    classify(~isSummit & classify == 16) = 6;

end