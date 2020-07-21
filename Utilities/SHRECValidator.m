%%%%%%%%%%%%%%%%%%%%%%%%%
%% P4UCC Project
%% S. Ortega, A. Trujillo, J.M. Santana, C. Ramírez, J.P. Suárez
%% Universidad de Las Palmas de Gran Canaria, 2020
%%%%%%%%%%%%%%%%%%%%%%%%%

classdef SHRECValidator
   properties
       realTags
   end
   methods
       function obj = SHRECValidator(tags)
          obj.realTags = labels(tags);
       end
       
       function [matrix,interclass,gClass] = validateWithTranslation(obj,tags)
           [matrix,interclass,gClass] = obj.validate(labels(tags));
       end
       
       function [matrix,interclass,gClass] = validate(obj,tags)
           gIdx = obj.realTags == 2;
           bIdx = obj.realTags == 6;
           pIdx = obj.realTags == 16;
           cIdx = obj.realTags == 17;
           vIdx = obj.realTags == 4;
           
           gClass = ["ground","building","pole","car","veggie","other"];
           matrix = zeros(5,6);
           %%%%% GROUND COMPARE
           cl = tags(gIdx);
           matrix(1,:) = [sum(cl == 2), sum(cl == 6), sum(cl == 16), ...
               sum(cl == 17), sum(cl == 4), sum(~ismember(cl,[2,6,16,17,4])) ];
           %%%%% BUILDING COMPARE
           cl = tags(bIdx);
           matrix(2,:) = [sum(cl == 2), sum(cl == 6), sum(cl == 16), ...
               sum(cl == 17), sum(cl == 4), sum(~ismember(cl,[2,6,16,17,4])) ];
           %%%%% POLE COMPARE
           cl = tags(pIdx);
           matrix(3,:) = [sum(cl == 2), sum(cl == 6), sum(cl == 16), ...
               sum(cl == 17), sum(cl == 4), sum(~ismember(cl,[2,6,16,17,4])) ];
           %%%%% CAR COMPARE
           cl = tags(cIdx);
           matrix(4,:) = [sum(cl == 2), sum(cl == 6), sum(cl == 16), ...
               sum(cl == 17), sum(cl == 4), sum(~ismember(cl,[2,6,16,17,4])) ];
           %%%%% VEGGIE COMPARE
           cl = tags(vIdx);
           matrix(5,:) = [sum(cl == 2), sum(cl == 6), sum(cl == 16), ...
               sum(cl == 17), sum(cl == 4), sum(~ismember(cl,[2,6,16,17,4])) ];
           %%% gMeasures:
           interclass = SHRECValidator.intraClassFromMatrix(matrix);
           
       end
   end
   
   methods(Static)
       function interclass = intraClassFromMatrix(matrix)
          
           gReal = matrix(1,1); gFN = sum(matrix(1,2:end)); gFP = sum(matrix(2:end,1));
           bReal = matrix(2,2); bFN = sum(matrix(2,[1,3:end])); bFP = sum(matrix([1,3:end],2));
           pReal = matrix(3,3); pFN = sum(matrix(3,[1:2,4:end])); pFP = sum(matrix([1:2,4:end],3));
           cReal = matrix(4,4); cFN = sum(matrix(4,[1:3,5:end])); cFP = sum(matrix([1:3,5:end],4));
           vReal = matrix(5,5); vFN = sum(matrix(5,[1:4,6:end])); vFP = sum(matrix([1:4,6:end],5));
           
           interclass = zeros(5,7);
           interclass(1,1:5) = [gReal,gFN,gFP,gReal/(gReal+gFN),gReal/(gReal+gFP)];
           interclass(2,1:5) = [bReal,bFN,bFP,bReal/(bReal+bFN),bReal/(bReal+bFP)];
           interclass(3,1:5) = [pReal,pFN,pFP,pReal/(pReal+pFN),pReal/(pReal+pFP)];
           interclass(4,1:5) = [cReal,cFN,cFP,cReal/(cReal+cFN),cReal/(cReal+cFP)];
           interclass(5,1:5) = [vReal,vFN,vFP,vReal/(vReal+vFN),vReal/(vReal+vFP)];
           interclass(1,6:7) = [2/((1/interclass(1,4)) + (1/interclass(1,5))),...
               gReal/(gReal+gFN+gFP)];
           interclass(2,6:7) = [2/((1/interclass(2,4)) + (1/interclass(2,5))),...
               bReal/(bReal+bFN+bFP)];
           interclass(3,6:7) = [2/((1/interclass(3,4)) + (1/interclass(3,5))),...
               pReal/(pReal+pFN+pFP)];
           interclass(4,6:7) = [2/((1/interclass(4,4)) + (1/interclass(4,5))),...
               cReal/(cReal+cFN+cFP)];
           interclass(5,6:7) = [2/((1/interclass(5,4)) + (1/interclass(5,5))),...
               vReal/(vReal+vFN+vFP)];
           
           %%
           fprintf('Matrix \n');
           disp(matrix);
           fprintf('Overall accuracy : %.4f \n',....
               (gReal+bReal+pReal+cReal+vReal)/sum(matrix(:)));
           fprintf('Class ground: recall %.4f , precision %.4f, F1 %.4f, IoU %.4f \n',...
               interclass(1,4),interclass(1,5),interclass(1,6),interclass(1,7));
           fprintf('Class building: recall %.4f , precision %.4f, F1 %.4f, IoU %.4f \n',...
               interclass(2,4),interclass(2,5),interclass(2,6),interclass(2,7));
           fprintf('Class pole: recall %.4f , precision %.4f, F1 %.4f, IoU %.4f \n',...
               interclass(3,4),interclass(3,5),interclass(3,6),interclass(3,7));
           fprintf('Class car: recall %.4f , precision %.4f, F1 %.4f, IoU %.4f \n',...
               interclass(4,4),interclass(4,5),interclass(4,6),interclass(4,7));
           fprintf('Class veggie: recall %.4f , precision %.4f, F1 %.4f, IoU %.4f \n',...
               interclass(5,4),interclass(5,5),interclass(5,6),interclass(5,7));
       end
   end
end


function labels = labels(data)
    
    labels = data;
    labels(data == 1) = 6; % PROVISIONAL
    labels(data == 2) = 17; % PROVISIONAL
    labels(data == 3) = 2; % PROVISIONAL
    labels(data == 4) = 16; % PROVISIONAL
    labels(data == 5) = 4; % PROVISIONAL
    
    % 0 - undefined
    % 1 - building
    % 2 - car
    % 3 - ground
    % 4 - pole
    % 5 - vegetation
end