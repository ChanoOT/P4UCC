%%%%%%%%%%%%%%%%%%%%%%%%%
%% P4UCC Project
%% S. Ortega, A. Trujillo, J.M. Santana, C. Ramírez, J.P. Suárez
%% Universidad de Las Palmas de Gran Canaria, 2020
%%%%%%%%%%%%%%%%%%%%%%%%%

classdef DrawStuff
    methods(Static)
        function theScatter(points,idx)
            figure; 
            scatter3(points(:,1),points(:,2),points(:,3),1,idx);
            axis equal;
        end
        
        function plotPoints(points)
            %% Exige una estructura tipo MatPointCloud (xyz-i-cl-ret)
            pcPlotSection(points,true);
        end
        
        function plotIndex(cl,idx,name)
            figure; axis equal; hold on;
            plot3(cl(~idx,1),cl(~idx,2),cl(~idx,3),'.r');
            plot3(cl(idx,1),cl(idx,2),cl(idx,3),'.g');
            title(name);
        end
        
        %%%%%%%
        %% FUNCIONES PARA SHREC
        %%%%%%%
        
        function openAndShow(folder,filename)
           data = importTxtCloud(sprintf('%s%s.txt',folder,filename));
           DrawStuff.showData(data,true);
        end
        
        function showData(data,withConversion)
            if exist('withConversion','var') && withConversion
                classify = labels(data(:,4));
            else
                classify = data(:,4);
            end
            DrawStuff.plotPoints([data(:,1:3),classify]);
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


function pcPlotSection(filename,isAlreadyOpen)
    warn = warning('off','all');
    if (~exist('isAlreadyOpen','var') || ~isAlreadyOpen)
        load(filename,'points');
        c = generateColor(points);
        figure; axis equal; 
        pcshow(points(:,1:3),c);
    else
        c = generateColor(filename);
        figure; axis equal; 
        pcshow(filename(:,1:3),c);
    end
    warning(warn);
end

function c = generateColor(p)
    c = zeros(size(p,1),3);
    
    
   theNormals = p(:,4) == 2 | p(:,4) == 25 | ((p(:,4) > 8 & p(:,4) < 12));
   % Algo extraño
   theUnclassifieds = p(:,4) == 21 | p(:,4) == 30 | p(:,4) == 27;
   
   % Veggies
   theGrass = p(:,4) == 3 | p(:,4) == 22;
   theTrees = (p(:,4) > 3 & p(:,4) < 6) | (p(:,4) > 22 & p(:,4) < 25);
   theBuildings = p(:,4) == 6 | p(:,4) == 8;
   theTurrets = p(:,4) == 16;
   theSecondaryTurrets = p(:,4) == 19;
   
   theWires = p(:,4) == 13 | p(:,4) == 14 | p(:,4) == 17 | p(:,4) == 18;
   
   theSecondaryWires = p(:,4) == 20;
   theSpecialWires = p(:,4) == 31;
   theChains = p(:,4) == 29;
   
   theRares = p(:,4) == 15;
   
   theZeros = p(:,4) == 0;
   
   %% El resto irá a 0 por sí mismo.

   c(theNormals,1) = 1; c(theNormals,2) = 1; 
   c(theUnclassifieds,2) = 1; c(theUnclassifieds,3) = 1;
   c(theGrass,2) = 1;
   c(theTrees,2) = 0.5;
   c(theBuildings,1) = 1;
   c(theTurrets,3) = 1;
   c(theSecondaryTurrets,2) = 0.5; c(theSecondaryTurrets,3) = 0.5;
   c(theWires,1) = 1;  c(theWires,3) = 1;
   %c(theSpecialWires,1) = 1; c(theSpecialWires,3) = 1;
   c(theSecondaryWires,1) = 0.8; c(theSecondaryWires,2) = 0.8; c(theSecondaryWires,3) = 0.8;
   c(theSpecialWires,1) = 1; c(theSpecialWires,2) = 0.5;
   c(theChains,1) = 0.5; c(theChains,3) = 0.5;
   
   c(theRares,1) = 0.5; c(theRares,2) = 0.5;
   c(theZeros,1) = 0.5; c(theZeros,2) = 0.5; c(theZeros,3) = 0.5;
   
%    c = uint8(c);
end