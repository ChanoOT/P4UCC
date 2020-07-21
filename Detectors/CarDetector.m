%%%%%%%%%%%%%%%%%%%%%%%%%
%% P4UCC Project
%% S. Ortega, A. Trujillo, J.M. Santana, C. Ramírez, J.P. Suárez
%% Universidad de Las Palmas de Gran Canaria, 2020
%%%%%%%%%%%%%%%%%%%%%%%%%

classdef CarDetector
   
    properties(Constant)
        EIGENSUM_OFFSET = 1.25;
        MAX_HEIGHT_CAR = 4;
        MAX_GROUND_SEPARATION = 2;
        MAX_AREA = 40;
        
        CAR_CLUSTERING_OFFSET_METERS = 0.5;
        CURVATURE_CHANGE_OFFSET = 0.01;
    end
    
    methods(Static)
        
        function classes = extractCars(cl,heightFeats,rH,feats) 
             %% Inputs: points (cl), height variation maps (hFeats), 
             %% relative height (rH), covariance maps (feats).
             
             %% Output: car classification (or 0 for non considered points).
             
             classes = zeros(size(cl(:,4)));
             
             % Only prisms with eigensum and height requirements are
             % considered for car detection.
             carIndex = feats(:,6) < CarDetector.EIGENSUM_OFFSET & ...
                 heightFeats(:,1) < CarDetector.MAX_HEIGHT_CAR & ...
                 heightFeats(:,2) < CarDetector.MAX_GROUND_SEPARATION;
    
             classes(carIndex) = carFiltering(cl(carIndex,:), ...
                 heightFeats(carIndex,:), rH(carIndex,:),...
                 feats(carIndex,:));
        end
        
    end
end

%% Private functions

function classify = carFiltering(p,hFeats,rH,cFeats)
    classify = zeros(size(p,1),1);
    
    % Grouping all the candidate points with agglomerative hierarchical
    % clustering.
    
    pc = pointCloud(p(:,1:3));
    idx = pcsegdist(pc,CarDetector.CAR_CLUSTERING_OFFSET_METERS);

    % Calculating area descriptors ...
    u = unique(idx);
    vX = accumarray(idx,p(:,1),[],@max) - accumarray(idx,p(:,1),[],@min);
    vY = accumarray(idx,p(:,2),[],@max) - accumarray(idx,p(:,2),[],@min);

    area = vX .* vY;
    float = accumarray(idx,hFeats(:,2),[],@min);
    maxH = accumarray(idx,hFeats(:,1),[],@max);
    
    % Calculating planarity descriptor
    vPlanar = accumarray(idx,cFeats(:,2),[],@mean);
    vPlanar = vPlanar(idx);
    
    % Height lesser than 1m, areas greater than 40 m2 or floating objects
    % are not cars, most probably ground or trees.
    
    % High planarities combined with clusters not in the abovementioned
    % exceptions will be cars.
    
    ground = ismember(idx,u(maxH < 1));
    safe = ismember(idx,u(area < CarDetector.MAX_AREA & float < 0.5));
    
    classify(ground & rH > 0.5) = JSONConfigManager.TREE_CLASS_VALUE;
    classify(ground & rH <= 0.5) = JSONConfigManager.GROUND_CLASS_VALUE;

    classify(~ground & safe & vPlanar > 0.2) = JSONConfigManager.CAR_CLASS_VALUE;
    
    % Doubt clusters are not cars, but are to be classified later or sent
    % empty to other classifiers... 
    
    classify(~ground & ~safe) = JSONConfigManager.TEMP_VALUE_1;
    classify(~ground & safe & vPlanar <= 0.2) = JSONConfigManager.TEMP_VALUE_1;
    
    doubtIdx = classify == JSONConfigManager.TEMP_VALUE_1;
    ent = accumarray(idx,rH,[],@mean); ent = ent(idx);
    classify(doubtIdx) = solveDoubts(p(doubtIdx,:),idx(doubtIdx),...
        classify(doubtIdx),ent(doubtIdx),safe(doubtIdx),rH(doubtIdx));
    
end

    
function cl = solveDoubts(cd,cIdx,classes,ent,safe,rH)
    cl = classes;
    
    meanRelativeHeightOffset = 1;
    localRelativeHeightOffset = 0.5;
    
    [u2,~,spIdx] = unique(cIdx);
    feats = zeros(length(u2),8); ct = 1;
    for i=u2'
        feats(ct,:) = FeatureExtractor3D.correctedGroupalCovarianceFeats(cd(cIdx == i,:));
        ct = ct + 1;
    end
    
    curvchangeOffset = feats(spIdx,8) < CarDetector.CURVATURE_CHANGE_OFFSET; 
     
    cl(ent >= meanRelativeHeightOffset & curvchangeOffset) = JSONConfigManager.BUILDING_CLASS_VALUE;
    cl(ent >= meanRelativeHeightOffset & ~curvchangeOffset) = 0;
    
    cl(ent < meanRelativeHeightOffset & ~safe & rH < localRelativeHeightOffset ) ...
        = JSONConfigManager.GROUND_CLASS_VALUE;
    cl(ent < meanRelativeHeightOffset & ~safe & rH > localRelativeHeightOffset ) ...
        = JSONConfigManager.TREE_CLASS_VALUE;
    cl(ent < meanRelativeHeightOffset & safe) = 0;
end
    




