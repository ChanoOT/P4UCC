%%%%%%%%%%%%%%%%%%%%%%%%%
%% P4UCC Project
%% S. Ortega, A. Trujillo, J.M. Santana, C. Ramírez, J.P. Suárez
%% Universidad de Las Palmas de Gran Canaria, 2020
%%%%%%%%%%%%%%%%%%%%%%%%%

classdef PoleDetector
    
    properties(Constant)
        OMNIVARIANCE_OFFSET = 0.2;
        
        DISPERSION_OFFSET = 1;
        CURVATURE_CHANGE_OFFSET = 0.01;
        ENT_OFFSET = -10;
        
        POLE_CLUSTERING_OFFSET_METERS = 0.75;
    end
    
    methods(Static)
        
        function [cl] = extractPoles(p,rH,feats,classes) 
            %% Inputs: points (p), relative height (rH),
            %% covariance maps (feats), result of car detector (classes).
            %% Output: pole classification (or 0 for non considered points).
            
            cl = classes;
            
            % Only unclassified points and cells with omnivariance below the 
            % limits will be considered for pole detection.
            poleIndex = classes == 0 & feats(:,1) < PoleDetector.OMNIVARIANCE_OFFSET;
            
            cl(poleIndex) = poleFiltering(p(poleIndex,:),rH(poleIndex),feats(poleIndex,:));
            
        end
        
    end
    
end

% Private functions

function classes = poleFiltering(p,rH,feats)
    classes = zeros(size(p,1),1);
    
    % Grouping poles via agglomerative hierarchical clustering.
    pc = pointCloud(p(:,1:3));
    idx = pcsegdist(pc,PoleDetector.POLE_CLUSTERING_OFFSET_METERS);
    
    % Calculating pole bbox dimensions
    vX = accumarray(idx,p(:,1),[],@max) - accumarray(idx,p(:,1),[],@min);
    vY = accumarray(idx,p(:,2),[],@max) - accumarray(idx,p(:,2),[],@min);
    vZ = accumarray(idx,p(:,3),[],@max) - accumarray(idx,p(:,3),[],@min);
    
    % Extracting coordinate dispersion, curvature change and entropy descriptors per cluster
    float = accumarray(idx,rH,[],@min);
    cChange = accumarray(idx,feats(:,8),[],@mean);
    ent = accumarray(idx,feats(:,7),[],@mean);
    disp = max(accumarray(idx,p(:,1),[],@std),accumarray(idx,p(:,2),[],@std));
    disp = disp(idx);
    ent = ent(idx);
    
    vPole = vZ > vX & vZ > vY & float < 1; 
    vPole = vPole(idx);
    cChange = cChange(idx);
    
    % Non floating with dominant Z, low curvature change and low
    % dispersion: pole !
    classes(vPole & cChange < PoleDetector.CURVATURE_CHANGE_OFFSET & ...
        disp < PoleDetector.DISPERSION_OFFSET) = JSONConfigManager.POLE_CLASS_VALUE;
    
    % Rest of candidates (groups that look like a pole but they are not): 
    % mostly building or tree.
    classes(vPole & cChange < PoleDetector.CURVATURE_CHANGE_OFFSET & ....
        disp >= PoleDetector.DISPERSION_OFFSET) = JSONConfigManager.BUILDING_CLASS_VALUE;
    classes(vPole & cChange >= PoleDetector.CURVATURE_CHANGE_OFFSET) = JSONConfigManager.TREE_CLASS_VALUE;
    
    classes(~vPole & cChange < PoleDetector.CURVATURE_CHANGE_OFFSET & ... 
        disp >= PoleDetector.DISPERSION_OFFSET) = JSONConfigManager.BUILDING_CLASS_VALUE;
    classes(~vPole & cChange < PoleDetector.CURVATURE_CHANGE_OFFSET & ...
        disp < PoleDetector.DISPERSION_OFFSET & ent < PoleDetector.ENT_OFFSET) = JSONConfigManager.BUILDING_CLASS_VALUE;
    classes(~vPole & cChange < PoleDetector.CURVATURE_CHANGE_OFFSET & ...
        disp < PoleDetector.DISPERSION_OFFSET & ent >= PoleDetector.ENT_OFFSET) = JSONConfigManager.TREE_CLASS_VALUE;
    
    classes(~vPole & cChange >= PoleDetector.CURVATURE_CHANGE_OFFSET) = ...
        solveDoubts(p(~vPole & cChange >= PoleDetector.CURVATURE_CHANGE_OFFSET,:),...
            feats(~vPole & cChange >=PoleDetector.CURVATURE_CHANGE_OFFSET,:), ...
            rH(~vPole & cChange >= PoleDetector.CURVATURE_CHANGE_OFFSET));
        
end


function classes = solveDoubts(p,cFeats,rH)

    classes = zeros(size(p,1),1);
    
    pc = pointCloud(p(:,1:3));
    idx = pcsegdist(pc,PoleDetector.POLE_CLUSTERING_OFFSET_METERS);
    
    vZ = accumarray(idx,p(:,3),[],@max) - accumarray(idx,p(:,3),[],@min);
    
    cFeats(isnan(cFeats(:,7)),7) = 0;
    
    ent = accumarray(idx,cFeats(:,7),[],@mean);
    float = accumarray(idx,rH,[],@min);
    
    planarity = zeros(length(idx),1);
    for i=1:max(idx)
        planarity(i) = sum(theRansacLineFit(p(idx == i,:))) / size(p(idx == i,:),1);
    end
 
    vZ = vZ(idx); 
    planarity = planarity(idx);
    ent = ent(idx);
    float = float(idx);
    classes(planarity <= 0.33) = 4;
    classes(planarity > 0.33 & planarity <= 0.5 & float > 1) = JSONConfigManager.TREE_CLASS_VALUE;
    classes(planarity > 0.33 & planarity <= 0.5 & float <= 1 & ent <= -1.5) = JSONConfigManager.BUILDING_CLASS_VALUE;
    classes(planarity > 0.33 & planarity <= 0.5 & float <= 1 & ent > -1.5) = JSONConfigManager.TREE_CLASS_VALUE;
    
    naturalElements = ent > -1.5 |  float > 2;
    
    classes(vZ > 2.5 & planarity > 0.5 & float <= 1) = JSONConfigManager.BUILDING_CLASS_VALUE; 
    classes(vZ > 2.5 & planarity > 0.5 & float > 1 & ent > -1.5) = JSONConfigManager.TREE_CLASS_VALUE; 
    classes(vZ > 2.5 & planarity > 0.5 & float > 1 & ent <= -1.5) = JSONConfigManager.BUILDING_CLASS_VALUE; 
    
    classes(vZ <= 2.5 & planarity > 0.5 & ~naturalElements) = JSONConfigManager.BUILDING_CLASS_VALUE;
    classes(vZ <= 2.5 & planarity > 0.5 & naturalElements) = JSONConfigManager.TREE_CLASS_VALUE;
    
end

function inliers = theRansacLineFit(points)
    if size(points,1) < 5
        inliers = zeros(size(points,1),1);
        return
    end
    w = warning('off','all');
    sampleSize = 2; 
    maxDistance = 0.1; 

    fitLineFcn = @(points) polyfit(points(:,1),points(:,2),1); 
    evalLineFcn = @(model, points) sum((points(:, 2) - polyval(model, points(:,1))).^2,2);

    [~, inliers] = ransac(points,fitLineFcn,evalLineFcn, sampleSize,maxDistance);
    warning(w);
end