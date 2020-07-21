%%%%%%%%%%%%%%%%%%%%%%%%%
%% P4UCC Project
%% S. Ortega, A. Trujillo, J.M. Santana, C. Ramírez, J.P. Suárez
%% Universidad de Las Palmas de Gran Canaria, 2020
%%%%%%%%%%%%%%%%%%%%%%%%%

classdef BuildingTreeDetector
   
    properties(Constant)
        PLANE_PROPORTION = 0.5;
        MIN_POINTS_IN_PLANE = 50;
        
        VERTICAL_OFFSET = 0.1;
        PLANE_ERROR_METERS = 0.1;
    end
    
    methods(Static)
        function cl = extractBuildingsAndTrees(p,feats,rh,classes)
            %% Inputs: points (p), covariance maps (feats),
            %% result of pole detector (classes).
            %% Output: building and tree classification for all the remaining input points. 
            
            cl = classes;
            
            % Unclassified and non-pole-like 
            bIndexes = classes == 0 & feats(:,1) >= PoleDetector.OMNIVARIANCE_OFFSET;
            
            cl(bIndexes) = treeFiltering(p(bIndexes,:),feats(bIndexes,:));
            
            cl = solveDoubts(cl,rh);
        end
        
        function isVertPlane = recursivePlaneExtractor(p,minDistance,cOffset)
            warn = warning('off','all');
            try
                % MSAC plane extractor (pcFitplane).
                pc = pointCloud(p(:,1:3));
                [model,in,out] = pcfitplane(pc,minDistance);
                clear pc;
                % Checks for verticality.
                if abs(model.Parameters(3)) < cOffset
                    isVertPlane = false(size(p,1),1);
                    isVertPlane(in) = true;
                    if length(out) > BuildingTreeDetector.MIN_POINTS_IN_PLANE
                        isVertPlane(out) = ...
                            BuildingTreeDetector.recursivePlaneExtractor(p(out,:), ... 
                                minDistance,cOffset);
                    end
                else
                    isVertPlane = false(size(p,1),1);
                end
            catch
                isVertPlane = false(size(p,1),1);
            end
            warning(warn);
        end 
    end
    
end

%% Private functions.

function classes = treeFiltering(p,cFeats)

    classes = zeros(size(p,1),1);
    
    pc = pointCloud(p(:,1:3));
    idx = pcsegdist(pc,PoleDetector.POLE_CLUSTERING_OFFSET_METERS);

    % Entropy descriptor
    ent = accumarray(idx,cFeats(:,7),[],@mean); ent = ent(idx);

    % Recursive plane extractor caller
    planeIdx = recursiveExtractorCaller(p,idx);
    
    % Calculating plane proportion in cluster
    planeCount = accumarray(idx,planeIdx,[],@sum) ./ ...
        accumarray(idx,ones(size(idx)),[],@sum);
    planeCount = planeCount(idx);
    
    humanObject = ent < PoleDetector.ENT_OFFSET | isnan(ent);

    classes(planeCount < BuildingTreeDetector.PLANE_PROPORTION) = ... 
        JSONConfigManager.TREE_CLASS_VALUE;
    classes(planeCount >= BuildingTreeDetector.PLANE_PROPORTION & humanObject) = ...
        JSONConfigManager.BUILDING_CLASS_VALUE;
    classes(planeCount >= BuildingTreeDetector.PLANE_PROPORTION & ~humanObject) = ... 
        JSONConfigManager.TREE_CLASS_VALUE;
end

function [binary] = recursiveExtractorCaller(p,idx)
    %% It calls the plane extractor per each cluster in the "idx" index.
    %% Output: binary index with vertical plane (1) or non-vertical-plane (0) point.
    
    minDistance = BuildingTreeDetector.PLANE_ERROR_METERS;
    cOffset = BuildingTreeDetector.VERTICAL_OFFSET;
    
    binary = false(size(p,1),1);
    v = accumarray(idx,ones(size(idx)),[],@sum);
    for i=1:max(idx)
        if (v(i) > BuildingTreeDetector.MIN_POINTS_IN_PLANE)
            binary(idx == i) = ...
                BuildingTreeDetector.recursivePlaneExtractor(p(idx == i,:), ...
                    minDistance,cOffset);
        end
    end

end

function cl = solveDoubts(classes,rH)
     cl = classes;
     cl(rH < 0.25) = JSONConfigManager.GROUND_CLASS_VALUE;
     cl(cl == 0) = JSONConfigManager.TREE_CLASS_VALUE;
end
