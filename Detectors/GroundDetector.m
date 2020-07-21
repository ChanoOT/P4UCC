classdef GroundDetector
   properties(Constant)
       GRASS_OFFSET = 0.15;
       ESTIMATED_INCLINATION_ANGLE = 1.5;
       PATCH_CLUSTER_OFFSET_METERS = 1;
   end
    
   methods (Static)
       function [classify,sensorApproxPoint,volIdx,sMin] = extractGround(data)
           %% Uses point data as input
           %% Returns: 1 - classification (ground/nonground), 2 - sensor position estimation
           %% 3 - index between point and feature prisms. 4 - size of feature maps.

            % Generating Min Height feature map (Ortega et al. , 2017).
            [rH,hMin,volIdx] = GroundDetector.getHeightIndex(data);

            % Potential ground will be points with relative height below grass height.
            rIndex = rH < GroundDetector.GRASS_OFFSET;

            % Estimating car position with accumulate feature map (Ortega et al., 2017)
            density = zeros(size(hMin));
            idx = volIdx(rIndex);
            [~,~,accumIndex] = unique(idx);
            v = accumarray(accumIndex,ones(size(accumIndex)),[],@sum);

            density(idx) = v(accumIndex)';
            [~,pos] = max(density(:));

            sensorApproxPoint = data(rH == 0 & volIdx' == pos,1:3);
            sensorApproxPoint = sensorApproxPoint(1,:);

            % Ground patches generation via hierarchical agglomerative clustering.

            ground = data(rIndex,1:3); 
            pc = pointCloud(ground);
            gIdx = pcsegdist(pc,GroundDetector.PATCH_CLUSTER_OFFSET_METERS);

            % Finding centroid coordinates vx-vy-vz per cluster. 
            % Calculating alfa inclination angles with respect to estimated
            % sensor position.

            vx = accumarray(gIdx,ground(:,1),[],@mean);
            vy = accumarray(gIdx,ground(:,2),[],@mean);
            vz = accumarray(gIdx,ground(:,3),[],@mean);
            vl = accumarray(gIdx,ones(size(gIdx)),[],@sum);
            v = [vx,vy,vz]; v = v - sensorApproxPoint;

            hypothenuse = sqrt(sum(v.^2,2));
            cathetus = vz - sensorApproxPoint(1,3);

            alfas = asind(abs(cathetus) ./ abs(hypothenuse));

            classify = rIndex;
            [~,gPatch] = max(vl);
            classify(rIndex) = alfas(gIdx) < GroundDetector.ESTIMATED_INCLINATION_ANGLE | gIdx == gPatch;

            classify = double(classify) .* JSONConfigManager.GROUND_CLASS_VALUE;
            sMin = size(hMin);
  
       end
      
       function [heightIndex,hMin,volIdx] = getHeightIndex(points)
           
            % It gives a bbox of the cloud
            [maxX,maxY,minX,minY] = GeneralUtilities.bbox2D(points);
            
            % It associates each point with a 1 sq. meter size cell of a
            % regular grid. 
        
            fX = floor(minX); cX = floor(maxX) + 1; sX = cX - fX;
            fY = floor(minY); cY = floor(maxY) + 1; sY = cY - fY;

            hMin = ones(sY,sX)*NaN;

            volIdx(:) = sub2ind(size(hMin),floor(points(:,2)) - fY + 1,...
                floor(points(:,1)) - fX + 1);
            
            % It generates a grid map of minimum height
            [~,~,idx] = unique(volIdx);
            v = accumarray(idx,points(:,3),[],@min);

            hMin(volIdx) = v(idx);
            
            % Based on the map, it also calculates relative height.
            heightIndex = points(:,3) - v(idx);

       end
        
       function [feats,hMin] = heightVariationMaps(p,volIdx,sMin)
            %% It extracts feature maps of height variation 
            %% Inputs: points, index point-prism, size of feature maps.
            %% Outputs: 
            %% feats(:,1) == difference highest point in each prism vs
            %% ground level.
            %% feats(:,2) == difference lowest non-ground point vs ground
            %% level.
            %% hMin = ground level height map.
            
            % For cases in which a prism is composed of only non-ground
            % points, hMin value is calculated via interpolation from
            % ground values.
            
            gIdx = p(:,4) == JSONConfigManager.GROUND_CLASS_VALUE;

            hMin = ones(sMin).* NaN; hMax = hMin; hMeta = hMin;
            [~,~,gIndex] = unique(volIdx(gIdx));
            v = accumarray(gIndex,p(gIdx,3),[],@min);
            hMin(volIdx(gIdx)) = v(gIndex);
            hMin = regionfill(hMin,isnan(hMin));

            [~,~,gIndex] = unique(volIdx(~gIdx));
            v = accumarray(gIndex,p(~gIdx,3),[],@max);
            v2 = accumarray(gIndex,p(~gIdx,3),[],@min);
            hMax(volIdx(~gIdx)) = v(gIndex);
            hMeta(volIdx(~gIdx)) = v2(gIndex);

            hDif = hMax - hMin;
            hDif2 = hMeta - hMin;

            ngVol = volIdx(~gIdx);

            feats = [hDif(ngVol)',hDif2(ngVol)'];

       end
       
       function [feats] = gridCovarianceMaps(cl,clIdx)
           
           % Returns the covariance features (Blomley et al., 2014, Zhao et al., 2019)
           % Inputs: points (cl), index point-prisms (clIdx)
           % Outputs:
           % feats(:,1) == omnivariance
           % feats(:,2) == planarity
           % feats(:,3) == linearity
           % feats(:,4) == anisotrophy
           % feats(:,5) == sphericity
           % feats(:,6) == eigensum
           % feats(:,7) == eigenentropy
           % feats(:,8) == curvature change
           
            [u,~,cIndex] = unique(clIdx);
            feats = zeros(length(u),8); ct=1;
            for i=u
                feats(ct,:) = FeatureExtractor3D.correctedGroupalCovarianceFeats(cl(clIdx == i,:));
                ct = ct+1;
            end
            feats = feats(cIndex,:);
       end
       
   end
end