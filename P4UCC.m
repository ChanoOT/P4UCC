%%%%%%%%%%%%%%%%%%%%%%%%%
%% P4UCC Project
%% S. Ortega, A. Trujillo, J.M. Santana, C. Ramírez, J.P. Suárez
%% Universidad de Las Palmas de Gran Canaria, 2020
%%%%%%%%%%%%%%%%%%%%%%%%%

function P4UCC(configFile)
    %% Main function of the classifier
    %% It expects a config.json file in the route provided in configFile
    %% When not provided, the system supposes it is found in the executable folder.
    
    close all;
    matrix = zeros(5,6);
    
    if exist('configFile','var')
        JSONConfigManager.generateConfig(configFile)
    else
        JSONConfigManager.generateDefaultConfig();
    end
    
    [a,b,c] = JSONConfigManager.getGeneralInformation();
%     tic;
    for i=1:size(b,1)
        matrix = matrix + launchTest(a,b{i},c);
    end
%     fprintf('Simulation concluded in %.2f seconds \n',toc);
    fprintf('--------------------------------- \n Overall data \n');
    SHRECValidator.intraClassFromMatrix(matrix);

end

function matrix = launchTest(filefolder, filename, output)
    %% It launches an individual instance of the P4UCC
    %% Input point cloud is in the path fileFolder/fileName.txt
    %% Output point cloud will be generated in output/fileName.txt
    
    s = sprintf('%s%s.txt',filefolder,filename);
    data = importTxtCloud(s);
    
    % Validator initialization with Ground Truth (GT). 
    % Comment this line for no final testing of the behaviour. 
    validator = SHRECValidator(data(:,4));
    
    % Overriding GT
    data(:,4) = 0;
    
    % Ground detector (o eso creo). 
    [data(:,4),~,volIdx,sMin] = GroundDetector.extractGround(data);
    [heightFeats,hMap] = GroundDetector.heightVariationMaps(data,volIdx,sMin);
    
    % Processing non-ground points ...
    intoAccount = data(:,4) ~= JSONConfigManager.GROUND_CLASS_VALUE;
    data(intoAccount,4) = progressiveNonGroundStages(data(intoAccount,:),...
        volIdx(intoAccount),heightFeats,hMap);
    
    matrix = GeneralUtilities.validatingPrintingAndSaving(data,validator,output,filename);
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [classes] = progressiveNonGroundStages(cl,clIdx,heightFeats,hMap)

    feats = GroundDetector.gridCovarianceMaps(cl,clIdx);
    rH = cl(:,3) - hMap(clIdx)';

    %% Car detector: input will be all prisms with eigensum lesser than 1.25. (Epsilon X)
    classes = CarDetector.extractCars(cl,heightFeats,rH,feats);
    classes = PoleDetector.extractPoles(cl,rH,feats,classes);
    classes = BuildingTreeDetector.extractBuildingsAndTrees(cl,feats,rH,classes); 
 
end







