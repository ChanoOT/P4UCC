%%%%%%%%%%%%%%%%%%%%%%%%%
%% P4UCC Project
%% S. Ortega, A. Trujillo, J.M. Santana, C. Ramírez, J.P. Suárez
%% Universidad de Las Palmas de Gran Canaria, 2020
%%%%%%%%%%%%%%%%%%%%%%%%%

classdef FeatureExtractor3D
    methods(Static)
               
        function feats = correctedGroupalCovarianceFeats(xyz)
            %% Ésta es la que hemos utilizado hasta ahora.
             feats = zeros(1,8);
             
             if length(xyz) < 8, return, end
             eigs = sort(eig(cov(xyz(:,1:3))),'descend');
              
            feats(1) = nthroot(eigs(1) .* eigs(2) .* eigs(3),3);
            feats(2) = (eigs(2) - eigs(3)) ./ eigs(1);
            feats(3) = (eigs(1) - eigs(2)) ./ eigs(1);
            feats(4) = eigs(3) ./ eigs(1) ;
            feats(5) = (eigs(1) - eigs(3)) ./ eigs(1);
            feats(6) = eigs(1) + eigs(2) + eigs(3);
            feats(7) = -( (eigs(1) .* log(eigs(1))) + (eigs(2) .* ...
                log(eigs(2))) + (eigs(3) .* log(eigs(3))));
            feats(8) = eigs(3) ./ feats(6);
              
        end
        
    end
end