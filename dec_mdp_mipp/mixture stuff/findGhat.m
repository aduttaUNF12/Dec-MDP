function gStar = findGstar(gHat,pStar)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% pStar = zeros(M*N,n);
% for v = 1:size(pStar,1) % for each poi
%     if any(v==VtilCen(:,1)) % if poi has been visited
%         pStar(v,:) = Pmix(v==VtilCen(:,1),:); % Assign mixture from EM
%     else % Assign mixture in inverse proportion to posterior variances
%         for j = 1:n, pStar(v,j) = 1/gHat(v,2); end
%         pStar(v,:) = pStar(v,:) / sum(pStar(v,:));
%     end
% end
gStar = fusePredictions(gHat(1),pStar);
end

