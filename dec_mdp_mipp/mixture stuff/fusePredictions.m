function gStar = fusePredictions(gHat,pHat)
% function gStar = fusePredictions(gHat,pHat)
% Computes second-order statistics of Gaussian mixtures
%   gHat -- a length n cell array, each cell containing a q-by-2 matrix
%           whose first column is the component means and second column are
%           the component variances
%   pHat -- a q-by-n matrix of per-component mixture parameters
%
%  gStar -- a q-by-2 matrix whose two columns are the mixed second-order 
%           statistics

gStar = zeros(size(pHat,1),2);
for i = 1:length(gHat)
    %disp(i);
    %if numel(gHat{i}) >0
        gStar(:,1) = gStar(:,1) + gHat{i}(:,1).*pHat(:,i);
        gStar(:,2) = gStar(:,2) + (gHat{i}(:,2) + (gHat{i}(:,1)).^2).*pHat(:,i);
    %end
end
gStar(:,2) = gStar(:,2) - gStar(:,1).^2;
