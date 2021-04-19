function [oracle] = composeOracleData(VtilAll, n)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
alpha = 1.00e-03;
pAll = nan(1,n); for j = 1:n, pAll(j) = size(VtilAll{j},1); end
oracle = nan(sum(pAll),2); Pmix = nan(sum(pAll),n);
for j = 1:n
  offset = sum(pAll(1:j-1));
  oracle(offset+(1:pAll(j)),:) = VtilAll{j}(:,1:2); % Compose visited cells
  temp = alpha*ones(1,n)/(n-1); temp(j) = 1-alpha;
  Pmix(offset+(1:pAll(j)),:) = repmat(temp,pAll(j),1);
end
end

