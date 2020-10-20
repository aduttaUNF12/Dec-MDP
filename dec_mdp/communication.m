function [gStar] = communication(Vtilde, id, n, comm_robots)% n includes the communicating robot as well. Thus, length(comm_robots) = n and comm_robots(1) = r_i.
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
str = 'Vtilde_' + num2str(id) + '.mat';
save(str,'Vtilde');

% (2) Compose measurements in order common to all robots and initialize EM
pAll = nan(1,n);
for j = 1:n
    ni = 'Vtilde_' + num2str(comm_robots(j)) + '.mat';
    load('ni','Vtilde')
    pAll(j) = size(Vtilde(:,1:2),1);
end
VtilCen = nan(sum(pAll),2); Pmix = nan(sum(pAll),n);
for j = 1:n
    ni = 'Vtilde_' + num2str(comm_robots(j)) + '.mat';
    load('ni','Vtilde')
    offset = sum(pAll(1:j-1));
    VtilCen(offset+(1:pAll(j)),:) = Vtilde(:,1:2); % Compose visited cells
    temp = alpha*ones(1,n)/(n-1); temp(j) = 1-alpha;
    Pmix(offset+(1:pAll(j)),:) = repmat(temp,pAll(j),1);
end

% (3) Execute EM, each E step requiring broadcast of log likelihoods
k = 0; Khat = GP.Sigma; mHat = GP.Mu; logNiCen = nan(sum(pAll),n);
while 1
    Pold = Pmix; k = k + 1;
    % Perform E step
    temp = sqrt(diag(Khat)); Pgau = [mHat(VtilCen(:,1)) temp(VtilCen(:,1))];
    logNi = log(normpdf((VtilCen(:,2)-Pgau(:,1))./Pgau(:,2)));
    logNi(logNi==Inf) = realmax; logNi(logNi==-Inf) = -realmax;
    
    % TO-DO in WEBOTS :: Broadcast logNi and store the received ones in logNiAll.
    str = 'logni_' + num2str(id) + '+_' + num2str(k) + '.mat';
    save(str,'logNi');
    
    % Assume robots' broadcasts are stored in length-n cell array logNiAll,
    % ordered according to robot indices:
    %eval(['load BroadcastedLikelihoods' num2str(k) '.mat logNiAll;']);
    %logNiAll{i} = logNi;
    for j = 1:n
        ni = 'logni_' + num2str(id) + '+_' + num2str(comm_robots(j)) + '.mat';
        load('ni','logNi');
        logNiCen(:,j) = logNi;
    end
    temp = exp(log(Pmix) + logNiCen);
    Pmix = temp ./ repmat(sum(temp,2),1,n);
    
    % Perform M step
    Ii = sum(pAll(1:i-1)) + (1:pAll(i));
    Psi = diag(sigma^2*ones(pAll(i),1)./Pmix(Ii,i));
    [mHat,Khat] = posteriorGP(GP,VtilCen(Ii,:),Psi);
    
    % Test convergence
    delta = norm(Pold(:)-Pmix(:))/sum(pAll);
    disp(['EM iteration ' num2str(k) ': delta = ' num2str(delta) '...']);
    if delta < toler, break; end % Converged
end

% TO-DO in WEBOTS :: Broadcast [mHat,Khat] and store the received ones in gHat.

% (4) Broadcast local posterior statistics and compute fused predictions
% Assume robots' broadcasts are stored in length-n cell arrays mHat and
% kHat, ordered according to robot indices:
% load BroadcastedPosteriors.mat gHat; gHat{i} = [mHat diag(Khat)];
str = 'mkhat_' + num2str(id) + '.mat';
save(str,'mHat','Khat');
gHat = cell(1,n);
for j = 1:n
    ni = 'mkhat_' + num2str(comm_robots(j)) + '.mat';
    load('ni','mHat','Khat')
    gHat{i} = [mHat diag(Khat)];
end

pStar = zeros(M*N,n);
for v = 1:size(pStar,1) % for each poi
    if any(v==VtilCen(:,1)) % if poi has been visited
        pStar(v,:) = Pmix(v==VtilCen(:,1),:); % Assign mixture from EM
    else % Assign mixture in inverse proportion to posterior variances
        for i = 1:n, pStar(v,i) = 1/gHat{i}(v,2); end
        pStar(v,:) = pStar(v,:) / sum(pStar(v,:));
    end
end
gStar = fusePredictions(gHat,pStar);
% For path planning from here, treat
%   gStar(:,1) as the posterior mean of all M*N cells
%   gStar(:,2) as the posterior variance of all M*N cells
% Note: if robot i is constrained to the cells in region V{i}, then it is
%       sufficient to compute only gStar(V{i},:) (and before that only
%       pHat(V{i},:) accordingly), but we did not exercise such savings in
%       computationl here
end

