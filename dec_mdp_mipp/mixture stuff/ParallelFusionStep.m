%%% Parallel implementation of GP fusion step among n distributed robots
clear all;
DEBUG = true; % Toggles visualization of fused vs unfused predictions

% (0) Define cell grid, robot regions and EM algorithm parameters
load CommonKnowledge.mat M N V alpha sigma toler; n = length(V);

% Assume each robot records its visited cells since time 0 in Vtilde, a 
% p-by-4 matrix, chronologically in time along the rows: column one is 
% each cell's linear index, column 2 is each observed (scalar) measurement 
% and columns 3&4 are each cell's spatial coordinates
load LocalKnowledge.mat Vtilde i; p = size(Vtilde,1); 

% (1) Estimate local GP prior and broadcast measurements to other robots 
phi = getHyperParameters(Vtilde(:,3:4),Vtilde(:,2));
GP = generateGP(M,N,phi);
% Assume robots' broadcasts are stored in length-n cell array VtilAll, 
% ordered according to robot indices: 
load BroadcastedMeasurements VtilAll; VtilAll{i} = Vtilde(:,1:2);

% (2) Compose measurements in order common to all robots and initialize EM 
pAll = nan(1,n); for j = 1:n, pAll(j) = size(VtilAll{j},1); end
VtilCen = nan(sum(pAll),2); Pmix = nan(sum(pAll),n);
for j = 1:n
  offset = sum(pAll(1:j-1));
  VtilCen(offset+(1:pAll(j)),:) = VtilAll{j}; % Compose visited cells
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
  % Assume robots' broadcasts are stored in length-n cell array logNiAll, 
  % ordered according to robot indices: 
  eval(['load BroadcastedLikelihoods' num2str(k) '.mat logNiAll;']);
  logNiAll{i} = logNi; for j = 1:n, logNiCen(:,j) = logNiAll{j}; end
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

% (4) Broadcast local posterior statistics and compute fused predictions
% Assume robots' broadcasts are stored in length-n cell arrays mHat and 
% kHat, ordered according to robot indices: 
load BroadcastedPosteriors.mat gHat; gHat{i} = [mHat diag(Khat)];
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

if DEBUG
  mStar = gStar(:,1); kStar = gStar(:,2);
  assert(n==2,'Visualization is currently only implemented for two robots');
  Vi = [V{1} V{2}]; 
  load Visualization.mat GP0 GPr; GPr{1} = GP;
  % Compute centralized posterior given ground truth prior
  Psi = sigma^2*eye(size(VtilCen,1));
  [mOpt,K] = posteriorGP(GP0,VtilCen,Psi); kOpt = diag(K);
  % Compute predictions without mixture modeling
  mZero = zeros(M*N,1); kZero = zeros(M*N,1); pZero = zeros(M*N,n);
  for i = 1:n
    Ii = sum(pAll(1:i-1)) + (1:pAll(i));
    [mTemp,Ktemp] = posteriorGP(GPr{i},VtilCen(Ii,:),sigma^2*eye(length(Ii)));
    mZero(V{i}) = mTemp(V{i}); 
    temp = diag(Ktemp); kZero(V{i}) = temp(V{i});
    pZero(V{i},i) = 1; 
  end

  subplot(2,2,1); errorbar(1:M*N,mOpt,sqrt(kOpt),'k.');
  hold on; errorbar(Vi,mStar(Vi),sqrt(kStar(Vi)),'.'); hold off;
  xlim([0.5,M*N+0.5]); yAx1 = ylim;
  xlabel('cell index'); ylabel('cell state');
  SKLD = 0.5*(log(kOpt./kStar) + 0.5*(kStar + (mStar-mOpt).^2)./kOpt - 0.5) + ...
         0.5*(log(kStar./kOpt) + 0.5*(kOpt + (mOpt-mStar).^2)./kStar - 0.5);
  title(['Mixed Predictions (vs Optimal): RMSE=' num2str(norm(mStar-mOpt)) ', SKLD=' num2str(norm(SKLD))]);
  subplot(2,2,2); errorbar(1:M*N,mOpt,sqrt(kOpt),'k.');
  hold on; errorbar(Vi,mZero(Vi),sqrt(kZero(Vi)),'.'); hold off;
  xlim([0.5,M*N+0.5]); yAx2 = ylim;
  xlabel('cell index'); ylabel('cell state');
  SKLD = 0.5*(log(kOpt./kZero) + 0.5*(kZero + (mZero-mOpt).^2)./kOpt - 0.5) + ...
         0.5*(log(kZero./kOpt) + 0.5*(kOpt + (mOpt-mZero).^2)./kZero - 0.5);
  title(['Unmixed Predictions (vs Optimal): RMSE=' num2str(norm(mZero-mOpt)) ', SKLD=' num2str(norm(SKLD))]);
  yAx = [min([yAx1(1),yAx2(1)]) max([yAx1(2),yAx2(2)])];
  subplot(2,2,1); ylim(yAx); subplot(2,2,2); ylim(yAx);
  subplot(2,2,3); bar(Vi(:),[zeros(size(pStar,1),1) pStar],'stacked');
  xlabel('cell index'); ylabel('probability'); title('Mixture Parameters');
  axis([0.5,M*N+0.5,-0.1,1.1]);
  subplot(2,2,4); bar(Vi(:),[zeros(size(pZero,1),1) pZero],'stacked');
  xlabel('cell index'); ylabel('probability'); title('Mixture Parameters');
  axis([0.5,M*N+0.5,-0.1,1.1]);
end