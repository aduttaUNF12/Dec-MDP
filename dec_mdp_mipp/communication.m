function [gStar,KHat_n,pStar,gHat,msgs] = communication(Vtilde_n, id, comm_robots, GPP, M,N, noMixFlag, V)
%   Detailed explanation goes here
%comm_robots = [id comm_robots];
ncomm = numel(comm_robots);
%disp(comm_robots);
alpha = 1.00e-03;
sigma = 0.500;
toler = 2.00e-04;
%str = 'Vtilde_' + num2str(id) + '.mat';
%save(str,'Vtilde');
% fprintf('%d saved Vtilde',id);
% pause(1/1000);
% (2) Compose measurements in order common to all robots and initialize EM
pAll = nan(1,ncomm);
for j = 1:ncomm
    rob = comm_robots(j);
    %ni = 'Vtilde_' + num2str(comm_robots(j)) + '.mat';
    %load('ni','Vtilde')
    pAll(j) = size(Vtilde_n{rob}(:,1:2),1);
end
VtilCen = nan(sum(pAll),2); Pmix = nan(sum(pAll),ncomm);
for j = 1:ncomm
    %ni = 'Vtilde_' + num2str(comm_robots(j)) + '.mat';
    %load('ni','Vtilde')
    rob = comm_robots(j);
    offset = sum(pAll(1:j-1));
    VtilCen(offset+(1:pAll(j)),:) = Vtilde_n{rob}(:,1:2); % Compose visited cells
    temp = alpha*ones(1,ncomm)/(ncomm-1);
    temp(j) = 1-alpha;
    Pmix(offset+(1:pAll(j)),:) = repmat(temp,pAll(j),1);
end
%fprintf('%d will execute EM',id);

gHat = cell(1,ncomm);
KHat_n = cell(1,ncomm);
if noMixFlag == 0 % mixture is happening
    % (3) Execute EM, each E step requiring broadcast of log likelihoods
    k = 0; logNiCen = nan(sum(pAll),ncomm);
    logNiAll = cell(1,ncomm);
    while 1
        Pold = Pmix; k = k + 1;
        for j=1:ncomm
            % Perform E step
            Khat = GPP{comm_robots(j)}.Sigma; mHat = GPP{comm_robots(j)}.Mu;
            %disp(size(mHat)); disp(size(VtilCen(:,1)));
            temp = sqrt(diag(Khat)); Pgau = [mHat(VtilCen(:,1)) temp(VtilCen(:,1))];
            logNi = log(normpdf((VtilCen(:,2)-Pgau(:,1))./Pgau(:,2)));
            logNi(logNi==Inf) = realmax; logNi(logNi==-Inf) = -realmax;
            logNiAll{j} = logNi;
        end
        % Assume robots' broadcasts are stored in length-n cell array logNiAll,
        % ordered according to robot indices:
        %eval(['load BroadcastedLikelihoods' num2str(k) '.mat logNiAll;']);
        %logNiAll{i} = logNi;
        for j = 1:ncomm
            %ni = 'logni_' + num2str(comm_robots(j)) + '+_' + num2str(k) + '.mat';
            %load('ni','logNi');
            logNiCen(:,j) = logNiAll{j};
        end
        temp = exp(log(Pmix) + logNiCen);
        Pmix = temp ./ repmat(sum(temp,2),1,ncomm);
%         if any(isnan(Pmix(:)))
%             Pmix = Pold;
%             break;
%         end
        % Perform M step
        % this would loop over j = 1:n
        for j = 1:ncomm
            Ii = sum(pAll(1:j-1)) + (1:pAll(j));
            Psi = diag(sigma^2*ones(pAll(j),1)./Pmix(Ii,j));
            [mHat,Khat] = posteriorGP(GPP{comm_robots(j)},VtilCen(Ii,:),Psi);
            gHat{j} = [mHat diag(Khat)]; KHat_n{j} = Khat;
        end
        %mygHat = gHat{1};
        % Test convergence
        delta = norm(Pold(:)-Pmix(:))/sum(pAll);
        
        %disp(['EM iteration ' num2str(k) ': delta = ' num2str(delta) '...']);
        if delta < toler, break; end % Converged
    end
    msgs = k;
    % TO-DO in WEBOTS :: Broadcast [mHat,Khat] and store the received ones in gHat.
    %fprintf('%d will deal with others posteriors\n',id);
    % (4) Broadcast local posterior statistics and compute fused predictions
    % Assume robots' broadcasts are stored in length-n cell arrays mHat and
    % kHat, ordered according to robot indices:
    % load BroadcastedPosteriors.mat gHat; gHat{i} = [mHat diag(Khat)];
    %disp(size(VtilCen));
    %disp(size(Pmix));
    pStar = zeros(M*N,ncomm);
    Vtilu = unique(VtilCen(:,1));
    for v = 1:size(pStar,1) % for each poi
        if any(v==Vtilu) % if poi has been visited
            pStar(v,:) = Pmix(v==Vtilu,:); % Assign mixture from EM
        else % Assign mixture in inverse proportion to posterior variances
            for j = 1:ncomm, pStar(v,j) = 1/gHat{j}(v,2); end
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
    % for future observations: pull out M and k-hats for ID and do noisy
    % observation and do posterior GP update. and calculate gStar again -- line 93.
    %fprintf('%d will return gStar here.',id);

else % no mixture, but opportunistic communication
    % Compute predictions without mixture modeling
    mZero = zeros(M*N,1); kZero = zeros(M*N,1); pZero = zeros(M*N,ncomm);
    for i = 1:ncomm
        Ii = sum(pAll(1:i-1)) + (1:pAll(i));
        [mTemp,Ktemp] = posteriorGP(GPP{comm_robots(i)},VtilCen(Ii,:),sigma^2*eye(length(Ii)));
        mZero(V{comm_robots(i)}) = mTemp(V{comm_robots(i)});
        temp = diag(Ktemp); kZero(V{comm_robots(i)}) = temp(V{comm_robots(i)});
        pZero(V{comm_robots(i)},i) = 1;
        gHat{i} = [mZero kZero];
    end
end
end

