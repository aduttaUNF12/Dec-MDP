function [] = mdp_IPP(n,M,x_active,noCommFlag,allCommFlag,noMixFlag,idx, centr, range,run)
%for n = 2:2:2 %robot count
%clear
%range = 0.5; % a var for CR --> diag * range.
if allCommFlag ==1
    range = 1;
end
if noCommFlag ==1
    range = 0;
end
%% arrays of data for n robots
unseen_n = cell (1,n);
unseen_idx_n = cell (1,n);
GW_n = cell(1,n);
myStates_n =cell (1,n);
myStates2d_n = cell (1,n);
otherRobotStates_n = cell (1,n);
budget_n = zeros(1,n);
Vtilde_n = cell (1,n);
trans_n = cell (1,n);
GP_n = cell(1,n);
GPP_n = cell(1,n);%Pat's GP models
Xactive_n = cell(1,n);
%Ypred_n =cell(1,n);
%Ysd_n = cell(1,n);
Yactive_n = cell(1,n);
train_locs_n = cell(1,n);
R_n = cell(1,n);
policy_n = cell(1,n);
Pred_n = cell(1,n);
SD_n = cell(1,n);
commr_n = cell(1,n); %commr_n{i} = [];
gStar_n = cell(1,n);
Khat_n = cell(1,n);
gHat_n = cell(1,n);
pStar_n = cell(1,n);
GPupdateflag_n = zeros(1,n);

% Data collection -- stores all robots' data together in n-length cells.
allMSE = cell(1,n);
allReward = cell(1,n);
%allVar = cell(1,n);
allVar = cell(1,n);
allSKLD = cell(1,n);
allPaths = cell(1,n);
allCC = [];%holds the #connected components formed bc of CR
posInList = zeros(1,n);%holds the index of the i-th robot in the comm. list -- gets updated w/ new neighbors.
msgcount = 0;

%% variable initialization -- common data to all robots.
%M = 12;% square environment: size x size
[x,y] = meshgrid(1:1:M,1:1:M);
all_locs = [y(:), x(:)];
unseen = all_locs;% initially every location is unseen.
update_freq = 1;
pUpdate_freq = 5;
%penalty = 0;% for visiting an already observed cell.
max_trn_data = 0.5;% percentage of all nodes in my partition.
initDataPlot = 0;
pGood = 0.8;
pBad = 0.2;
neighbors = cell(1,M);
CR = sqrt(2)*M*range;
% for Pat's code
%M=M;
N=M;
alpha = 1.00e-03;
sigma = 0.25;
toler = 2.00e-04;

%% create a GRID world -- one for each robot
for i=1:n
    GW_n{i} = createGridWorld(M,M);
    newrow = centr(i,1); newcol = centr(i,2);
    GW_n{i}.CurrentState = append('[',num2str(newrow),',',num2str(newcol),']');
    myStates_n{i} = find(idx == i);% only adding my partition's cells to my state-space.
    %disp(numel(myStates_n{i}));
    myStates2d_n{i} = all_locs(myStates_n{i},:);
    otherRobotStates_n{i} = find(idx ~= i);% other robots' states are obstacles to me -- will be treated accordingly.
    budget_n(i) = 20;%ceil(max_trn_data*numel(myStates_n{i}));% budget parameter
    unseen_n{i} = unseen;
    Vtilde_n{i} = [];
    
    allReward{i} = [];  
    allMSE{i} = [];%zeros(1, budget_n(i));
end


%% padded with obstacles to avoid going out of the arena.
obstacles = [];
lr = M;fr = 1;lc = M*M;
for ind = 1:1:M
    fc=ind;
    %GW.T(fr,:,1) = 0;% taking action N in the top row is prohibited.
    obstacles = [obstacles idx2state(GW_n{1},fc)];
    %GW.T(lr,:,2) = 0;% taking action S in the bottom row is prohibited.
    obstacles = [obstacles idx2state(GW_n{1},lr)];
    lr = lr + M;
    %GW.T(fc,:,4) = 0;% taking action W in the left col is prohibited.
    obstacles = [obstacles idx2state(GW_n{1},lc)];
    lc = lc-1;
    %GW.T(lc,:,3) = 0;% taking action E in the right col is prohibited.
    obstacles = [obstacles idx2state(GW_n{1},fr)];
    fr = fr+M;
end

for i=1:n
    otherRobotStates_n{i} = otherRobotStates_n{i}';
    %myObstacles = [obstacles (idx2state(GW_n{i},otherRobotStates_n{i}))'];% other robots' states are added as obstacles to avoid going in there...
    %GW_n{i}.ObstacleStates = unique(myObstacles,'stable');
    %updateStateTranstionForObstacles(GW);
    myObstacles = (idx2state(GW_n{i},otherRobotStates_n{i}))';% other robots' states are added as obstacles to avoid going in there...
    GW_n{i}.ObstacleStates = unique(myObstacles,'stable');
    [GW_n{i}.T,neighbors] = customTransition(GW_n{i},M, pGood, pBad);% update the transition (s,s',a) matrix
    %northStateTransition = GW.T(:,:,1);
    %southStateTransition = GW.T(:,:,2);
    %eastStateTransition = GW.T(:,:,3);
    %westStateTransition = GW.T(:,:,4);
    %updateStateTranstionForObstacles(GW_n{i});
    trans_n{i} = GW_n{i}.T;
    trans_n{i} = trans_n{i}(myStates_n{i},:,:);
    trans_n{i} = trans_n{i}(:,myStates_n{i},:);% extract the transition sub-matrix for my partition.
end

%% Read the ground-truth data from file & initialize GP
tic
data_gt = data_import('dataFile_ell25_50by50.csv', 1, 50);
data_gt = data_gt(1:M,1:M);
Y_gt = str2double(reshape(table2array(data_gt),[M*M 1]));
X_train = (1:M*M)';
Z_gt = str2double(table2array(data_gt));
%x_active = randsample(X_train,ceil(init_trn_data*M*M));%select k% random points to initialize GP hyperparameters.
init_training_locs = all_locs(x_active,:);% initial locs to initialize the hyperparameters -- same for all robots.
y_active = Y_gt(x_active);% meausurements of the initial training locs -- same for all robots.

for i=1:n
    unseen_n{i} = setdiff(unseen_n{i},init_training_locs,'rows');% unseen locations in the environment (includes other robot states).
    unseen_idx_n{i} = setdiff(X_train,x_active);
    Xactive_n{i} = x_active;
end
sigma0 = std(y_active);
%kparams0 = [3.5, 0.25];
%gprMdl = fitrgp(init_training_locs,y_active,'KernelFunction','squaredexponential','Sigma',sigma0);%using 2D attribute -- (x,y) coordinates
%ypred = resubPredict(gprMdl);% not actually needed -- just for testing.

%% add the starting location to the training matrix and do the 1st prediction
for i=1:n
    GW = GW_n{i};
    %y_gt = Y_gt(state2idx(GW,GW.CurrentState));%measure the data at the i-th location.
    [currrow,currcol] = state2rc(GW.CurrentState);
    %newData = {currrow,currcol,y_gt};
    %attributes = [attributes;newData];
    Xactive_n{i}(numel(Xactive_n{i})+1) = state2idx(GW,GW.CurrentState);%add the new visited state for retraining.
    Xactive_n{i} = unique(Xactive_n{i},'stable');%keep unique locations
    noiseVec = normrnd(0,sigma, size(Y_gt(Xactive_n{i})));
    Yactive_n{i} = Y_gt(Xactive_n{i}) + noiseVec;%add the new measurement for retraining.
    train_locs_n{i} = init_training_locs;
    train_locs_n{i} = unique(train_locs_n{i},'rows','stable');
    if numel(intersect(train_locs_n{i}, [currrow,currcol],'rows'))==0
        train_locs_n{i} = [train_locs_n{i}; currrow currcol];
    end
    Vtilde_n{i} = [Xactive_n{i} Yactive_n{i} train_locs_n{i}];
    %new = [76 0 5 5];
    %shared_data = [shared_data; 76 0 5 5];
    %y_active = [y_active; y_gt];
    unseen_n{i} = setdiff(all_locs, train_locs_n{i},'rows','stable');%setdiff(unseen,[currrow,currcol],'rows');
    unseen_idx_n{i} = setdiff(X_train,Xactive_n{i},'stable');
    %gprMdl = update_GP_model(gprMdl,x_active, y_active);%update the current GP model.
    GP_n{i} = fitrgp(train_locs_n{i},Yactive_n{i},'KernelFunction','exponential','Sigma',sigma0);%using 2D attribute -- (x,y) coordinates
    %[Y_pred,Y_sd] = predict(GP_n{i}, all_locs);%predict measurements for the locations with the new model.
    %SD_n{i} = Y_sd;
    phi = GP_n{i}.KernelInformation.KernelParameters;
    GPP_n{i} = generateGP(M,N,phi);% using pat's code.
    Pred_n{i} = GPP_n{i}.Mu;
    SD_n{i} = diag(GPP_n{i}.Sigma);
    %initData = Vtilde_n{i}(:,1:2);
    %Psi = sigma^2*eye(size(initData,1));
    %[mInit,~] = posteriorGP(GPP_n{i},initData,Psi);
    allVar{i} = [allVar{i}, SD_n{i}];
    allMSE{i} = [allMSE{i},(immse(GPP_n{i}.Mu,Y_gt(X_train)))];%calculate the current RMSE for ALL cells.
end
%GP0 = GPP_n{1};% needs to change w/ GOD GP.
%% for online entropy calculation - Entropy(Z_A) w/o any conditionals
% kfcn = gprMdl.Impl.Kernel.makeKernelAsFunctionOfXNXM(gprMdl.Impl.ThetaHat);
% cov_mat = kfcn(x_active(:,:),x_active(:,:));%full covariance matrix.
% U_size = numel(unseen);%number of unseen locs
% %H_Za = 0.5 * log(((2*pi*exp(1))^U_size)* det(cov_mat)); %Liu entropy for online measurement
% H_za = 0.5 * log (det(cov_mat)) + ((size*size)/2) * (1 + log(2*pi)); % Wei, Zheng (2020)

%% plot initial results
if initDataPlot == 1
    figure(1);
    plot(x_active,y_active,'r.');
    hold on
    %plot(x,ypred1,'b');
    ypred = resubPredict(gprMdl);% not actually needed -- just for testing.
    plot(x_active,ypred,'go');
    xlabel('locations');
    ylabel('measurements');
    legend({'ground truth','Predicted'},...
        'Location','Best');
    title('GP on 10% ground truth data (training)');
    hold off
    resubLoss(gprMdl)
end

fprintf("Initialization is done -- now going to update model with each new observation\n");
%% update the rewards from the GP calculations -- separately for each robot
iter = 1;
for i=1:n
    GW_n{i}.R = zeros(numel(GW_n{i}.States),numel(GW_n{i}.States),numel(GW_n{i}.Actions));
    GW_n{i}.R = update_reward(SD_n{i}, GW_n{i}, unseen_n{i},1);
    rwd = GW_n{i}.R;
    rwd = rwd(myStates_n{i},:,:);
    rwd = rwd(:,myStates_n{i},:);
    R_n{i} = rwd;
    %GW.R(:,state2idx(GW,GW.TerminalStates),:) = 1000;
    
    %% initial MDP solution here from the MDPToolbox
    %[V, policy, iter, cpu_time] = mdp_policy_iteration_modified(GW.T, GW.R, 0.99, 0.01, 1000);
    [policy, iteration, cpu_time] = mdp_value_iteration(trans_n{i}, R_n{i}, 0.99, 0.01, 2000);
    %initPolicy = reshape(policy,[size,size]);
    policy_n{i} = policy;
    fprintf("robot %d I have found the epsilon-optimal policy\n",i);
    %GW_n{i}.R(:,state2idx(GW_n{i},GW_n{i}.CurrentState),:) = penalty;%assign a high penalty for visited cells -- currently set to 0.
    allReward{i} = [allReward{i}, GW_n{i}.R(1,state2idx(GW,GW.CurrentState),1)];% for the initial state
    %allVar{i}(iter) = mean(SD_n{i});%variance at the start
end

%% follow the found optimal policy and store results...
%newPolicy = reshape(policy,[size,size]);
%allStates = reshape(GW.States,[size,size]);

%allReward(iter) = GW.R(1,state2idx(GW,GW.CurrentState),1);% for the initial state
%allVar(iter) = mean(Y_sd);%variance at the start
%GW.R(:,state2idx(GW,GW.CurrentState),:) = 0;

move_flag = 1;
comm_flag = 0;
coord_flag = 0;

% if I have everyone's data available with me, what
% would that GP look like: GP0;
% VtilOracle = composeOracleData(Vtilde_n,n);
% PsiO = sigma^2*eye(size(VtilOracle,1));
% [mO,kO] = posteriorGP(GP0,VtilOracle,PsiO);

%% The main loop -- runs till there is budget left.
reset = zeros(1,n);
while max(budget_n) > 0
    if move_flag==1 % the robots will move in this loop synchronously.
        iter = iter+1;
        for i = 1:1:n
            %allVar{i}(iter+1) = mean(SD_n{i}(indices));%calculate the current VARIANCE for my own partition.
            if budget_n(i) > 0 %&& GW.CurrentState ~= GW.TerminalStates
                %fprintf("budget left: %d\n",budget);
                [row,col] = state2rc(GW_n{i}.CurrentState);
                %best_a =newPolicy(row,col);
                %disp(i);
                best_a = policy_n{i}(myStates_n{i} == state2idx(GW_n{i},GW_n{i}.CurrentState));
                %disp(best_a);
                [newrow,newcol] = action2neighbor(best_a,row,col);
                nextState = append('[',num2str(newrow),',',num2str(newcol),']');
                if newrow <1 || newrow>M || newcol <1 || newcol>M || ~ismember(state2idx(GW_n{i},nextState),myStates_n{i})
                    [newrow,newcol,best_a] = find_greedy_a(row, col, GW_n{i}, M);
                    nextState = append('[',num2str(newrow),',',num2str(newcol),']');
%                     newrow = allPaths{i}(end,1);
%                     newcol = allPaths{i}(end,2);
%                     nextState = append('[',num2str(newrow),',',num2str(newcol),']');
%                     GW_n{i}.CurrentState = nextState;
%                     reset(i) = pUpdate_freq;
%                     continue;
                end
                nextId = state2idx(GW_n{i},nextState);
                %fprintf('original best A: %d',best_a);
                best_a = uncerainA(best_a,myStates_n{i},GW_n{i},pGood,neighbors{state2idx(GW_n{i},GW_n{i}.CurrentState)},nextId);
                %fprintf('After adding uncertainty best A: %d',best_a);
                [newrow,newcol] = action2neighbor(best_a,row,col);
                if newrow <1 || newrow>M || newcol <1 || newcol>M
                    newrow = allPaths{i}(end,1);
                    newcol = allPaths{i}(end,2);
                    nextState = append('[',num2str(newrow),',',num2str(newcol),']');
                    GW_n{i}.CurrentState = nextState;
                    reset(i) = pUpdate_freq;
                    continue;
                end
                nextState = append('[',num2str(newrow),',',num2str(newcol),']');
                %nextId = state2idx(GW_n{i},nextState);
                %fprintf("\n Current state is : %s and best action is: %d",GW.CurrentState, best_a);
                if(abs(newrow+newcol - row-col) > 1)
                    fprintf('something is wrong!!');
                end
                if ismember(state2idx(GW_n{i},nextState),myStates_n{i})
                    GW_n{i}.CurrentState = nextState;
                    budget_n(i) = budget_n(i) - 1;
                    if GW_n{i}.R(1,state2idx(GW,GW.CurrentState),1) ~= -100000
                        allReward{i} = [allReward{i}, GW_n{i}.R(1,state2idx(GW,GW.CurrentState),1)];% for the initial state
                    end
                    reset(i) = reset(i)+1;
                    Xactive_n{i}(numel(Xactive_n{i})+1) = state2idx(GW_n{i},GW_n{i}.CurrentState);%add the new visited state for retraining.
                    %Xactive_n{i} = unique(Xactive_n{i},'stable');%keep unique locations
                    Yactive_n{i}(numel(Yactive_n{i})+1) = Y_gt(Xactive_n{i}(end))+ normrnd(0,sigma);%add the new NOISY measurement for retraining.
                    %unseen = setdiff(X_train,x_active);
                    %                     if numel(intersect(train_locs_n{i}, [newrow, newcol],'rows'))==0
                    %                         train_locs_n{i} = [train_locs_n{i}; newrow newcol];
                    %                     end
                    train_locs_n{i} = [train_locs_n{i}; newrow newcol];
                    noisy_y = Yactive_n{i}(end);
                    Vtilde_n{i} = [Vtilde_n{i}; state2idx(GW_n{i},GW_n{i}.CurrentState) noisy_y newrow newcol];% this data will be shared with the others
                    %y_active = [y_active; Y_gt(state2idx(GW,GW.CurrentState))];
                    unseen_n{i} = setdiff(all_locs,train_locs_n{i},'rows','stable');
                    unseen_idx_n{i} = setdiff(X_train,Xactive_n{i},'stable');
                    allPaths{i} = [allPaths{i}; newrow newcol];
                    allMSE{i} = [allMSE{i}, immse(Pred_n{i},Y_gt(X_train))];%calculate the current RMSE for ALL cells.
                    allVar{i} = [allVar{i}, SD_n{i}];%calculate the current VARIANCE over ALL cells.
                else
                    newrow = allPaths{i}(end,1);
                    newcol = allPaths{i}(end,2);
                    nextState = append('[',num2str(newrow),',',num2str(newcol),']');
                    GW_n{i}.CurrentState = nextState;
                    reset(i) = pUpdate_freq;
                end
                %y_gt = Y_gt(state2idx(GW,GW.CurrentState));%measure the data at the i-th location.
                %update the GP model after every FREQ observations.
            end
        end
        move_flag=0; comm_flag=1;
        
        % if I have everyone's data available with me, what
        % would that GP look like: GP0
        %         VtilOracle = composeOracleData(Vtilde_n,n);
        %         PsiO = sigma^2*eye(size(VtilOracle,1));
        %[mO,kO] = posteriorGP(GP0,VtilOracle,PsiO);
    end
    %% communication happens here.
    if comm_flag == 1 % check if you have neighbors
        %disp('here2',GPP_n{}.Param);
        % create a communication graph.
        source=[]; targ = [];
        for one=1:n
            if budget_n(one) <= 0
                %continue;
            end
            [row1,col1] = state2rc(GW_n{one}.CurrentState);
            for two = 1:n
                if budget_n(two) <= 0
                    %continue;
                end
                if (one~=two)
                    [row2,col2] = state2rc(GW_n{two}.CurrentState);
                    dist = sqrt((row1-row2)^2 + (col1-col2)^2);
                    if dist <= CR %&& budget_n(two) > 0 && budget_n(one) > 0
                        source = [source one];
                        targ = [targ two];
                        coord_flag=1;
                    else
                        source = [source one];
                        targ = [targ one];
                        source = [source two];
                        targ = [targ two];
                    end
                end
            end
        end
        G = graph(source,targ,'omitselfloops');
        comm_flag = 0;
        bins = n;
        if coord_flag == 0
            move_flag = 1;
        else% find the connected sub-graphs if the coordination flag is = 1.
            bins = conncomp(G);
            %disp(bins);
            for i=1:n
                commr_n{i} = [];
                commr_n{i} = find (bins == bins(i));
                %disp( commr_n{i});
%                 if numel(commr_n{i})>1
%                     for j=1:numel(commr_n{i})
%                         if budget_n(commr_n{i}(j))<=0 && commr_n{i}(j)~=i
%                             thisrj = commr_n{i}(j);
%                             commr_n{i} = commr_n{i}(commr_n{i}~=thisrj);
%                         end
%                     end
%                 end
                if numel(commr_n{i})>1
                    posInList(i) = find(commr_n{i} == i);
                end
                %commr_n{i} = commr_n{i}(commr_n{i}~=i);
            end
        end
        allCC = [allCC max(bins)];
        % ESTIMATE MU and SIGMA iff you have no neighbors, otherwise it is
        % anyway going to be updated in the COORDINATION section.
        for i=1:n
            if numel(commr_n{i})<=1 && budget_n(i) > 0
                % do local estimation here for the newObs
                if mod(budget_n(i),update_freq) == 0
                    newObs = Vtilde_n{i}(end,1:2);
                    % I need the mus and sigmas from the last
                    % communicated robots. GHT{i} PST{i}
                    if numel(pStar_n{i})>0 && noMixFlag == 0 % if I have communited w/ SOMEONE before
                        %Psi = sigma^2*eye(numel(VtilCen(:,1))+1);
                        Psi = sigma^2*eye(1,1);
                        %                         if newObs(1,1) > size(GPP_n{i}.Sigma,1)
                        %                             keyboard;
                        %                         end
                        %GP_n{i} = fitrgp(train_locs_n{i},Yactive_n{i},'KernelFunction','exponential','Sigma',sigma0);%using 2D attribute -- (x,y) coordinates
                        %[mOpt,kOpt] = predict(GP_n{i}, all_locs);%predict measurements for the locations with the new model.
                        [mOpt,K] = posteriorGP(GPP_n{i},newObs,Psi); kOpt = diag(K);
                        gHat_n{i}{posInList(i)} = [mOpt kOpt];
                        gStar_n{i} = fusePredictions(gHat_n{i},pStar_n{i});
                        SD_n{i} = gStar_n{i}(:,2);
                        Pred_n{i} = gStar_n{i}(:,1);
                        GW_n{i}.R = update_reward(SD_n{i}, GW_n{i}, unseen_n{i},1);
                        mStar = gStar_n{i}(:,1); kStar = gStar_n{i}(:,2);
                        %SKLD = 0.5*(log(kO./kStar) + 0.5*(kStar + (mStar-mO).^2)./kO - 0.5) + ...
                        %0.5*(log(kStar./kO) + 0.5*(kO + (mO-mStar).^2)./kStar - 0.5);
                        GPP_n{i}.Sigma = K;
                        GPP_n{i}.Mu = mOpt;
                    else
                        Psi = sigma^2*eye(1,1);
                        %GP_n{i} = fitrgp(train_locs_n{i},Yactive_n{i},'KernelFunction','exponential','Sigma',sigma0);%using 2D attribute -- (x,y) coordinates
                        %[mZero,kzd] = predict(GP_n{i}, all_locs);%predict measurements for the locations with the new model.
                        [mZero,kZero] = posteriorGP(GPP_n{i},newObs,Psi); kzd = diag(kZero);
                        Pred_n{i} = mZero; SD_n{i} = kzd;
                        GW_n{i}.R = update_reward(SD_n{i}, GW_n{i}, unseen_n{i},1);
                        %SKLD = 0.5*(log(kO./kZero) + 0.5*(kZero + (mZero-mO).^2)./kO - 0.5) + ...
                        %0.5*(log(kZero./kO) + 0.5*(kO + (mO-mZero).^2)./kZero - 0.5);
                        GPP_n{i}.Sigma = kZero;
                        GPP_n{i}.Mu = mZero;
                    end
                    %allMSE{i}(iter) = (immse(Pred_n{i},Y_gt(X_train)));%calculate the current RMSE for ALL cells.
                    %allSKLD{i} = [allSKLD{i}, SKLD];
                end
                fprintf("Robot %d has updated its GP model. Budget Left: %d\n",i,budget_n(i));
                
                %update the rewards after every FREQ observations.
                if mod(reset(i),pUpdate_freq) == 0
                    %GW.T(:,state2idx(GW,GW.ObstacleStates),:) = 0;
                    %GW_n{i}.R = update_reward(SD_n{i},GW_n{i}, unseen_n{i},1);%update the rewards accordingly.
                    rwd = GW_n{i}.R;
                    rwd = rwd(myStates_n{i},:,:);
                    rwd = rwd(:,myStates_n{i},:);
                    R_n{i} = rwd;
                    %[V, policy, iteration, cpu_time] = mdp_policy_iteration_modified(GW.T, GW.R, 0.99, 0.01, 10000);
                    [policy, iteration, cpu_time] = mdp_value_iteration(trans_n{i}, R_n{i}, 0.99, 0.01, 2000);
                    policy_n{i} = policy;
                    fprintf("LOCAL UPDATE R%d has found optimal policy; budget left: %d\n",i,budget_n(i));
                    %newPolicy = reshape(policy,[size,size]);
                    reset(i) = 0;
                end
                %[sharedvals,indices] = intersect(unseen_idx_n{i},myStates_n{i},'stable');
                %allMSE{i}(iter) =
                %sqrt(immse(Pred_n{i},Y_gt(unseen_idx_n{i})));%calculate
                %the current RMSE for ALL UNSEEN cells.
                %allMSE{i}(iter) = sqrt(immse(Pred_n{i}(indices),Y_gt(unseen_idx_n{i}(indices))));%calculate the current RMSE for my own partition.
            end
        end
    end
    %% coordination happens here.
    if coord_flag ==1 % you have found neighbors -- now it's time for coordination via Pat's code.
        %% communication + mixture happens here...
        % share local meaurements, and then calculate likelihoods using Pat's logni's, and finally share the likelihoods.
        % assume that you have stored the received data in VtilAll -- using
        % Pat's code here (should be done inside MATLAB)...
        fprintf('\n communication + mixture happens here...');
        for i = 1:n
            %disp('here3',GPP_n{1}.Param);
            if numel(commr_n{i})>1%only if you have neighbors to talk to
                [gStar,Khat_com,pStar,gHat,msgs] = communication(Vtilde_n,i,commr_n{i},GPP_n,M,N,noMixFlag,myStates2d_n);
                msgcount = msgs+msgcount;
%                 if noMixFlag == 0
%                     Khat_n{i} = Khat_com{1};
%                     pStar_n{i} = pStar;
%                     GPupdateflag_n(i) = 0;
%                     SD_n{i} = gStar_n{i}(:,2);
%                     GPP_n{i}.Sigma = Khat_n{i};
%                     GPP_n{i}.Mu = gStar_n{i}(:,1);
%                     gHat_n{i} = gHat;
%                 else
%                     gHat_n{i} = gHat;
%                     
%                 end
%                 GW_n{i}.R = update_reward(gStar_n{i}(:,2),GW_n{i}, unseen_n{i},1);%update the rewards accordingly.
%                 rwd = GW_n{i}.R;
%                 rwd = rwd(myStates_n{i},:,:);
%                 rwd = rwd(:,myStates_n{i},:);
%                 R_n{i} = rwd;
%                 %[V, policy, iteration, cpu_time] = mdp_policy_iteration_modified(GW.T, GW.R, 0.99, 0.01, 10000);
%                 [policy_n{i}, iteration, cpu_time] = mdp_value_iteration(trans_n{i}, R_n{i}, 0.99, 0.01, 2000);
                for j = 1:numel(commr_n{i})
                    if noMixFlag == 0
                        gStar_n{commr_n{i}(j)} = gStar;
                        Khat_n{commr_n{i}(j)} = Khat_com{j};
                        GPP_n{commr_n{i}(j)}.Sigma = Khat_com{j};
                        GPP_n{commr_n{i}(j)}.Mu = gStar(:,1);
                        pStar_n{commr_n{i}(j)} = pStar;
                        GPupdateflag_n(commr_n{i}(j)) = 0;
                        SD_n{commr_n{i}(j)} = gStar(:,2);
                        gHat_n{commr_n{i}(j)} = gHat;
                        Pred_n{commr_n{i}(j)} = GPP_n{commr_n{i}(j)}.Mu;
                    else
                        gHat_n{commr_n{i}(j)} = gHat;
                    end
                    %allMSE{commr_n{i}(j)}(iter) = (immse(GPP_n{commr_n{i}(j)}.Mu,Y_gt(X_train)));%calculate the current RMSE for ALL cells.
                    %update the rewards accordingly and find a new
                    %policy.
                    GW_n{commr_n{i}(j)}.R = update_reward(SD_n{commr_n{i}(j)},GW_n{commr_n{i}(j)}, unseen_n{commr_n{i}(j)},1);%update the rewards accordingly.
                    rwd = GW_n{commr_n{i}(j)}.R;
                    rwd = rwd(myStates_n{commr_n{i}(j)},:,:);
                    rwd = rwd(:,myStates_n{commr_n{i}(j)},:);
                    R_n{commr_n{i}(j)} = rwd;
                    %                     %[V, policy, iteration, cpu_time] = mdp_policy_iteration_modified(GW.T, GW.R, 0.99, 0.01, 10000);
                    [policy_n{commr_n{i}(j)}, iteration, cpu_time] = mdp_value_iteration(trans_n{commr_n{i}(j)}, R_n{commr_n{i}(j)}, 0.99, 0.01, 2000);
                    fprintf("After COORD Robot %d has found a new epsilon-optimal policy after MIXTURE and budget left: %d\n",commr_n{i}(j),budget_n(commr_n{i}(j)));
                    %commr_n{commr_n{i}(j)} = [];
                    reset(commr_n{i}(j)) = 0;
                end
                
                for j = 2:numel(commr_n{i})
                    commr_n{commr_n{i}(j)} = [];
                end
                commr_n{i} = [];
            end
            % For path planning from here, treat
            %   gStar(:,1) as the posterior mean of all M*N cells
            %   gStar(:,2) as the posterior variance of all M*N cells
            % Note: if robot i is constrained to the cells in region V{i}, then it is
            %       sufficient to compute only gStar(V{i},:) (and before that only
            %       pHat(V{i},:) accordingly), but we did not exercise such savings in
            %       computationl here
            
            % for future observations: pull out M and k-hats for ID and do noisy
            % observation and do posterior GP update. and calculate gStar again -- line 93.
        end
        comm_flag = 0;coord_flag=0;move_flag=1;
    end
end
%hold off;
%fprintf("Run time is %f Minutes with an average reward per iteration: %f.", (toc/60), (allReward(iter-1)/(iter-1)));
%sim(agent,env);
%cumulativeReward = sum(data.Reward);
timeSec = toc;
%% plotting data at the end
titleS = 'Dec-MDP-OC';
if allCommFlag ==1
    titleS = 'Dec-MDP-CC';
end
if noCommFlag ==1
    titleS = 'Dec-MDP-NC';
end

if 0
%     figure();
%     for i=1:n
%         plot(allMSE{i}(1:end));
%         hold all;
%     end
%     hold off;
%     title(titleS);
%     ylabel('MSE','FontSize',14);
%     xlabel('Path length','FontSize',14);
    
    figure();
    avgMSE = (sum(cell2mat(allMSE')))/n ;
    plot(avgMSE);
    title(titleS);
    ylabel('Average MSE','FontSize',14);
    xlabel('Path length','FontSize',14);
    %
    % figure(3);
    % for i=1:n
    %     plot(allReward{i}(1:end));
    %     hold all;
    % end
    % hold off;
    % title('MDP-MIPP');
    % ylabel('Entropy','FontSize',14);
    % xlabel('Path length','FontSize',14);
    
    
%     figure();
%     plot(allCC);
%     ylim([0 n+1]);
%     title(titleS);
%     ylabel('#Connected Components','FontSize',14);
%     xlabel('Path length','FontSize',14);
    
    figure();
    sumEnt = 0;
    for i=1:n
        sumEnt = sumEnt + sum(allReward{i});
    end
    bar(sumEnt);
    title(titleS);
    ylabel('Total Entropy','FontSize',14);
    xlabel('Dec-MDP','FontSize',14);
    
%     figure();
%     for i=1:n
%         plot(allPaths{i}(:,2),allPaths{i}(:,1), 'o-');
%         hold all;
%     end
%     hold off;
%     xlim([0 M+1]);
%     ylim([0 M+1]);
%     title(titleS);
%     ylabel('Latitude','FontSize',14);
%     xlabel('Longitude','FontSize',14);
    
    figure();
    subplot(1,2,1);
    surf(Z_gt,'EdgeColor','None'); view(2);
    title('Ground Truth');
    ylabel('Latitude','FontSize',12);
    xlabel('Longitude','FontSize',12);
    %calculate the average model and plot.
    subplot(1,2,2);
    A = Pred_n;
    as = size(A,2);
    matSize = size(A{1},1);
    B = reshape(cell2mat(A),matSize,[],as);
    C = sum(B,3);
    C = C/n;
    surf(reshape(C,[M,M]),'EdgeColor','None'); view(2);
    title('Predicted Model');
    ylabel('Latitude','FontSize',12);
    xlabel('Longitude','FontSize',12);
    sgtitle(titleS);
    filename = strcat(titleS,num2str(n),'_GTvsPred.png');
    %saveas(gcf,filename);
    
    figure();
    subplot(1,2,1);
    SD_init = zeros(M*M,1);
    for i=1:n
        SD_init = plus(SD_init,allVar{i}(:,1));
    end
    SD_init = reshape(SD_init/n,[M,M]);
    %SD_init = reshape(allVar{1}(:,1),[M,M]);
    surf(SD_init,'EdgeColor','None'); %view(2);
    title('Initial Variance');
    %ylabel('Latitude','FontSize',12);
    %xlabel('Longitude','FontSize',12);
    zlabel('Variance','FontSize',14);
    %calculate the average model and plot.
    subplot(1,2,2);
    SD_final = zeros(M*M,1);
    for i=1:n
        SD_final = plus(SD_final,allVar{i}(:,end));
    end
    SD_final = reshape(SD_final/n,[M,M]);
    surf(SD_final,'EdgeColor','None'); %view(2);
    title('Final Variance');
    %ylabel('Latitude','FontSize',12);
    %xlabel('Longitude','FontSize',12);
    sgtitle(titleS);
    %colorbar;
    filename = strcat(titleS,num2str(n),'_SDinitvFinal.png');
    %saveas(gcf,filename);
    %
    % figure();
    % for i=1:n
    %     plot(allVar{i}(end));
    %     hold all;
    % end
    % hold off;
    % title('MDP-MIPP');
    % ylabel('Variance','FontSize',14);
    % xlabel('Path length','FontSize',14);
end
str = strcat('Results\',titleS,num2str(n),'_run_',num2str(run),'.mat');
save (str, 'allMSE','allReward','allVar','timeSec', 'allPaths', 'allCC','Pred_n','msgcount');

%end % END of robot count data collection
end



