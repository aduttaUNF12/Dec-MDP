%parfor id = 1:2

%% variable initialization
size = 32;% square environment: size x size
[x,y] = meshgrid(1:1:size,1:1:size);
all_locs = [y(:), x(:)];
unseen = all_locs;% initially every location is unseen.
update_freq = 10;
penalty = 0;% for visiting an already observed cell.
max_trn_data = 0.45;% percentage of all nodes in my partition.
init_trn_data = 0.2;% percentage of all nodes in the environment.
initDataPlot = 0;
pGood = 0.8;
pBad = 0.2;

%% pre-processing -- K-means clustering for partitioining the region.
id = 1;
[idx,centr] = find_partitions(2,size,0);% params: (robot#,size,display_flag)

%% create a GRID world
GW = createGridWorld(size,size);
newrow = centr(id,1); newcol = centr(id,2);
GW.CurrentState = append('[',num2str(newrow),',',num2str(newcol),']');
myStates = find(idx == id);% only adding my partition's cells to my state-space.
myStates2d = all_locs(myStates,:);
otherRobotStates = find(idx ~= id);% other robots' states are obstacles to me -- will be treated accordingly.
budget = ceil(max_trn_data*numel(myStates));% budget parameter
allMSE = zeros(1, budget);
allReward = zeros(1, budget);
allVar = zeros(1, budget);
% if id == 1
%     GW.CurrentState = '[2,2]';
% else
%     GW.CurrentState = '[2,3]';
% end
%GW.TerminalStates = idx2state(GW,size*size);
env = rlMDPEnv(GW);
%env2 = rlMDPEnv(GW2);
% qTable = rlTable(getObservationInfo(env),getActionInfo(env));
% critic = rlQValueRepresentation(qTable,getObservationInfo(env),getActionInfo(env));
% opt = rlQAgentOptions;
% agent = rlQAgent(critic,opt);
%plot(env);%plot(env2);

%% padded with obstacles to avoid going out of the arena.
obstacles = [];
lr = size;fr = 1;lc = size*size;
for ind = 1:1:size
    fc=ind;
    %GW.T(fr,:,1) = 0;% taking action N in the top row is prohibited.
    obstacles = [obstacles idx2state(GW,fc)];
    %GW.T(lr,:,2) = 0;% taking action S in the bottom row is prohibited.
    obstacles = [obstacles idx2state(GW,lr)];
    lr = lr + size;
    %GW.T(fc,:,4) = 0;% taking action W in the left col is prohibited.
    obstacles = [obstacles idx2state(GW,lc)];
    lc = lc-1;
    %GW.T(lc,:,3) = 0;% taking action E in the right col is prohibited.
    obstacles = [obstacles idx2state(GW,fr)];
    fr = fr+size;
end
otherRobotStates = otherRobotStates';
obstacles = [obstacles (idx2state(GW,otherRobotStates))'];% other robots' states are added as obstacles to avoid going in there...
GW.ObstacleStates = unique(obstacles,'stable');
%updateStateTranstionForObstacles(GW);
GW.T = customTransition(GW.T,size, pGood, pBad);% update the transition (s,s',a) matrix
%northStateTransition = GW.T(:,:,1);
%southStateTransition = GW.T(:,:,2);
%eastStateTransition = GW.T(:,:,3);
%westStateTransition = GW.T(:,:,4);
trans = GW.T;
trans = trans(myStates,:,:);
trans = trans(:,myStates,:);% extract the transition sub-matrix for my partition.


%% Read the ground-truth data from file & initialize GP
tic
data_gt = data_import('dataFile_ell25_50by50.csv', 1, 50);
data_gt = data_gt(1:size,1:size);
Y_gt = str2double(reshape(table2array(data_gt),[size*size 1]));
X_train = (1:size*size)';
x_active = randsample(X_train,ceil(init_trn_data*size*size));%select 10% random points to initialize GP hyperparameters.
init_training_locs = all_locs(x_active,:);% initial locs to initialize the hyperparameters.
y_active = Y_gt(x_active);% meausurements of the initial training locs.
unseen = setdiff(unseen,init_training_locs,'rows');% unseen locations in the environment (includes other robot states).
unseen_idx = setdiff(X_train,x_active);

sigma0 = std(y_active);
%kparams0 = [3.5, 0.25];
%gprMdl = fitrgp(init_training_locs,y_active,'KernelFunction','squaredexponential','Sigma',sigma0);%using 2D attribute -- (x,y) coordinates
%ypred = resubPredict(gprMdl);% not actually needed -- just for testing.

%% add the starting location to the training matrix and do the 1st prediction
y_gt = Y_gt(state2idx(GW,GW.CurrentState));%measure the data at the i-th location.
[currrow,currcol] = state2rc(GW.CurrentState);
%newData = {currrow,currcol,y_gt};
%attributes = [attributes;newData];
x_active(numel(x_active)+1) = state2idx(GW,GW.CurrentState);%add the new visited state for retraining.
x_active = unique(x_active,'stable');%keep unique locations
y_active = Y_gt(x_active);%add the new measurement for retraining.
train_locs = init_training_locs;
train_locs = unique(train_locs,'rows','stable');
if numel(intersect(train_locs, [currrow,currcol],'rows'))==0
    train_locs = [train_locs; currrow currcol];
end
%y_active = [y_active; y_gt];
unseen = setdiff(all_locs,train_locs,'rows','stable');%setdiff(unseen,[currrow,currcol],'rows');
unseen_idx = setdiff(X_train,x_active,'stable');
%gprMdl = update_GP_model(gprMdl,x_active, y_active);%update the current GP model.
gprMdl = fitrgp(train_locs,y_active,'KernelFunction','exponential','Sigma',sigma0);%using 2D attribute -- (x,y) coordinates
[Y_pred,Y_sd] = predict(gprMdl, unseen);%predict for the UNSEEN locations with the new model.


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

%% plot the environment and the followed path..
plot(env);
env.Model.Viewer.ShowTrace = true;
env.Model.Viewer.clearTrace;

fprintf("Initialization is done -- now going to update model with each new observation\n");


%% update the rewards from the GP calculations
GW.R = zeros(numel(GW.States),numel(GW.States),numel(GW.Actions));
GW.R = update_reward(gprMdl, Y_pred, Y_sd, x_active, GW, size, unseen);
rwd = GW.R;
rwd = rwd(myStates,:,:);
rwd = rwd(:,myStates,:);
%GW.R(:,state2idx(GW,GW.TerminalStates),:) = 1000;

%% initial MDP solution here from the MDPToolbox
%[V, policy, iter, cpu_time] = mdp_policy_iteration_modified(GW.T, GW.R, 0.99, 0.01, 1000);
[policy, iteration, cpu_time] = mdp_value_iteration(trans, rwd, 0.99, 0.01, 2000);
%initPolicy = reshape(policy,[size,size]);
fprintf("I have found the epsilon-optimal policy\n");

%% follow the found optimal policy and store results...
%newPolicy = reshape(policy,[size,size]);
%allStates = reshape(GW.States,[size,size]);
iter = 1;
allReward(iter) = GW.R(1,state2idx(GW,GW.CurrentState),1);% for the initial state
allVar(iter) = mean(Y_sd);%variance at the start
%GW.R(:,state2idx(GW,GW.CurrentState),:) = 0;
GW.R(:,state2idx(GW,GW.CurrentState),:) = penalty;%assign a high penalty for visited cells -- currently set to 0.


%% The main loop -- runs till there is budget left.
reset_counter = 0;
while budget > 0 %&& GW.CurrentState ~= GW.TerminalStates
    %fprintf("budget left: %d\n",budget);
    [row,col] = state2rc(GW.CurrentState);
    %best_a =newPolicy(row,col);
    best_a = policy(myStates == state2idx(GW,GW.CurrentState));
    [newrow,newcol] = action2neighbor(best_a,row,col);
    %fprintf("\n Current state is : %s and best action is: %d",GW.CurrentState, best_a);
    nextState = append('[',num2str(newrow),',',num2str(newcol),']');
    if state2idx(GW,nextState)~=x_active(end) % if the policy is NOT asking you to move to the last visited cell
        GW.CurrentState = nextState;
    else % if the policy IS asking you to move to the last visited cell
        %GW.T(:,state2idx(GW,GW.ObstacleStates),:) = 0;
        GW.R = update_reward(gprMdl, Y_pred, Y_sd,x_active, GW, size, unseen);%update the rewards accordingly.
        %[V, policy, iteration, cpu_time] = mdp_policy_iteration_modified(GW.T, GW.R, 0.99, 0.01, 10000);
        rwd = GW.R;
        rwd = rwd(myStates,:,:);
        rwd = rwd(:,myStates,:);
        [policy, iteration, cpu_time] = mdp_value_iteration(trans, rwd, 0.99, 0.01, 2000);
        reset_counter = 0;
        fprintf("Robot %d has found a new epsilon-optimal policy as it was going BACK. Budget Left: %d\n",id,budget);
        best_a = policy(myStates == state2idx(GW,GW.CurrentState));
        %fprintf("\n Current state is : %s and best action is: %d",GW.CurrentState, best_a);
        [newrow,newcol] = action2neighbor(best_a,row,col);
        nextState = append('[',num2str(newrow),',',num2str(newcol),']');
        GW.CurrentState = nextState;
    end
    %y_gt = Y_gt(state2idx(GW,GW.CurrentState));%measure the data at the i-th location.
    x_active(numel(x_active)+1) = state2idx(GW,GW.CurrentState);%add the new visited state for retraining.
    x_active = unique(x_active,'stable');%keep unique locations
    y_active = Y_gt(x_active);%add the new measurement for retraining.
    %unseen = setdiff(X_train,x_active);
    if numel(intersect(train_locs, [newrow, newcol],'rows'))==0
        train_locs = [train_locs; newrow newcol];
    end
    %y_active = [y_active; Y_gt(state2idx(GW,GW.CurrentState))];
    unseen = setdiff(all_locs,train_locs,'rows','stable');
    unseen_idx = setdiff(X_train,x_active,'stable');
    
    %if GW.CurrentState ~= GW.TerminalStates
    if GW.R(1,state2idx(GW,GW.CurrentState),1) ~= -100000
        allReward(iter+1) = allReward(iter) + GW.R(1,state2idx(GW,GW.CurrentState),1);
    else
        allReward(iter+1) = allReward(iter);
    end
    GW.R(:,state2idx(GW,GW.CurrentState),:) = penalty;%assign a high penalty for visited cells -- currently set to 0.
    %disp(allReward(iter));
    reset_counter = reset_counter+1;
    %update the GP model after every FREQ observations.
    if mod(budget,update_freq) == 0
        %gprMdl = update_GP_model(gprMdl,x_active, y_active);%update the current GP model.
        sigma0 = std(y_active);
        gprMdl = fitrgp(train_locs,y_active,'KernelFunction','exponential','Sigma',sigma0);
        reset_counter = 0;
        fprintf("Robot %d has updated its GP model. Budget Left: %d\n",id,budget);
    end
    [Y_pred,Y_sd] = predict(gprMdl, unseen);%predict for the UNSEEN locations with the new model.
    %update the rewards after every FREQ observations.
    if reset_counter==update_freq
        %GW.T(:,state2idx(GW,GW.ObstacleStates),:) = 0;
        GW.R = update_reward(gprMdl, Y_pred, Y_sd,x_active, GW, size, unseen);%update the rewards accordingly.
        rwd = GW.R;
        rwd = rwd(myStates,:,:);
        rwd = rwd(:,myStates,:);
        %[V, policy, iteration, cpu_time] = mdp_policy_iteration_modified(GW.T, GW.R, 0.99, 0.01, 10000);
        [policy, iteration, cpu_time] = mdp_value_iteration(trans, rwd, 0.99, 0.01, 2000);
        fprintf("Robot %d has found a new epsilon-optimal policy and budget left: %d\n",id,budget);
        %newPolicy = reshape(policy,[size,size]);
        reset_counter = 0;
    end
    [sharedvals,indices] = intersect(unseen_idx,myStates,'stable');
    %allMSE(iter) = sqrt(immse(Y_pred,Y_gt(unseen_idx)));%calculate the current RMSE
    allMSE(iter) = sqrt(immse(Y_pred(idx),Y_gt(unseen_idx(idx))));%calculate the current RMSE for my own partition.
    allVar(iter+1) = mean(Y_sd(idx));%calculate the current VARIANCE for my own partition.
    iter = iter+1;
    budget = budget - 1;
    %plot(env);
    %pause(0.02);
end
%hold off;
fprintf("Run time is %f Minutes with an average reward per iteration: %f.", (toc/60), (allReward(iter-1)/(iter-1)));
%sim(agent,env);
%cumulativeReward = sum(data.Reward);

%% plotting data at the end
figure(3);
plot(allMSE(1:end));
%title('id='+id);
ylabel('RMSE','FontSize',12);
xlabel('Path length','FontSize',12);

figure(4);
plot(allReward(1:end));
%title('id='+id);
ylabel('Entropy','FontSize',12);
xlabel('Path length','FontSize',12);

figure(5);
plot(allVar(1:end));
%title('id='+id);
ylabel('Posterior Variance','FontSize',12);
xlabel('Path length','FontSize',12);
%end



