%% variable initialization
size = 20;
budget = 38;
update_freq = 1;
allMSE = zeros(1, budget);
allReward = zeros(1, budget);
GW = createGridWorld(size,size);
id = 1;

if id == 1
    GW.CurrentState = '[1,1]';
else
    GW.CurrentState = '[1,4]';
end
GW.TerminalStates = idx2state(GW,size*size);
tic
env = rlMDPEnv(GW);
%env2 = rlMDPEnv(GW2);
qTable = rlTable(getObservationInfo(env),getActionInfo(env));
critic = rlQValueRepresentation(qTable,getObservationInfo(env),getActionInfo(env));
opt = rlQAgentOptions;
agent = rlQAgent(critic,opt);
%plot(env);%plot(env2);

northStateTransition = GW.T(:,:,1);
southStateTransition = GW.T(:,:,1);
eastStateTransition = GW.T(:,:,1);
westStateTransition = GW.T(:,:,1);

%% GP stuff here
data_gt = data_import('dataFile_ell25_50by50.csv', 1, 50);
data_gt = data_gt(1:size,1:size);
Y_gt = str2double(reshape(table2array(data_gt),[size*size 1]));
X_train = (1:size*size)';
x_active = randsample(X_train,0.2*size*size);%select 10% random points to initialize GP hyperparameters.
y_active = Y_gt(x_active);
%x_active = X_train;
%y_active = data_gt_copy;
sigma0 = std(y_active);
kparams0 = [3.5, 0.25];
unseen = setdiff(X_train,x_active);
gprMdl = fitrgp(x_active,y_active,'KernelFunction','squaredexponential',...
     'KernelParameters',kparams0,'Sigma',sigma0);
ypred = resubPredict(gprMdl);% not actually needed -- just for testing.

% for online entropy calculation - Entropy(Z_A) w/o any conditionals
kfcn = gprMdl.Impl.Kernel.makeKernelAsFunctionOfXNXM(gprMdl.Impl.ThetaHat);
cov_mat = kfcn(x_active(:,:),x_active(:,:));%full covariance matrix.
U_size = numel(unseen);%number of unseen locs
%H_Za = 0.5 * log(((2*pi*exp(1))^U_size)* det(cov_mat)); %Liu entropy for online measurement
H_za = 0.5 * log (det(cov_mat)) + ((size*size)/2) * (1 + log(2*pi)); % Wei, Zheng (2020)

% figure(1);
% plot(x_active,y_active,'r.');
% hold on
% %plot(x,ypred1,'b');
% plot(x_active,ypred,'g');
% xlabel('locations');
% ylabel('measurements');
% legend({'ground truth','Predicted'},...
% 'Location','Best');
% title('GP on 10% ground truth data (training)');
% hold off

fprintf("Initialization is done -- now going to update model with each new observation\n");


% for i= 1:1:budget
y_gt = Y_gt(state2idx(GW,GW.CurrentState));%measure the data at the i-th location.
x_active(numel(x_active)+1) = state2idx(GW,GW.CurrentState);%add the new visited state for retraining.
x_active = unique(x_active,'stable');%keep unique locations
y_active = Y_gt(x_active);%add the new measurement for retraining.
unseen = setdiff(X_train,x_active);
gprMdl = update_GP_model(gprMdl,x_active, y_active);%update the current GP model.
[Y_pred,Y_sd] = predict(gprMdl, unseen);%predict for the UNSEEN locations with the new model.

%% update the rewards from the GP calculations
nS = numel(GW.States);
nA = numel(GW.Actions);
GW.R = zeros(nS,nS,nA);
GW.R = update_reward(gprMdl, Y_pred, Y_sd, x_active, GW, size, unseen);
%GW.R = -1*ones(nS,nS,nA);
%GW.TerminalStates = '[30,30]';
GW.R(:,state2idx(GW,GW.TerminalStates),:) = 1000;

%% follow the policy
plot(env);
env.Model.Viewer.ShowTrace = true;
env.Model.Viewer.clearTrace;

%newPolicy = reshape(policy,[size,size]);
allStates = reshape(GW.States,[size,size]);
iter = 1;
allReward(iter) = GW.R(1,state2idx(GW,GW.CurrentState),1);% for the initial state
GW.R(:,state2idx(GW,GW.CurrentState),:) = 0;


%the main loop
while budget > 0 && GW.CurrentState ~= GW.TerminalStates
    fprintf("budget left: %d\n",budget);
    current_state_id = erase(GW.CurrentState,"[");
    current_state_id = erase(current_state_id,"]");
    current_state_id = strsplit(current_state_id,',');
    row = str2double(current_state_id{1,1});
    col = str2double(current_state_id{1,2});
    [newrow, newcol] = find_greedy_a(row, col, GW, size);%returns the BEST neighbor -- greedy approach.
    
    GW.CurrentState = append('[',num2str(newrow),',',num2str(newcol),']');
    %y_gt = Y_gt(state2idx(GW,GW.CurrentState));%measure the data at the i-th location.
    x_active(numel(x_active)+1) = state2idx(GW,GW.CurrentState);%add the new visited state for retraining.
    x_active = unique(x_active,'stable');%keep unique locations
    y_active = Y_gt(x_active);%add the new measurement for retraining.
    unseen = setdiff(X_train,x_active);
    if GW.CurrentState ~= GW.TerminalStates
        allReward(iter+1) = allReward(iter) + GW.R(1,state2idx(GW,GW.CurrentState),1);
        %disp(allReward(iter));
        GW.R(:,state2idx(GW,GW.CurrentState),:) = -1;%assign a high penalty for visited cells.
    end
    %update the GP model after every FREQ observations.
    if mod(budget,update_freq) == 0
        gprMdl = update_GP_model(gprMdl,x_active, y_active);%update the current GP model.
    end
    [Y_pred,Y_sd] = predict(gprMdl, unseen);%predict for the UNSEEN locations with the new model.
    allMSE(iter) = sqrt(immse(Y_pred,Y_gt(unseen)));%calculate the current RMSE
    iter = iter+1;
    budget = budget - 1;
    plot(env);
    pause(0.02);
end
%hold off;
fprintf("Run time for Budget %d is %f seconds with an average reward per iteration: %f.",budget+1, toc, (allReward(iter-1)/(iter-1)));
%data = sim(agent,env);
%cumulativeReward = sum(data.Reward);

figure(3);
plot(allMSE(1:end));
ylabel('RMSE','FontSize',12);

figure(4);
plot(allReward(1:end));
ylabel('Reward','FontSize',12);
