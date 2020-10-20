% MATLAB controller for Webots
% File:          dec_mdp.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;

TIME_STEP = 64;



%parfor id = 1:2
emitter = wb_robot_get_device('emitter');
wb_emitter_set_range(emitter, -1);
receiver = wb_robot_get_device('receiver');
wb_receiver_enable(receiver, TIME_STEP);
%data = [num2str(0) ',' num2str(0) ];
id = str2num(wb_robot_get_name());


%% variable initialization
size = 12;% square environment: size x size
[x,y] = meshgrid(1:1:size,1:1:size);
all_locs = [y(:), x(:)];
unseen = all_locs;% initially every location is unseen.
update_freq = 10;
comm_freq = 5;
penalty = 0;% for visiting an already observed cell.
max_trn_data = 0.45;% percentage of all nodes in my partition.
init_trn_data = 0.2;% percentage of all nodes in the environment.
initDataPlot = 0;
pGood = 0.8;
pBad = 0.2;
comm_robots=[]; comm_robots = [id, comm_robots];
num_robot = 2;
CR = floor(size * sqrt(2) * 0.5);

%% for Pat's code
M=size; N=size;
alpha = 1.00e-03;
sigma = 0.500;
toler = 2.00e-04;
Vtilde = [];
comm_flag = 0;%will be flipped to 1 if communication happening
done = 0;

%% pre-processing -- K-means clustering for partitioining the region.
[idx,centr] = find_partitions(num_robot,size,0);% params: (robot#,size,display_flag)

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
y_active = Y_gt(x_active)+normrnd(0,sigma);% meausurements of the initial training locs.
unseen = setdiff(unseen,init_training_locs,'rows');% unseen locations in the environment (includes other robot states).
unseen_idx = setdiff(X_train,x_active);

sigma0 = std(y_active);
%kparams0 = [3.5, 0.25];
%gprMdl = fitrgp(init_training_locs,y_active,'KernelFunction','squaredexponential','Sigma',sigma0);%using 2D attribute -- (x,y) coordinates
%ypred = resubPredict(gprMdl);% not actually needed -- just for testing.

%% add the starting location to the training matrix and do the 1st prediction
y_gt = Y_gt(state2idx(GW,GW.CurrentState))+normrnd(0,sigma);%measure the data at the i-th location.
[currrow,currcol] = state2rc(GW.CurrentState);
%newData = {currrow,currcol,y_gt};
%attributes = [attributes;newData];
x_active(numel(x_active)+1) = state2idx(GW,GW.CurrentState);%add the new visited state for retraining.
x_active = unique(x_active,'stable');%keep unique locations
y_active = Y_gt(x_active)+normrnd(0,sigma);%add the new measurement for retraining.
train_locs = init_training_locs;
train_locs = unique(train_locs,'rows','stable');
if numel(intersect(train_locs, [currrow,currcol],'rows'))==0
    train_locs = [train_locs; currrow currcol];
end
Vtilde = [x_active y_active train_locs];
%new = [76 0 5 5];
%shared_data = [shared_data; 76 0 5 5];
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

while wb_robot_step(TIME_STEP) ~= -1
    %% The main loop -- runs till there is budget left.
    reset_counter = 0;
    while budget > 0 %&& GW.CurrentState ~= GW.TerminalStates
        wb_console_print('Another round...', WB_STDOUT );
        if comm_flag==0
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
            y_active = Y_gt(x_active)+normrnd(0,sigma);%add the new measurement for retraining.
            %unseen = setdiff(X_train,x_active);
            if numel(intersect(train_locs, [newrow, newcol],'rows'))==0
                train_locs = [train_locs; newrow newcol];
            end
            Vtilde = [Vtilde; state2idx(GW,GW.CurrentState) Y_gt(state2idx(GW,GW.CurrentState))+normrnd(0,sigma) newrow newcol];% this data will be shared with the others
            %y_active = [y_active; Y_gt(state2idx(GW,GW.CurrentState))];
            unseen = setdiff(all_locs,train_locs,'rows','stable');
            unseen_idx = setdiff(X_train,x_active,'stable');
            data = [num2str(id) ',' num2str(newrow) ',' num2str(newcol) ];
            if mod(budget,comm_freq) == 0
                % TO-Do Webots comm here to check if I have any neighbors.
                % update comm_robots
                wb_emitter_send(emitter, double(data));
                while wb_robot_step(TIME_STEP) ~= -1 && done == 0
                    queue_length = wb_receiver_get_queue_length(receiver);
                    if queue_length > 0 % message detected
                        %% get the next message
                        msg = wb_receiver_get_data(receiver, 'double');
                        msg = char(msg)'; % convert message back to string
                        wb_receiver_next_packet(receiver);
                        disp([num2str(id), 'received: ', msg]);
                    end
                    done = 1;
                end
                msg = regexp(msg, ',', 'split');
                neigh_row = str2double(msg{2}); neigh_col=str2double(msg{3});
                dist = sqrt((newrow-neigh_row)^2 + (newcol-neigh_col)^2);
                if (dist <= CR) comm_robots = [comm_robots, msg{1}];
                    if length(comm_robots) > 1
                        comm_flag=1;
                        %continue;
                    end
                end
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
                    gprMdl = fitrgp(train_locs,y_active,'KernelFunction','exponential','Sigma',sigma0);% MATLAB's method, not Kreidl's
                    reset_counter = 0;
                    phi = gprMdl.KernelInformation.KernelParameters;
                    GP = generateGP(M,N,phi);% using pat's code.
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
                allMSE(iter) = sqrt(immse(Y_pred(indices),Y_gt(unseen_idx(indices))));%calculate the current RMSE for my own partition.
                allVar(iter+1) = mean(Y_sd(indices));%calculate the current VARIANCE for my own partition.
                iter = iter+1;
                budget = budget - 1;
                %plot(env);
                %pause(0.02);
            else
                %% communication + mixture happens here...
                % share local meaurements, and then calculate likelihoods using Pat's logni's, and finally share the likelihoods.
                % assume that you have stored the received data in VtilAll -- using
                % Pat's code here (should be done inside MATLAB)... let c_n denote the number of communicataing robots
                c_n = length(comm_robots);
                gStar = communication(Vtilde,id, c_n, comm_robots);
                % For path planning from here, treat
                %   gStar(:,1) as the posterior mean of all M*N cells
                %   gStar(:,2) as the posterior variance of all M*N cells
                % Note: if robot i is constrained to the cells in region V{i}, then it is
                %       sufficient to compute only gStar(V{i},:) (and before that only
                %       pHat(V{i},:) accordingly), but we did not exercise such savings in
                %       computationl here
                comm_robots=[id]; comm_flag = 0;
                %% Pat's code ens here.
            end
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
        break;
        
        % read the sensors, e.g.:
        %  rgb = wb_camera_get_image(camera);
        
        % Process here sensor data, images, etc.
        
        % send actuator commands, e.g.:
        %  wb_motor_set_postion(motor, 10.0);
        
        % if your code plots some graphics, it needs to flushed like this:
        %drawnow;
        
    end
end
    
    % cleanup code goes here: write data to files, etc.