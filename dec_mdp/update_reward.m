function [rewards] = update_reward(gpr, Y_pred, Y_sd, x_active, world, size, unseen)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% for i=1:1:size
%     for j=1:1:size
%
%     end
% end

%% (Krause) entropy for the nodes in the env.
H = 0.5 * log(2*pi*exp(1)*(Y_sd.^2));

%% online entropy calculation here...
% kfcn = gpr.Impl.Kernel.makeKernelAsFunctionOfXNXM(gpr.Impl.ThetaHat);
% cov_mat = kfcn(x_active(:,:),x_active(:,:));%full covariance matrix.
% U_size = (size*size) - numel(x_active);%number of unseen locs
% H_on = 0.5 * log(((2*pi*exp(1))^U_size)* det(cov_mat)); %Liu entropy for online measurement
% H_zab = 0.5 * log (det(cov_mat)) + ((size*size)/2) * (1 + log(2*pi)); % Wei, Zheng (2020)
% MI = H_za - H_zab;

%% update the unseen locations' rewards (e.g., Entropy)
for i = 1:1:numel(unseen)/2
    loc=unseen(i,:);newrow= loc(1,1);newcol=loc(1,2);
    stateid = state2idx(world,append('[',num2str(newrow),',',num2str(newcol),']')); 
    world.R(:,stateid,:) = H(i);
%     if ismember(stateid,x_active)%if visited before, assign large penalty
%         world.R(:,stateid,:) = 0;
%     end
end
%world.R(:,state2idx(world,world.TerminalStates),:) = 1000;
world.R(:,state2idx(world,world.ObstacleStates),:) = -100000;
%world.R(state2idx(world,world.ObstacleStates),:,:) = -100000;
rewards = world.R;
end

