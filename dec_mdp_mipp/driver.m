%% common initial data fo both the algorithms
clc
clear
M=14;% for a MxM environment.
X_train = (1:M*M)';
init_trn_data = 0.1;% percentage of all nodes in the environment.
tic
for n=2:2:6
    for run = 1:10
        x_active = randsample(X_train,ceil(init_trn_data*M*M));%select k% random points to initialize GP hyperparameters.
        range = 0.30;
        
        %% pre-processing -- K-means clustering for partitioining the region -- common variable.
        %id = 1;
        [idx,centr] = find_partitions(n,M,0);% params: (robot#,size,display_flag)
        
        %% call the MDP-OC function
        mdp_IPP(n,M,x_active,0,0,0,idx, centr, range,run);
        pause(1);
        clc
        
        %% call the MDP-NC function
        %mdp_IPP(n,M,x_active,1,0,0,idx, centr, range,run);
        pause(1);
        clc
        
        %% call the MDP-CC function
        mdp_IPP(n,M,x_active,0,1,0,idx, centr, range,run);
        pause(1);
        clc
        
        %% call the greedy-OC function
        %greedy(n,M,x_active,0,0,0,idx, centr, range,run);
        pause(1);
        clc
        
        
        %% call the greedy-NC function
        %greedy(n,M,x_active,1,0,0,idx, centr, range,run);
        pause(1);
        clc
        
        %% call the greedy-CC function
        %greedy(n,M,x_active,0,1,0,idx, centr, range,run);
        pause(1);
        clc
    end
end
disp(toc);
clear;