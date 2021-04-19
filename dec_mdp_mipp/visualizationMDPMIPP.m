folderCR30 = 'C:\Users\n01388138\Documents\MATLAB\dec_mdp_mipp\Results\CR=30percent\';
folderMSG = 'C:\Users\n01388138\Documents\MATLAB\dec_mdp_mipp\Results\CR=30percent\msg_CC_OC\';

%% average MSE plot
for n =2:2:6
    figure();
    for al = 1:6
        if al==1
            titleS = 'Dec-MDP-OC';
        end
        if al==2
            titleS = 'Dec-MDP-NC';
        end
        if al==3
            titleS = 'Dec-MDP-CC';
        end
        if al==4
            titleS = 'Greedy-OC';
        end
        if al==5
            titleS = 'Greedy-NC';
        end
        if al==6
            titleS = 'Greedy-CC';
        end
        allruns = [];
        for run =1:10
            str = strcat(folderCR30,titleS,num2str(n),'_run_',num2str(run),'.mat');
            load(str);
            avgMSE = (sum(cell2mat(allMSE')))/n ;
            allruns = [allruns; avgMSE];
        end
        if al==1
            stdv = std(allruns);
            xx = 1:1:numel(stdv);
            shadedErrorBar(xx,mean(allruns),stdv,'lineprops','-b','patchSaturation',0.03);
        else
            if al<4
                plot(mean(allruns),'-');
            else
                plot(mean(allruns),'--');
            end
        end
        hold all;
        
        clearvars allMSE allReward allVar timeSec allPaths allCC Pred_n;
    end
    hold off;
    ylabel('Average MSE','FontSize',14);
    xlabel('Path length','FontSize',14);
    plotT = strcat('n=',num2str(n));
    title(plotT,'FontSize',14);
    legend('Dec-MDP-OC','Dec-MDP-NC','Dec-MDP-CC','Greedy-OC','Greedy-NC','Greedy-CC','FontSize',10,'Location','best')
    %filename = strcat('msecomp',num2str(n),'.png');
    %saveas(gcf,filename);
end

%% average variance plot
allruns=[];
for n =2:2:6
    figure();
    for al = 1:6
        if al==1
            titleS = 'Dec-MDP-OC';
        end
        if al==2
            titleS = 'Dec-MDP-NC';
        end
        if al==3
            titleS = 'Dec-MDP-CC';
        end
        if al==4
            titleS = 'Greedy-OC';
        end
        if al==5
            titleS = 'Greedy-NC';
        end
        if al==6
            titleS = 'Greedy-CC';
        end
        allruns = [];
        for run =1:10
            str = strcat(folderCR30,titleS,num2str(n),'_run_',num2str(run),'.mat');
            load(str);
            out = mean(mean(cat(3,allVar{:}),3));
            allruns = [allruns; out];
        end
        if al<4
            plot(mean(allruns),'-');
        else
            plot(mean(allruns),'--');
            
        end
        hold all;
        clearvars allMSE allReward allVar timeSec allPaths allCC Pred_n;
    end
    hold off;
    ylabel('Average Variance','FontSize',14);
    xlabel('Path length','FontSize',14);
    plotT = strcat('n=',num2str(n));
    title(plotT,'FontSize',14);
    legend('Dec-MDP-OC','Dec-MDP-NC','Dec-MDP-CC','Greedy-OC','Greedy-NC','Greedy-CC','FontSize',10,'Location','best')
    filename = strcat('varcomp',num2str(n),'.png');
    %saveas(gcf,filename);
end


%% average time plot
X = [2,4,6];
allruns = [];
for n =2:2:6
    oneN = [];
    for al = 1:6
        if al==1
            titleS = 'Dec-MDP-OC';
        end
        if al==2
            titleS = 'Dec-MDP-NC';
        end
        if al==3
            titleS = 'Dec-MDP-CC';
        end
        if al==4
            titleS = 'Greedy-OC';
        end
        if al==5
            titleS = 'Greedy-NC';
        end
        if al==6
            titleS = 'Greedy-CC';
        end
        yy1 = [];
        for run =1:10
            str = strcat(folderCR30,titleS,num2str(n),'_run_',num2str(run),'.mat');
            load(str);
            yy1 = [yy1; timeSec];
        end
        yy = mean(yy1);
        oneN = [oneN yy];
        %plot(mean(allruns));
        %hold all;
        clearvars allMSE allReward allVar timeSec allPaths allCC Pred_n;
    end
    allruns = [allruns; oneN];
    %hold off;
end
bar(X, allruns);
ylabel('Time(sec.)','FontSize',14);
xlabel('Number of robots','FontSize',14);
legend('Dec-MDP-OC','Dec-MDP-NC','Dec-MDP-CC','Greedy-OC','Greedy-NC','Greedy-CC','Location','best')
filename = strcat('timecomp.png');
%saveas(gcf,filename);

%% average msg plot
X = [2,4,6];
allruns = [];
for n =2:2:6
    oneN = [];
    for al = 1:1
        if al==1
            titleS = 'Dec-MDP-OC';
        end
        if al==2
            titleS = 'Dec-MDP-CC';
        end
        yy1 = [];
        for run =1:10
            str = strcat(folderMSG,titleS,num2str(n),'_run_',num2str(run),'.mat');
            load(str);
            yy1 = [yy1; (msgcount+2)];
        end
        yy = mean(yy1);
        oneN = [oneN yy];
        %plot(mean(allruns));
        %hold all;
        clearvars allMSE allReward allVar timeSec allPaths allCC Pred_n msgcount;
    end
    allruns = [allruns; oneN];
    %hold off;
end
bar(X, allruns);
ylabel('No. of messages sent','FontSize',14);
xlabel('Number of robots','FontSize',14);
legend('Dec-MDP-OC','Location','best')
filename = strcat('message.png');
saveas(gcf,filename);

%% average reward (entropy) plot
X = [2,4,6];
allruns = [];
for n =2:2:6
    oneN = [];
    for al = 1:2
        if al==1
            titleS = 'Dec-MDP-OC';
        end
        if al==2
            titleS = 'Greedy-OC';
        end
        yy1 = [];
        for run =1:10
            str = strcat(folderCR30,titleS,num2str(n),'_run_',num2str(run),'.mat');
            load(str);
            yy1 = [yy1; sum(cell2mat(allReward))/n];
        end
        yy = mean(yy1);
        oneN = [oneN yy];
        %plot(mean(allruns));
        %hold all;
        clearvars allMSE allReward allVar timeSec allPaths allCC Pred_n;
    end
    allruns = [allruns; oneN];
    %hold off;
end
bar(X, allruns);
ylabel('Reward','FontSize',14);
xlabel('Number of robots','FontSize',14);
legend('Dec-MDP-OC','Greedy-OC','Location','best')
filename = strcat(folderCR30,'entropy2.png');
saveas(gcf,filename);

%% connected component plot
allruns = [];
for n =2:2:6
    for al = 1:2
        if al==1
            titleS = 'Dec-MDP-OC';
        end
        if al==2
            titleS = 'Greedy-OC';
        end
        allruns = [];
        for run =1:10
            str = strcat('Results\',titleS,num2str(n),'_run_',num2str(run),'.mat');
            load(str);
            allruns = [allruns; allCC(1,1:20)];
        end
        if n==2
            plot(mean(allruns),'-');
        end
        if n==4
            plot(mean(allruns),'--');
        end
        if n==6
            plot(mean(allruns),'-.');
        end
        hold all;
        clearvars allMSE allReward allVar timeSec allPaths allCC Pred_n;
    end
end
hold off;
ylabel('No. of Connected Components','FontSize',14);
xlabel('Path length','FontSize',14);
xlim([0 21]);
ylim([0 7]);
legend('n=2, Dec-MDP-OC','n=2, Greedy-OC','n=4, Dec-MDP-OC','n=4, Greedy-OC','n=6, Dec-MDP-OC','n=6, Greedy-OC','Location','best')
filename = strcat('ConnComp.png');
saveas(gcf,filename);

%% Intro picture: partition + information field
clear;
%first plot the partitions.
M=14;k=3;
[x,y] = meshgrid(1:1:M,1:1:M);
X = [y(:), x(:)];
[idx,C] = find_partitions(k,M,0);
Z_part1 = -2 * ones(1,numel(X(idx==1,1)));
Z_part2 = -2 * ones(1,numel(X(idx==2,1)));
Z_part3 = -2 * ones(1,numel(X(idx==3,1)));
Z_cent = [-2,-2,-2];
scatter3(X(idx==1,1)-0.5,X(idx==1,2)-0.5,Z_part1,'rd','LineWidth',1);
hold all;
scatter3(X(idx==2,1)-0.5,X(idx==2,2)-0.5,Z_part2,'ko','filled');
scatter3(X(idx==3,1)-0.5,X(idx==3,2)-0.5,Z_part3,'mx','LineWidth',2.5);
plot3(C(:,2)-0.5,C(:,1)-0.5,Z_cent,'ro',...
     'MarkerSize',10,'LineWidth',2);
 hold on;
% plot the information field now.
data_gt = data_import('dataFile_ell25_50by50.csv', 1, 50);
data_gt = data_gt(1:M+1,1:M+1);
[Y,X] = meshgrid(0:1:M,0:1:M);
Z_gt = str2double(table2array(data_gt));
surf(Y,X,Z_gt,'EdgeColor','None'); view(2);
hold off
%camroll(180)
%set(gca, 'XDir','reverse');
%set(gca, 'YDir','reverse');
xlim([0 M]);
ylim([0 M]);
ylabel('Latitude','FontSize',14);
xlabel('Longitude','FontSize',14);

%% predicted model visualization
M = 14;
n=2;
run = 8;
figure();
data_gt = data_import('dataFile_ell25_50by50.csv', 1, 50);
data_gt = data_gt(1:M,1:M);
Z_gt = str2double(table2array(data_gt));
surf(Z_gt,'EdgeColor','None'); view(2);
surf(Z_gt,'EdgeColor','None'); view(2);
title('Ground Truth');
%colorbar;
cl = caxis;
xlim([1 M]);
ylim([1 M]);
ylabel('Latitude','FontSize',12);
xlabel('Longitude','FontSize',12);
filename = strcat('GroundTruth',num2str(n),num2str(run),'_GTvsPred.png');
saveas(gcf,filename);
for al = 1:3
    figure();
    if al==1
        titleS = 'Dec-MDP-OC';
    end
    if al==2
        titleS = 'Greedy-OC';
    end
    if al==3
        titleS = 'Dec-MDP-NC';
    end
    %calculate the average model and plot.
    str = strcat(folderCR30,titleS,num2str(n),'_run_',num2str(run),'.mat');
    load(str);
    A = Pred_n;
    as = size(A,2);
    matSize = size(A{1},1);
    B = reshape(cell2mat(A),matSize,[],as);
    C = sum(B,3);
    C = C/n;
    surf(reshape(C,[M,M]),'EdgeColor','None'); view(2);
    title(titleS);
    %ylabel('Latitude','FontSize',12);
    xlabel('Longitude','FontSize',12);
    %sgtitle(titleS);
    %colorbar;
    %caxis([cl(1) cl(2)]);
    xlim([1 M]);
    ylim([1 M]);
    filename = strcat(titleS,num2str(n),num2str(run),'_GTvsPred.png');
    %saveas(gcf,filename);
    clearvars allMSE allReward allVar timeSec allPaths allCC Pred_n;
end



%% variance visualization
M = 14;
n=4;
run = 1;
for al = 1:3
    figure();
    if al==1
        titleS = 'Dec-MDP-OC';
    end
    if al==2
        titleS = 'Greedy-OC';
    end
    if al==3
        titleS = 'Dec-MDP-NC';
    end
    %titleS = 'Dec-MDP-OC';
    str = strcat(folderCR30,titleS,num2str(n),'_run_',num2str(run),'.mat');
    load(str);
    SD_final = zeros(M*M,1);
    for i=1:n
        SD_final = plus(SD_final,allVar{i}(:,end));
    end
    SD_final = reshape(SD_final/n,[M,M]);
    surf(SD_final,'EdgeColor','None'); %view(2);
    colormap(hsv);
    title('Final Variance');
    SD_init = zeros(M*M,1);
    for i=1:n
        SD_init = plus(SD_init,allVar{i}(:,1));
    end
    dim = [.2 .5 .3 .3];
    str = strcat('Initial variance = ',num2str(round(mean(SD_init/n),2)));
    annotation('textbox',dim,'String',str,'FitBoxToText','on','FontSize',10);
    title(titleS, 'FontSize',12);
    ylabel('Latitude','FontSize',12);
    xlabel('Longitude','FontSize',12);
    colorbar;
    filename = strcat(titleS,num2str(n),'_SDinitvFinal.png');
    saveas(gcf,filename);
    clearvars allMSE allVar SD_final SD_init;
end

