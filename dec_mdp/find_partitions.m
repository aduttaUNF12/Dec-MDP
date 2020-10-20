function [idx,C] = find_partitions(k,size,display)
%FIND_PARTITIONS Summary of this function goes here
%   Detailed explanation goes here
[x,y] = meshgrid(1:1:size,1:1:size);
X = [y(:), x(:)];
[idx,C] = kmedoids(X,k,'Distance','cityblock');
if display==1
    figure;
    gscatter(X(:,2),X(:,1),idx)
    hold on
    plot(C(:,2),C(:,1),'kx','MarkerSize',15);
    %legend('Cluster 1','Cluster 2','Cluster 3','Cluster Centroid')
    title 'Partitions and Initial robot locations (centroids)'
    hold off
    %camroll(180)
    %set(gca, 'XDir','reverse');
    set(gca, 'YDir','reverse');
end
end

