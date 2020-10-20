function [T] = customTransition(T,size, pGood, pBad)
%custom transition matrix is created here.
firstR = (1:size:size*size);
firstC = 1:1:size;
lastR = size:size:size*size;
lastC = firstR(end):1:size*size;

for n=1:1:size*size
    neigh = [];
    %for NORTH
    if ~ismember(n,firstR)
        neigh = [neigh, n-1];
        %T(n,n-1,3) = pBad/2;
        %T(n,n-1,4) = pBad/2;
    else
        neigh = [neigh, n];%stay put
    end
    T(n,neigh(1),1) = pGood;
    if ismember(n,lastC)
        T(n,n-size,1) = pBad;
    else
        if ismember(n,firstC)
            T(n,n+size,1) = pBad;
        else
            T(n,n-size,1) = pBad/2;
            T(n,n+size,1) = pBad/2;
        end
    end
    % for SOUTH
    if ~ismember(n,lastR)
        neigh = [neigh, n+1];
        T(n,n+1,2) = pGood;
        %T(n,n+1,3) = pBad/2;
        %T(n,n+1,4) = pBad/2;
    else
        neigh = [neigh, n];%stay put
    end
    T(n,neigh(2),2) = pGood;
    if ismember(n,lastC)
       T(n,n-size,2) = pBad; 
    else
        if ismember(n,firstC)
            T(n,n+size,2) = pBad;
        else
            T(n,n-size,2) = pBad/2;
            T(n,n+size,2) = pBad/2;
        end
    end
    % for EAST
    if ~ismember(n,lastC)
        neigh = [neigh, n+size];%east
        T(n,n+size,3) = pGood;
        %T(n,n+size,1) = pBad/2;
        %T(n,n+size,2) = pBad/2;
    else
        neigh = [neigh, n];%stay put
    end
    T(n,neigh(3),3) = pGood;
    if ismember(n,lastR)
       T(n,n-1,3) = pBad; 
    else
        if ismember(n,firstR)
            T(n,n+1,3) = pBad;
        else
            T(n,n-1,3) = pBad/2;
            T(n,n+1,3) = pBad/2;
        end
    end
    % for WEST
    if ~ismember(n,firstC)
        neigh = [neigh, n-size];%west
        T(n,n-size,4) = pGood;
        %T(n,n-size,1) = pBad/2;
        %T(n,n-size,2) = pBad/2;
    else
        neigh = [neigh, n];%stay put
    end
    T(n,neigh(4),4) = pGood;
    if ismember(n,lastR)
       T(n,n-1,4) = pBad; 
    else
        if ismember(n,firstR)
            T(n,n+1,4) = pBad;
        else
            T(n,n-1,4) = pBad/2;
            T(n,n+1,4) = pBad/2;
        end
    end
end

end

