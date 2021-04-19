function act = uncerainA(best_a,myStates,GW,pg,neighbors,nextId)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
[row,col] = state2rc(GW.CurrentState);
n = state2idx(GW,GW.CurrentState);
%next = append('[',num2str(newrow),',',num2str(newcol),']');
%nextId = state2idx(GW,next);
%probs = zeros (1,numel(neighbors));
idx = find(neighbors==nextId);
probs = GW.T(n,neighbors,best_a);
p = rand;

if p <= pg
    act = best_a;
    return;
else
    probs(idx) = 0;
    while 1
        r = randi(length(neighbors), 1);
        if probs(r)~=0
            act = r;
            [newr,newc] = action2neighbor(act,row,col);
            potential = append('[',num2str(newr),',',num2str(newc),']');
            if ~ismember(state2idx(GW,potential),myStates)
               probs(r)=0;
            end
            break;
        end
        if max(probs) == 0
            act = best_a;
            return;
        end
    end
end


end

