function [newrow,newcol] = find_greedy_a(row, col, grid, size)
neigh_rewards = zeros(4)*-1000;
newrow = -1; newcol = -1;
for best_a = 1:1:4
    if(best_a==1)
        if row-1>0
            newrow = row-1;
            newcol = col;
        end
    end
    if(best_a==2)
        if row+1<=size
            newrow = row+1;
            newcol = col;
        end
    end
    if(best_a==3)
        if col+1<=size
            newrow = row;
            newcol = col+1;
        end
    end
    if(best_a==4)
        if col-1>0
            newrow = row;
            newcol = col-1;
        end
    end
    if newrow > 0 && newrow <= size && newcol > 0 && newcol <= size
        state = append('[',num2str(newrow),',',num2str(newcol),']');
        neigh_rewards(best_a) = grid.R(1,state2idx(grid,state),1);
    end
end
best_a = find(neigh_rewards == max(neigh_rewards(:)));
if(best_a==1)
    newrow = row-1;
    newcol = col;
end
if(best_a==2)
    newrow = row+1;
    newcol = col;
end
if(best_a==3)
    newrow = row;
    newcol = col+1;
end
if(best_a==4)
    newrow = row;
    newcol = col-1;
end
end

