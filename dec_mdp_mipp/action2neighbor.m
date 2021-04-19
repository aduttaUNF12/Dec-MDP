function [newrow,newcol] = action2neighbor(action, row, col)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
if(action==1)
    newrow = row-1;
    newcol = col;
end
if(action==2)
    newrow = row+1;
    newcol = col;
end
if(action==3)
    newrow = row;
    newcol = col+1;
end
if(action==4)
    newrow = row;
    newcol = col-1;
end
end

