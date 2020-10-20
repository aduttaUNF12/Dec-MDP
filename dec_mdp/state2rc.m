function [row,col] = state2rc(state)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
current_state_id = erase(state,"[");
current_state_id = erase(current_state_id,"]");
current_state_id = strsplit(current_state_id,',');
row = str2double(current_state_id{1,1});
col = str2double(current_state_id{1,2});
end

