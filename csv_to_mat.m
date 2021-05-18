% Small script to export position and local updraft estimate 
% from csv to .matfile

logdata = readmatrix('./HIL_Test_17_05/log_17-May-2021_13-00.csv');

position_array = logdata(1:601,5:7)';
local_updraft_estimate = logdata(2:601,8); % aka energy reward

save('HIL_1705_filter_input.mat','position_array','local_updraft_estimate');