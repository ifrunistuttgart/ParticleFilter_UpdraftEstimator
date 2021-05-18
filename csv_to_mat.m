% Small script to export position and local updraft estimate 
% from csv to .matfile

logdata = readmatrix('./HIL_Test_17_05/log_17-May-2021_13-00.csv');

position_array = logdata(2:601,5:7)';
local_updraft_estimate = logdata(2:601,8);

save('HiL_1705.mat','position_array'