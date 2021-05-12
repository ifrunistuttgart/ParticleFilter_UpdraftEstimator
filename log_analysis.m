%% Analyse log from HiL-Testing

data = readtable('log_11-May-2021_14-19.csv');

NED_temp = char(table2cell(data(:,5)));
position_array = zeros(3,614);

for i=1:614
    exp = char(regexp(NED_temp(i,:),'\[\s*.*\]','match'));
    position_array(:,i) = str2num(exp(2:end-1))';
end
local_updraft_estimate = table2array(data(:,6))';
save('HiL_test_filter_input.mat','position_array','local_updraft_estimate')

