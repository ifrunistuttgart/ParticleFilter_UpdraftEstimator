%% Small script to analyse local updraft from flight test

%%
close all
clear 
clc 

% load local updraft from postprocessed filter results 
load('Flight_Test_24_09/Flight_Test_24_09_filter_result.mat', 'local_updraft');

aoi = 13500:18500;
plot(movmean(local_updraft(aoi),10),'r');
hold on
plot(medfilt1(local_updraft(aoi),8),'k');
hold on
plot(local_updraft(aoi),'c');
hold on
legend
xlim([0,5001]);
grid