%% Script for plotting particles after simulation
 % This tool is used to visualize the particle distribution of the 
 % particle filter, which is used for updraft estimation. 
 %

%% Clean workspae and load flight log 
% clean variables and workspace
clc 
clear all
close all

% load data from flight test
load('Flight_Test_24_09/Flight_Test_24_09_filter_result.mat');
log_frequency = 10; % number of logged data points per second

%% Settings 

start_step = 13000;
end_step = filter_steps;
start_time = start_step/log_frequency;
end_time = end_step/log_frequency;

%% Add main figure

figure;
set(gcf, 'Position',  [200, 200, 500, 600]);

%% Particle and updraft plot

subplot(3,1,[1,2]);
title('Visualization of particle filter');
hold on

% Add scatter plot handle for particles
particle_plot = scatter(particle_array(2,:,1),particle_array(1,:,1),particle_array(5,:,1));
xlim([-400,400]);
ylim([-150,500]);
xlabel('East [m]');
ylabel('North [m]');
hold on 

% plot triangle
plot([-350,350,0,-350],...
     [0,0,350,0],...
    '--k','LineWidth',1);
hold on

% createhandle for aircraft position
position_plot = scatter(vehicle_position(2,1),vehicle_position(1,1),5,'ok','LineWidth',3);

% range from 0 to 2*pi for plotting circles 
theta = 0:pi/20:2*pi;

% Add handles for estimated updrafts
updraft_est_1 = plot(50*cos(theta),50*sin(theta), 'Color','r','LineWidth',1.5);
hold on 
updraft_est_2 = plot(50*cos(theta),50*sin(theta), 'Color','r','LineWidth',1.5);
hold on
updraft_est_3 = plot(50*cos(theta),50*sin(theta), 'Color','r','LineWidth',1.5);
hold on
updraft_est_4 = plot(50*cos(theta),50*sin(theta), 'Color','r','LineWidth',1.5);
hold on
updraft_est_5 = plot(50*cos(theta),50*sin(theta), 'Color','r','LineWidth',1.5);
hold on
updraft_est_6 = plot(50*cos(theta),50*sin(theta), 'Color','r','LineWidth',1.5);
hold on

% add grid and legend
legend([particle_plot,position_plot,updraft_est_1],...
    {'Particles','Aircraft Position','Real Updraft','Estimated Updraft'})
grid on

% Add textboxes for time and control mode
time_annotation = annotation('textbox',[.15 .5 .3 .41],'String',sprintf('t=%3d',i),'FitBoxToText','on');
control_annotation = annotation('textbox',[.3 .5 .3 .41],'String',sprintf('Mode=%s','Manual'),'FitBoxToText','on');

%% Altitude plot
subplot(3,1,3)
height_plot = animatedline(0,0,'Color','r','LineWidth',1);
xlim([start_time, end_time]);
ylim([0,500]);
xlabel('t [s]')
ylabel('Altitude [m]')
legend([height_plot],...
    {'Altitude [m]'})
grid on

%% Update plot

% iterate through particle array and update plots
for i = start_step:log_frequency:(end_step)
    
    % update particles 
    x = particle_array(2,:,i);
    y = particle_array(1,:,i);
    marker = particle_array(5,:,i).*300;
    set(particle_plot,'XData',x,'YData',y,'SizeData',marker,'CData',[1 0 0]);
    
    % udpdate updraft positions
    update_updraft_plot(updraft_est_1, filtered_state_array, 1, i);
    update_updraft_plot(updraft_est_2, filtered_state_array, 2, i);
    update_updraft_plot(updraft_est_3, filtered_state_array, 3, i);
    update_updraft_plot(updraft_est_4, filtered_state_array, 4, i);
    update_updraft_plot(updraft_est_5, filtered_state_array, 5, i);
    update_updraft_plot(updraft_est_6, filtered_state_array, 6, i);
    
    % update aircraft position
    set(position_plot,'XData',vehicle_position(i,2),'YData',vehicle_position(i,1));
    set(time_annotation,'String',sprintf('t=%d s',(i-1)/10));
    
    if control_mode(i) == 1
        set(control_annotation,'String',sprintf('Mode=%s','Manual'));
    elseif control_mode(i) == 2
        set(control_annotation,'String',sprintf('Mode=%s','Auto'));
    end
    
    % update height
    addpoints(height_plot, double(i/10),-vehicle_position(i,3));
    
    %drawnow 
    frame = getframe(gcf); %get frame
    pause(0.01)
end

function update_updraft_plot(updraft_handle, filtered_state_array, updraft, index)
 % Update updraft position plot for up to 6 updrafts
    
theta = 0:pi/20:2*pi;
    x_up_est = filtered_state_array(2,updraft,index) + 50 * cos(theta); % EAST
    y_up_est = filtered_state_array(1,updraft,index) + 50 * sin(theta); % NORTH
    
    if filtered_state_array(2,updraft,index) == 0
        set(updraft_handle,'Visible','Off');
    else
        set(updraft_handle,'Visible','On');
        set(updraft_handle,'XData',x_up_est,'YData',y_up_est);
    end
    
end
