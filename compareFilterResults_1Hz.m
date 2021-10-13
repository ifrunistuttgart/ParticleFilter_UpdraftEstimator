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

file1 = matfile('Flight_Test_24_09/Flight_Test_24_09_filter_result_1_Hz.mat');
file2 = matfile('Flight_Test_24_09/Flight_Test_24_09_filter_result_1_Hz_1.mat');
files = {file1,file2};

fig_title = {'Filter result 1', 'Filter result 2'};
%% Settings 

start_step = 1300;
end_step = 1900;
start_time = start_step;
end_time = end_step;

%% Load data

for i=1:2
    filtered_state{i} = files{i}.filtered_state_array(:,:,start_step:end_step);
    local_updraft{i} = files{i}.local_updraft(1,start_step:end_step);
    particles{i} = files{i}.particle_array(:,:,start_step:end_step);
    vehicle_position{i} = files{i}.vehicle_position(start_step:end_step,:);
    control_mode{i} = files{i}.control_mode(1,start_step:end_step);
end
%% Add main figure

f(1) = figure;
set(gcf, 'Position',  [200, 200, 500, 600]);
f(2) = figure;
set(gcf, 'Position',  [200, 200, 500, 600]);

%% Particle and updraft plot
for fig_num=1:2
    
    figure(f(fig_num));
    
    subplot(3,1,[1,2]);
    title(fig_title{fig_num});
    hold on

    % Add scatter plot handle for particles
    particle_plot(fig_num) = scatter(particles{i}(2,:,1),particles{i}(1,:,1),particles{i}(5,:,1));
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
    position_plot(fig_num) = scatter(vehicle_position{i}(2,1),vehicle_position{i}(1,1),5,'ok','LineWidth',3);

    % range from 0 to 2*pi for plotting circles 
    theta = 0:pi/20:2*pi;

    % Add handles for estimated updrafts
    updraft_est_1(fig_num) = plot(50*cos(theta),50*sin(theta), 'Color','r','LineWidth',1.5);
    hold on 
    updraft_est_2(fig_num) = plot(50*cos(theta),50*sin(theta), 'Color','r','LineWidth',1.5);
    hold on
    updraft_est_3(fig_num) = plot(50*cos(theta),50*sin(theta), 'Color','r','LineWidth',1.5);
    hold on
    updraft_est_4(fig_num) = plot(50*cos(theta),50*sin(theta), 'Color','r','LineWidth',1.5);
    hold on
    updraft_est_5(fig_num) = plot(50*cos(theta),50*sin(theta), 'Color','r','LineWidth',1.5);
    hold on
    updraft_est_6(fig_num) = plot(50*cos(theta),50*sin(theta), 'Color','r','LineWidth',1.5);
    hold on

    % add grid and legend
    legend([particle_plot(fig_num),position_plot(fig_num),updraft_est_1(fig_num)],...
        {'Particles','Aircraft Position','Updraft Estimates','Estimated Updraft'})
    grid on

    % Add textboxes for time and control mode
    time_annotation(fig_num) = annotation('textbox',[.15 .5 .3 .41],'String',sprintf('t=%3d',i),'FitBoxToText','on');
    control_annotation(fig_num) = annotation('textbox',[.3 .5 .3 .41],'String',sprintf('Mode=%s','Manual'),'FitBoxToText','on');

    %% Altitude plot
    subplot(3,1,3)
    height_plot(fig_num) = animatedline(0,0,'Color','r','LineWidth',1);
    xlim([start_time, end_time]);
    ylim([0,500]);
    xlabel('t [s]')
    ylabel('Altitude [m]')
    legend([height_plot(fig_num)],...
        {'Altitude [m]'})
    grid on
end

%% Update plot

% iterate through particle array and update plots
for i = 1:(end_step-start_step)
    for fig_num=1:2
        
        figure(f(fig_num));
        
        % update particles 
        x = particles{fig_num}(2,:,i);
        y = particles{fig_num}(1,:,i);
        marker = max(1e-200, particles{fig_num}(5,:,i).*300);
        set(particle_plot(fig_num),'XData',x,'YData',y,'SizeData',marker,'CData',[1 0 0]);

        % udpdate updraft positions
        update_updraft_plot(updraft_est_1(fig_num), filtered_state{fig_num}, 1, i);
        update_updraft_plot(updraft_est_2(fig_num), filtered_state{fig_num}, 2, i);
        update_updraft_plot(updraft_est_3(fig_num), filtered_state{fig_num}, 3, i);
        update_updraft_plot(updraft_est_4(fig_num), filtered_state{fig_num}, 4, i);
        update_updraft_plot(updraft_est_5(fig_num), filtered_state{fig_num}, 5, i);
        update_updraft_plot(updraft_est_6(fig_num), filtered_state{fig_num}, 6, i);

        % update aircraft position
        set(position_plot(fig_num),'XData',vehicle_position{fig_num}(i,2),'YData',vehicle_position{fig_num}(i,1));
        set(time_annotation(fig_num),'String',sprintf('t=%d s',(start_step+i-1)));

        if control_mode{fig_num}(i) == 1
            set(control_annotation(fig_num),'String',sprintf('Mode=%s','Manual'));
        elseif control_mode{fig_num}(i) == 2
            set(control_annotation(fig_num),'String',sprintf('Mode=%s','Auto'));
        end

        % update height (start_step + i )/10 calculates correct flight time in [s]
        addpoints(height_plot(fig_num), double(start_step+i),-vehicle_position{fig_num}(i,3));

        %drawnow 
        frame = getframe(gcf); %get frame
        %pause(0.01)
    end
end


function update_updraft_plot(updraft_handle, filtered_state, updraft, index)
 % Update updraft position plot for up to 6 updrafts
    
    theta = 0:pi/20:2*pi;
    updraft_radius = filtered_state(4,updraft,index);
    x_up_est = filtered_state(2,updraft,index) + updraft_radius * cos(theta); % EAST
    y_up_est = filtered_state(1,updraft,index) + updraft_radius * sin(theta); % NORTH
    
    if filtered_state(2,updraft,index) == 0
        set(updraft_handle,'Visible','Off');
    else
        set(updraft_handle,'Visible','On');
        if filtered_state(3,updraft,index) > 0
            set(updraft_handle,'XData',x_up_est,'YData',y_up_est, 'Color','r');
        else
            set(updraft_handle,'XData',x_up_est,'YData',y_up_est, 'Color','b');
        end
        
    end
    
end
