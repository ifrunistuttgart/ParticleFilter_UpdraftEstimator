%% Script for plotting particles after simulation

addpath(fullfile('gnc'));
atmosphere = initAtmos();

% load old simulation data

filepath = 'HiL_test_filter_result.mat'
load(filepath);

sim_time = 1200;

% create figure object
figure;

set(gcf, 'Position',  [200, 200, 500, 600])


subplot(3,1,[1,2])
title('Visualization of particle filter')
hold on

% create scatter plot handle for particles and set x-y limits
particle_plot = scatter(particle_array(2,:,1),particle_array(1,:,1),particle_array(5,:,1));
xlim([-550,650]);
ylim([-300,650]);
xlabel('East [m]');
ylabel('North [m]');
hold on 

% plot triangle (code taken and modified from state monitor)
plot([-350,350,0,-350],...
     [0,0,350,0],...
    '--k','LineWidth',1);
hold on

% create scatter handle for plotting aircraft position
position_plot = scatter(position_array(2,1),position_array(1,1),5,'ok','LineWidth',3);

% height plot


% range from 0 to 2*pi for plotting circles 
theta = 0:pi/20:2*pi;

% plot updrafts
% for i=1:2
%        radius = 80;
%        real_updraft_x = atmosphere.updraft_position(2,i) + radius * cos(theta); % EAST -> x-axis
%        real_updraft_y = atmosphere.updraft_position(1,i) + radius * sin(theta); % NORTH -> y-axis
%        real_updraft_plot = plot(real_updraft_x,real_updraft_y, 'Color','b','LineWidth',1.5);
% end

% create handle for plotting estimated updraft
updraft_est_1 = plot(50*cos(theta),50*sin(theta), 'Color','r','LineWidth',1.5);
hold on 
updraft_est_2 = plot(50*cos(theta),50*sin(theta), 'Color','r','LineWidth',1.5);
hold on

% add grid and legend
legend([particle_plot,position_plot,updraft_est_1],...
    {'Particles','Aircraft Position','Real Updraft','Estimated Updraft'})
grid on

time_annotation = annotation('textbox',[.2 .5 .3 .41],'String',sprintf('t=%3d',i),'FitBoxToText','on');

%% Plot height
subplot(3,1,3)
X = 1:1800;
height_plot = animatedline(0,0,'Color','r','LineWidth',1);
xlim([0,sim_time]);
ylim([0,500]);
xlabel('t [s]')
ylabel('Altitude [m]')
legend([height_plot],...
    {'Altitude [m]'})
grid on

%  myVideo = VideoWriter('particleAnimation','MPEG-4'); %open video file
%  myVideo.FrameRate = 5;  %can adjust this, 5 - 10 works well for me
%  open(myVideo)

% iterate through particle array and update plots
for i = 1:5:(sim_time+5)
    
    % update particles 
    x = particle_array(2,:,i);
    y = particle_array(1,:,i);
    marker = particle_array(5,:,i).*300;
    set(particle_plot,'XData',x,'YData',y,'SizeData',marker,'CData',[1 0 0]);
    
    % update updraft estimation 1 
    x_up_est_1 = filtered_state_array(2,1,i) + 50 * cos(theta); % EAST
    y_up_est_1 = filtered_state_array(1,1,i) + 50 * sin(theta); % NORTH
    %set(updraft_est_1,'XData',x_up_est_1,'YData',y_up_est_1);
    
    if filtered_state_array(2,1,i) == 0
        set(updraft_est_1,'Visible','Off');
    else
        set(updraft_est_1,'Visible','On');
        set(updraft_est_1,'XData',x_up_est_1,'YData',y_up_est_1);
    end
    % update updraft estimation 2
    x_up_est_2 = filtered_state_array(2,2,i) + 50 * cos(theta); % EAST
    y_up_est_2 = filtered_state_array(1,2,i) + 50 * sin(theta); % NORTH
    
    
    if filtered_state_array(2,2,i) == 0
        set(updraft_est_2,'Visible','Off');
    else
        set(updraft_est_2,'Visible','On');
        set(updraft_est_2,'XData',x_up_est_2,'YData',y_up_est_2);
    end
    
    % update aircraft position
    set(position_plot,'XData',position_array(2,i),'YData',position_array(1,i));
    
    set(time_annotation,'String',sprintf('t=%d s',i-1));
    
    % update height
    addpoints(height_plot, i,-position_array(3,i));
    
    %drawnow 
     frame = getframe(gcf); %get frame
%      writeVideo(myVideo, frame);
    pause(0.1)
end
% close(myVideo)