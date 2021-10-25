%% Short script to export filter result to .dat-File for plotting

%% export flight path
start_point = 1380;
end_point = 1700;
index = 0:(end_point-start_point);
xy_position = [vehicle_position(start_point:end_point,2), vehicle_position(start_point:end_point,1), index'];
alt_over_time = [index', -vehicle_position(start_point:end_point,3)];
%% export particles 
at_time = 1537;
cluster_1 = [particle_array(2,[assigned_cluster(:,at_time)==1],at_time)', particle_array(1,[assigned_cluster(:,at_time)==1],at_time)'];
cluster_1_center = [filtered_state_array(2,1,at_time),filtered_state_array(1,1,at_time)];
cluster_1_radius = filtered_state_array(4,1,at_time);
cluster_2 = [particle_array(2,[assigned_cluster(:,at_time)==2],at_time)', particle_array(1,[assigned_cluster(:,at_time)==2],at_time)'];
cluster_2_center = [filtered_state_array(2,2,at_time),filtered_state_array(1,2,at_time)];
cluster_2_radius = filtered_state_array(4,2,at_time);
noise_particles = [particle_array(2,[assigned_cluster(:,at_time)==0],at_time)', particle_array(1,[assigned_cluster(:,at_time)==0],at_time)'];

%% write data to .dat-Files
csvwrite('learning2soar_figures/flightpath.dat',xy_position);
csvwrite('learning2soar_figures/alt_over_time.dat',alt_over_time);
csvwrite('learning2soar_figures/cluster_1_particles.dat',cluster_1);
csvwrite('learning2soar_figures/cluster_1_center.dat',cluster_1_center);
csvwrite('learning2soar_figures/cluster_1_radius.dat',cluster_1_radius);
csvwrite('learning2soar_figures/cluster_2_particles.dat',cluster_2);
csvwrite('learning2soar_figures/cluster_2_center.dat',cluster_2_center);
csvwrite('learning2soar_figures/cluster_2_radius.dat',cluster_2_radius);
csvwrite('learning2soar_figures/noise_particles.dat',noise_particles);