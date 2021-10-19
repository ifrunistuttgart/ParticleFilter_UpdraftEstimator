%% Short script to export filter result to .dat-File for plotting

%% export flight path
start_point = 1380;
end_point = 1700;
xy_position = [vehicle_position(start_point:end_point,2), vehicle_position(start_point:end_point,1)];

%% export particles 
at_time = 1492;
cluster_1 = [particle_array(2,[assigned_cluster(:,at_time)==1],at_time)', particle_array(1,[assigned_cluster(:,at_time)==1],at_time)'];
cluster_1_center = [filtered_state_array(2,1,at_time),filtered_state_array(1,1,at_time)];
cluster_1_radius = filtered_state_array(4,1,at_time);
noise_particles = [particle_array(2,[assigned_cluster(:,at_time)==0],at_time)', particle_array(1,[assigned_cluster(:,at_time)==0],at_time)'];

%% write data to .dat-Files
csvwrite('learning2soar_figures/flightpath.dat',xy_position);
csvwrite('learning2soar_figures/cluster_1_particles.dat',cluster_1);
csvwrite('learning2soar_figures/cluster_1_center.dat',cluster_1_center);
csvwrite('learning2soar_figures/cluster_1_radius.dat',cluster_1_radius);
csvwrite('learning2soar_figures/noise_particles.dat',noise_particles);