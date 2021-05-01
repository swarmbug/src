%% Clear console and workspace and add project root to path

close all;
clearvars -except app;

project_root = strcat(extractBefore(mfilename('fullpath'),mfilename),'../..');
addpath(genpath(project_root));


%% Simulation options

DRONE_TYPE = "quadcopter"; % swarming mode supports quadcopter and point_mass
ACTIVE_ENVIRONMENT = true;
DEBUG = false;
VIDEO = true;
CENTER_VIEW_ON_SWARM = false; 
SWARM_ALGORITHM = "olfati_saber"; % either vicsek or olfati_saber

if DEBUG || VIDEO
    results_dirname = strcat('results/results_swarm');
    date_string = datestr(now,'yyyy_mm_dd_HH_MM_SS');
    subfolder = strcat(erase(mfilename,"example_"), '_', date_string);
    results_dirname = strcat(results_dirname, '/', subfolder);
    if ~exist(results_dirname, 'dir')
        mkdir(results_dirname)
    end
end

fontsize = 12;


%% Get changes from GUI

if exist('app', 'var')
    % Simulation parameters
    p_sim.end_time = app.sim_time;
    
    % Drone parameters
    DRONE_TYPE = app.drone_type;
    
    % Swarming parameters
    SWARM_ALGORITHM = "olfati_saber";
    p_swarm.nb_agents = app.nb_agents;
    p_swarm.d_ref = app.d_ref;
    p_swarm.v_ref = app.v_ref;
       
    % Map parameters
    ACTIVE_ENVIRONMENT = app.active_environment;
    
    % Debug plot
    DEBUG = app.debug_plot;
end

if DRONE_TYPE == "point_mass"
   SWARM_VIEWER_TYPE = "agent";
elseif DRONE_TYPE == "quadcopter"
   SWARM_VIEWER_TYPE = "drone";
end


%% Call parameters files

run('param_sim');
run('param_battery');
run('param_physics');
if DRONE_TYPE == "fixed_wing" || DRONE_TYPE == "quadcopter"
    run('param_drone'); 
elseif DRONE_TYPE == "point_mass"
    run('param_drone'); 
end
run('param_map');
run('param_swarm');


%% Init Swarm object, Wind, Viewer and other variables

% Init swarm and set positions
swarm = Swarm();
swarm.algorithm = SWARM_ALGORITHM;

for i = 1 : p_swarm.nb_agents
    swarm.add_drone(DRONE_TYPE, p_drone, p_battery, p_sim, p_physics,...
         map);
    swarm.add_fake_drone1(DRONE_TYPE, p_drone, p_battery, p_sim, p_physics,...
         map); %(CJ) added
     swarm.add_fake_drone2(DRONE_TYPE, p_drone, p_battery, p_sim, p_physics,...
         map); %(CJ) added
     swarm.add_fake_drone3(DRONE_TYPE, p_drone, p_battery, p_sim, p_physics,...
         map); %(CJ) added
     swarm.add_fake_drone4(DRONE_TYPE, p_drone, p_battery, p_sim, p_physics,...
         map); %(CJ) added
     swarm.add_fake_drone5(DRONE_TYPE, p_drone, p_battery, p_sim, p_physics,...
         map); %(CJ) added
end


% IMPORTANT.
% number of drone is 3 for now.

% coor_d1 = readmatrix('data/1st_drone_agent_2_ran_gen.csv');
% coor_d2 = readmatrix('data/2nd_drone_agent_1_ran_gen.csv');
% coor_d3 = readmatrix('data/3rd_drone_agent_3_ran_gen.csv');
% 
% new_init_pos_ned = [coor_d1(1,:)',coor_d2(1,:)',coor_d3(1,:)'];


disp("p_swarm.Pos0");
disp(p_swarm.Pos0);
    
%%%%%%%%%%%%%%%% CONTROL PANNEL
p_swarm.randomtest = 0;
writeMode = 1;  % 1 is for Dcc value writing



if randomtest
    disp("p_swarm.Pos0");
    disp(p_swarm.Pos0);
    
    swarm.set_pos(app.new_init_pos_ned);
    
    disp("swarm.get_pos_ned()");
    disp(swarm.get_pos_ned());
else
    swarm.set_pos(p_swarm.Pos0);
    
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % for test
    % separately...
    

    p_swarm.c_vm  = app.c_vm; %(CJ) 3 -> 6, 0.6"

    p_swarm.a     = app.a; %(CJ) 1 -> 2, 0.2");

    p_swarm.b     = app.b; %(CJ) 5 -> 10, 2.5");

    p_swarm.delta = app.delta; %(CJ) 0.2 -> 0.04, 0.4");

    p_swarm.k     = app.k; %(CJ) 2 -> 4, 0.4");

    p_swarm.r0 = app.r0; %(CJ) 10 -> 20, 2 ::: 25 is proper fix!

    p_swarm.lambda = app.lambda; %(CJ) 1 -> 0.2

    p_swarm.c_pm_obs = app.c_pm_obs; %(CJ) 5 -> 10, 1


    p_swarm.max_a = app.max_a; %(CJ) 10 -> 20, 2

    p_swarm.max_v = app.max_v; %(CJ) 7 -> 14, 2
    
    p_swarm.r = app.r;
        
    p_swarm.d_ref = app.d_ref; %(CJ) 25 -> 50, 5
    
    p_swarm.v_ref = app.v_ref; %(CJ) 6 -> 12, 3
        
    p_swarm.max_neig = app.max_neig;


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    disp("###########################################");
    
    disp("p_swarm.c_vm  = app.c_vm; %(CJ) 3 -> 6, 0.6");
    disp(p_swarm.c_vm);
    disp("p_swarm.a     = app.a; %(CJ) 1 -> 2, 0.2");
    disp(p_swarm.a);
    disp("p_swarm.b     = app.b; %(CJ) 5 -> 10, 2.5");
    disp(p_swarm.b);
    disp("p_swarm.delta = app.delta; %(CJ) 0.2 -> 0.04, 0.4");
    disp(p_swarm.delta);
    disp("p_swarm.k     = app.k; %(CJ) 2 -> 4, 0.4");
    disp(p_swarm.k);
    disp("p_swarm.r0 = app.r0; %(CJ) 10 -> 20, 2 ::: 25 is proper fix!");
    disp(p_swarm.r0);
    disp("p_swarm.lambda = app.lambda; %(CJ) 1 -> 0.2");
    disp(p_swarm.lambda);
    disp("p_swarm.c_pm_obs = app.c_pm_obs; %(CJ) 5 -> 10, 1");
    disp(p_swarm.c_pm_obs);
    
    disp("p_swarm.max_a = app.max_a; %(CJ) 10 -> 20, 2");
    disp(p_swarm.max_a);
    disp("p_swarm.max_v = app.max_v; %(CJ) 7 -> 14, 2");
    disp(p_swarm.max_v);
    
    
    disp("p_swarm.r = 150; %(CJ)150 ->  300, 30");
    disp(p_swarm.r);
        
    disp("p_swarm.d_ref = 10; %(CJ) 10 -> 20, 2");
    disp(p_swarm.d_ref);
        
    disp("p_swarm.v_ref = 6; %(CJ) 6 -> 12, 3");
    disp(p_swarm.v_ref);
        
    disp("p_swarm.max_neig = 10; 10 -> 20, 2");
    disp(p_swarm.max_neig);
    
    
    disp("###########################################");
    
end

swarm.set_vel(p_swarm.Vel0);
if randomtest
    disp("swarm.get_vel_ned()");
    disp(swarm.get_vel_ned());
end




% Init wind
wind = zeros(6,1); % steady wind (1:3), wind gusts (3:6)

% Init video
if VIDEO    
    video_filename = strcat(erase(mfilename, "example_"), '_', date_string);
    video_filepath = strcat(results_dirname, '/', video_filename);
    video = VideoWriterWithRate(video_filepath, p_sim.dt_video);
end

% Init viewer
swarm_viewer = SwarmViewer(p_sim.dt_plot, CENTER_VIEW_ON_SWARM);
swarm_viewer.viewer_type = SWARM_VIEWER_TYPE;
states_handle = [];

%% Main simulation loop

goal_reached = false;
dlmwrite('position.csv','recording starts','delimiter',',','-append');
    
disp('Type CTRL-C to exit');
for time = p_sim.start_time:p_sim.dt:p_sim.end_time
%     fprintf('[%f]=>', time);
%     disp("swarm.get_vel_ned()");
%     disp(swarm.get_vel_ned());
    % Check if program terminated from GUI
    if exist('app', 'var')
        switch app.StartsimulationSwitch.Value
            case 'Off'
                close all;
                return;
        end
    end
    
    % Get change from GUI
    if exist('app', 'var')
        % Wind parameters
        % (CJ) wind_active = app.wind;
        wind_active = true;
        wind_gust_active = app.wind_gust;
        wind_level = app.wind_level;
        wind_gust_level = app.wind_gust_level;
        
        % Debug plot
        debug_plot = app.debug_plot;
        
        % Orientation of swarm migration
        orientation = app.orientation;
        p_swarm.u_ref = [-cosd(orientation), -sind(orientation), 0]';
        % disp(p_swarm.u_ref) e.g., 1, 0, 0
        % (CJ) OK: p_swarm.u_ref = [0, 0, 0]';
    end
    
    % disp(swarm.drones(1).get_state()) 
    % Compute velocity commands from swarming algorithm
    
    for i = 1:3
        swarm.fake_drones1(i).pos_ned = swarm.drones(i).pos_ned;
        swarm.fake_drones2(i).pos_ned = swarm.drones(i).pos_ned;
        swarm.fake_drones3(i).pos_ned = swarm.drones(i).pos_ned;
        swarm.fake_drones4(i).pos_ned = swarm.drones(i).pos_ned;
        swarm.fake_drones5(i).pos_ned = swarm.drones(i).pos_ned;
    end
    
    [~, collisions] = swarm.update_command(p_swarm, p_swarm.r_coll, p_sim.dt);
    
    % Update swarm states and plot the drones자
    swarm.update_state(wind, time); 
    
    
    N = swarm.get_pos_ned();
    fake_N1 = swarm.fake_get_pos_ned1();
    fake_N2 = swarm.fake_get_pos_ned2();
    fake_N3 = swarm.fake_get_pos_ned3();
    fake_N4 = swarm.fake_get_pos_ned4();
    fake_N5 = swarm.fake_get_pos_ned5();
    fake_d_1_pos_ned1 = swarm.fake_drones1(1).pos_ned;
    fake_d_1_pos_ned2 = swarm.fake_drones2(1).pos_ned;
    fake_d_1_pos_ned3 = swarm.fake_drones3(1).pos_ned;
    fake_d_1_pos_ned4 = swarm.fake_drones4(1).pos_ned;
    fake_d_1_pos_ned5 = swarm.fake_drones5(1).pos_ned;
    %fake_d_1_pos_ned(1:1) 
    main_agent = 2;
    
    %1 : 'contribution_score'; 2: 'dist'
    
    
    
    if writeMode == 1
        
        %dlmwrite('test.csv','This is drone_1','delimiter',',','-append'); 
        %dlmwrite('test.csv',N(:, main_agent),'delimiter',',','-append'); 
        
        %dlmwrite('test.csv','This is fake_drone_1','delimiter',',','-append'); 
        %dlmwrite('test.csv',fake_N(:, main_agent),'delimiter',',','-append'); 
        
        temp_norm1 = sqrt((N(1:1, main_agent) - fake_N1(1:1, main_agent))^2 ...
            + (N(2:2, main_agent) - fake_N1(2:2, main_agent))^2 ...
            + (N(3:3, main_agent) - fake_N1(3:3, main_agent))^2 );

        temp_norm2 = sqrt((N(1:1, main_agent) - fake_N2(1:1, main_agent))^2 ...
            + (N(2:2, main_agent) - fake_N2(2:2, main_agent))^2 ...
            + (N(3:3, main_agent) - fake_N2(3:3, main_agent))^2 );

        temp_norm3 = sqrt((N(1:1, main_agent) - fake_N3(1:1, main_agent))^2 ...
            + (N(2:2, main_agent) - fake_N3(2:2, main_agent))^2 ...
            + (N(3:3, main_agent) - fake_N3(3:3, main_agent))^2 );

        temp_norm4 = sqrt((N(1:1, main_agent) - fake_N4(1:1, main_agent))^2 ...
            + (N(2:2, main_agent) - fake_N4(2:2, main_agent))^2 ...
            + (N(3:3, main_agent) - fake_N4(3:3, main_agent))^2 );

        temp_norm5 = sqrt((N(1:1, main_agent) - fake_N5(1:1, main_agent))^2 ...
            + (N(2:2, main_agent) - fake_N5(2:2, main_agent))^2 ...
            + (N(3:3, main_agent) - fake_N5(3:3, main_agent))^2 );
        
        temp_sum = temp_norm1 + temp_norm2 + temp_norm3 + temp_norm4 + temp_norm5;
        
        temp_dcc1 = temp_norm1 / temp_sum;
        temp_dcc2 = temp_norm2 / temp_sum;
        temp_dcc3 = temp_norm3 / temp_sum;
        temp_dcc4 = temp_norm4 / temp_sum;
        temp_dcc5 = temp_norm5 / temp_sum;
        
        dlmwrite('Dcc.csv',[temp_dcc1, temp_dcc2, temp_dcc3, temp_dcc4, temp_dcc5] ,'delimiter',',','-append'); 
        

    end
    
    if writeMode == 1
        dlmwrite('dist.csv','time','delimiter',',','-append'); 
        dlmwrite('dist.csv',time,'delimiter',',','-append'); 
    end
    

   
   % Plot state variables for debugging
    if DEBUG
        swarm.plot_state(time, p_sim.end_time, ...
            1, p_sim.dt_plot, collisions, p_swarm.r_coll/2);
    end
    
    % Update video
    if VIDEO
        swarm_viewer.update(time, swarm, map);
        video.update(time, swarm_viewer.figure_handle);  
    end

end

if VIDEO
    video.close(); 
end

% Close all plots
close all;

if DEBUG && ~isempty(results_dirname)
    %% Plot offline viewer
    
    swarm_viewer_off = SwarmViewerOffline(p_sim.dt_video, ...
    CENTER_VIEW_ON_SWARM, p_sim.dt, swarm, map);


    %% Analyse swarm state variables
    
    time_history = p_sim.start_time:p_sim.dt:p_sim.end_time;
    pos_ned_history = swarm.get_pos_ned_history();
    pos_ned_history = pos_ned_history(2:end,:);
    vel_ned_history = swarm.get_vel_xyz_history();
    accel_history = [zeros(1, p_swarm.nb_agents*3); ...
        diff(vel_ned_history,1)/p_sim.dt];
    
    % Save workspace
    wokspace_path = strcat(results_dirname,'/state_var');
    save(wokspace_path,'time_history','pos_ned_history','vel_ned_history', ...
        'accel_history');
    
    % Plot state variables
    agents_color = swarm.get_colors();
    lines_color = [];

    plot_state_offline(time_history', pos_ned_history, vel_ned_history, ...
        accel_history, agents_color, p_swarm, map, fontsize, lines_color, ...
        results_dirname);

    
    %% Analyse performance
    
    % Compute swarm performance
    [safety, order, union, alg_conn, safety_obs] = ...
        compute_swarm_performance(pos_ned_history, vel_ned_history, ...
        p_swarm, results_dirname);
    
    % Plot performance
    [perf_handle] = plot_swarm_performance(time_history', safety, order, ...
        union, alg_conn, safety_obs, p_swarm, fontsize, results_dirname);
    
    
end


disp('Simulation completed successfully');
