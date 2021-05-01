function [ ] = fuzz_fct_Test_Swarm_3d_IndepWaypoints(input_seed_matrix) %Nd = 4 as example

% input_seed_matrix
% x1 x2 x3
% y1 y2 y3
% z1 z2 z3
% input_seed_matrix(:,1)
% x1 
% y1 
% z1 


%% Test Swarm Trajectory in 3-D
% Author: Christian Howard
% Date   : 2/17/2015
% Info    : This is a script developed to show the swarm navigation
% algorithms in 3-D. This script also allows for the user to specify
% various waypoints so that they can maneuver around a space

clear all;
false = 0;
true = 1;
rng(1)

%% Setup the movie stuff
make_movie = false;
make_pot     = false; % make video showing potential field | Takes a LONG time to make, so usually set to false
writerObj = [];

%% Define the plane of points to evaluate potential function, if you are making video of potential function
x = linspace(0, 50, 100);
y = linspace(0, 50, 100);
z = 5;

[X,Y] = meshgrid(x,y);
[rx, cx] = size(X);
Z = zeros(rx, cx);

%% Setup the movie file and frame rate
if( make_movie )
    writerObj = VideoWriter('3D_test3.avi');
    writerObj.FrameRate = 15;
    open(writerObj);
end

% initial perspective in video
az = -60;
el = 20;

% final perspective in video
azf = -60;
elf = 20;

%%  Parameter specifications for drones
Nd = 1+3;                    % number of drones in swarm
ind_c = -1;                       % index for center/lead drone
radius = 2;                     % radius for drones and possibly obstacles
dims = [0, 50; 0, 50; 0, 50]; % first row is lower and upper x bounds
                                                  % second row is lower and upper y bounds
                                                  % third row is lower and upper z bounds
xc = [40, 40, 40]'; % initial location for central/lead drone
end_loc = [5, 5, 5]'; % Desired end location

%% Setup the waypoint object
dist_thresh = 2; % The distance threshold the swarm must be within of the waypoint
                           % to make the waypoint change to the new one
droneWaypoints = [];
copyWaypoints = [];

droneWaypoints_fake = [];
copyWaypoints_fake = [];

for i = 1:Nd
    wypt = Waypoints( dist_thresh );
    
    % Add waypoint structure to drone waypoint structure
    % The drone waypoint structure represents individual waypoints for each
    % drone, so they don't have to move in the same direction. 
    droneWaypoints(i).w = wypt;
    droneWaypoints_fake(i).w = wypt;
    
    wypt2 = Waypoints( dist_thresh ); % copy waypoints for use in plotting waypoints
    
    % Add waypoint structure to copy waypoint structure
    copyWaypoints(i).w = wypt2;
    copyWaypoints_fake(i).w = wypt2;
    
end



% Initialize the waypoints 
% For simplicity in this example, will initilize waypoint paths to be the
% same thing for each drone. 
pt1 = [15;50;20];
pt2 = [0; 50; 0];
pt3 = end_loc;

for i = 1:Nd
    
    droneWaypoints(i).w.addPoint( pt1 );
    droneWaypoints(i).w.addPoint( pt2 );
    droneWaypoints(i).w.addPoint( pt3 );
    
    copyWaypoints(i).w.addPoint( pt1 );
    copyWaypoints(i).w.addPoint( pt2 );
    copyWaypoints(i).w.addPoint( pt3 );

end


%% Parameter specifications for the obstacles
No = 4;
i = 1;
obst = [];
dims_o = [0, 10; 0, 10; 0, 10]; % first row is lower and upper x bounds
                                      % second row is lower and upper y bounds
% while( i <= No )
%     %randomly generate position of obstacle within bounds
%     pos = rand(3,1).*( dims_o(:,2)-dims_o(:,1) ) + dims_o(:,1);
%     disp("pos");
%     disp(pos);
%     disp(rand(3,1));
%     disp(( dims_o(:,2)-dims_o(:,1) ));
%     disp(dims_o(:,1));
%     % Add new obstacle to obstacle array obst
%     obst = [obst, Obstacle(pos,radius)];
% 
%     i = i + 1;
% end
% disp("obst");

pos1 = [7.5; 50; 10.0];
pos2 = [5.0; 150; 6.66];
pos3 = [7.5; 150; 10.0];
pos4 = [10.0; 150; 13.33];
% 
obst = [Obstacle(pos1,radius), Obstacle(pos2,radius), ...
    Obstacle(pos3,radius), Obstacle(pos4,radius)];


%% Create drones and obstacle arrays
i = 1;
drones = [];
while( i <= Nd )
    if( i == ind_c )
        % Add lead drone to drone array
        drones = [drones, Drone(radius, xc , 2)];
    else
        % Add a follower drone to drone array
        DEL = [xc - 10, xc + 10]; % make swarm initialize around lead drone randomly
        %pos = rand(3,1).*( DEL(:,2)-DEL(:,1) ) + DEL(:,1);
        pos = input_seed_matrix(:,i);
        drones = [drones, Drone(radius, pos, 1)];
    end
    i = i + 1;
end

%% Setup figure traits for the movie
close all;
h = figure('Position', [10, 10, 1000, 800]);
set(gca,'nextplot','replacechildren');
set(gcf,'Renderer','zbuffer');
dims2 = dims';

%% Do the initial drawing
i = 1;

figure(1)
N = 1;
if( make_pot )
    N = 2;
end


subplot(1,N,1)
while( i <=Nd ) % loop through drones
    drawObject(drones(i));
    if( i == 1 )
        hold on
    end
    i = i + 1;
end

i = 1;
while( i <= No ) % loop through obstacles
    drawObject(obst(i));
    i = i + 1;
end
hold off
grid on
view(az, el)
axis(dims2(:))
xpos = [];

%% Do iterations of the drone moving to final location
it = 1;
count = 0;
done = 0;
while( done == 0 )
    i = 1;
    
    previous_pos = drones(1).pos;
    
    % Compute new locations
    while( i <= Nd )
        pt = droneWaypoints(i).w.getWaypoint( drones(i).pos );
        drones(i).pos = drones(i).pos + GradientDescentUpdate(drones(i).pos, i, drones, obst, pt );

        if( count > 20 || it >= 150 )
            done = 1;
        end
        i = i + 1;
    end
    
    temp_1_pos = drones(1).pos;
    
    
    % 1
    % redefine
    drones_fake = drones;
    obst_fake = obst;
    temp_2_pos = drones(2).pos;
    
    pt_1 = droneWaypoints(1).w.getWaypoint( drones_fake(1).pos );
    temp_1_pt_1 = pt_1;
    
    
    
    drones_fake(2).pos = [999; 999; 999];
    fake_coor_d2 = previous_pos + GradientDescentUpdate(drones_fake(1).pos, 1, drones_fake, obst_fake, pt_1); % this is for drone1's coord
    drones_fake(2).pos = temp_2_pos;
    
    drones_fake(1).pos = temp_1_pos;
%     dlmwrite('fake_coor_d2.csv',[fake_coor_d2, temp_1_pos, drones_fake(1).pos, drones(1).pos] ,'delimiter',',','-append');

    % 2
    % redefine
    drones_fake = drones;
    obst_fake = obst;
    temp_3_pos = drones(3).pos;
    
    drones_fake(3).pos = [999; 999; 999];
    fake_coor_d3 = previous_pos + GradientDescentUpdate(drones_fake(1).pos, 1, drones_fake, obst_fake, pt_1);
    drones_fake(3).pos = temp_3_pos;
    
    drones_fake(1).pos = temp_1_pos;
    
    % 3
    % redefine
    drones_fake = drones;
    obst_fake = obst;
    temp_4_pos = drones(4).pos;
    
    drones_fake(4).pos = [999; 999; 999];
    fake_coor_d4 = previous_pos + GradientDescentUpdate(drones_fake(1).pos, 1, drones_fake, obst_fake, pt_1);
    drones_fake(4).pos = temp_4_pos;
    
    % 4
    % redefine
    drones_fake = drones;
    obst_fake = obst;
    temp_o_pos = obst(1).pos;
    
    obst_fake(1).pos = [999; 999; 999];
    fake_coor_o = previous_pos + GradientDescentUpdate(drones_fake(1).pos, 1, drones_fake, obst_fake, pt_1);
    obst_fake(1).pos = temp_o_pos;
    
    drones_fake(1).pos = temp_1_pos;
    
    
    
    
    % for waypoint
    
    
    
    
    
    
%     pt1_fake = drones(1).pos;
%     pt2_fake = drones(1).pos;
%     pt3_fake = end_loc;
% 
%     for i = 1:N
% 
%         droneWaypoints_fake(i).w.addPoint( pt1_fake );
%         droneWaypoints_fake(i).w.addPoint( pt2_fake );
%         droneWaypoints_fake(i).w.addPoint( pt3_fake );
% 
% %         copyWaypoints_fake(i).w.addPoint( pt1_fake );
% %         copyWaypoints_fake(i).w.addPoint( pt2_fake );
% %         copyWaypoints_fake(i).w.addPoint( pt3_fake );
% 
%     end
    
%     pt_1_fake = droneWaypoints_fake(1).w.getWaypoint( drones_fake(1).pos );
%     disp(pt_1_fake);
    fake_coor_dw = previous_pos + GradientDescentUpdate(drones_fake(1).pos, 1, drones_fake, obst_fake, drones_fake(1).pos); % this is for drone1's coord
    
%     pt_1_fake = temp_1_pt_1;
    
    
    
    
    
    drones(1).pos = temp_1_pos;
    
    delta_d2 = norm(drones(1).pos - fake_coor_d2);
    delta_d3 = norm(drones(1).pos - fake_coor_d3);
    delta_d4 = norm(drones(1).pos - fake_coor_d4);
    delta_o = norm(drones(1).pos - fake_coor_o);
    delta_w = norm(drones(1).pos - fake_coor_dw);
   
%     dlmwrite('delta.csv',[delta_d2, delta_d3, delta_d4, delta_o] ,'delimiter',',','-append');
   
    sum_delta = delta_d2 + delta_d3 + delta_d4 + delta_o + delta_w;
    
    dcc_d2 = delta_d2 / sum_delta;
    dcc_d3 = delta_d3 / sum_delta;
    dcc_d4 = delta_d4 / sum_delta;
    dcc_o = delta_o / sum_delta;
    dcc_w = delta_w / sum_delta;
    
   dlmwrite('dcc.csv',[dcc_d2, dcc_d3, dcc_d4, dcc_o, dcc_w] ,'delimiter',',','-append');

%     dist_1 = norm(drones(1).pos - pos1);
%     check_crash_1 = (norm(drones(1).pos - pos1) < 4.00);
%     % x, y, z, dist, t/f (crash?)
%     dlmwrite('drone_1_coord.csv',[drones(1).pos', dist_1, check_crash_1] ,'delimiter',',','-append');
    % Draw drones in new position
    i = 1;
    
    figure(1)
    subplot(1,N,1)
    
    while( i <= Nd ) % loop through drones
        drawObject(drones(i));
        if( i == 1 )
            hold on
        end
        i = i + 1;
    end
   
    i = 1;
    while( i <= Nd )
%         drawWaypoints( copyWaypoints(i).w );
        i = i + 1;
    end
    
    i = 1;
    while( i <= No ) % loop through obstacles
        drawObject(obst(i));
        i = i + 1;
    end
    hold off
    coef = it/100;
    
    %zoom(2)
    grid on
    axis(dims2(:))
    view(az + coef*(azf - az) , el + coef*(elf - el) )
    %rotate(h, [0, 0, 1], 1)
    pause(.01)
    
    % Add frame to movie, if you are wanting to make the movie
    if( make_movie && it > 3 )
        ind = 1;
        if( N > 1 )
            subplot(1, N, 2)
            ind = 1;
            for ii = 1:rx
                for jj = 1:cx
                    Z(ii, jj) = getTotalPotential( [X(ii,jj); Y(ii, jj); z(1)], ind, drones, obst, end_loc );
                end
            end
            
            surf(X,Y,Z)
            axis( [x(1), x(end), y(1), y(end)] )
        end
        
        
        frame = getframe(gcf);
%         writeVideo(writerObj,frame);
    end
    
    it = it + 1;
end

%% Close movie file, if you are recording a movie
if( make_movie )
    close(writerObj);
end

end