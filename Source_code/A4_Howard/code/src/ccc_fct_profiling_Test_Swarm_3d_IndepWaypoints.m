function [obst, drones] = ccc_fct_profiling_Test_Swarm_3d_IndepWaypoints(c_dist_thresh, c_obst_pot_c, c_mag, c_loc_pot_c, c_d_pot_c, c_d_pot_out_r_c)  %% Test Swarm Trajectory in 3-D
% Author: Christian Howard
% Date   : 2/17/2015
% Info    : This is a script developed to show the swarm navigation
% algorithms in 3-D. This script also allows for the user to specify
% various waypoints so that they can maneuver around a space

% clear all;
false = 0;
true = 1;
% rng(1)
visual = 0;

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
Nd = 1+19;                    % number of drones in swarm
ind_c = -1;                       % index for center/lead drone
radius = 2;                     % radius for drones and possibly obstacles
dims = [0, 50; 0, 50; 0, 50]; % first row is lower and upper x bounds
                                                  % second row is lower and upper y bounds
                                                  % third row is lower and upper z bounds
xc = [40, 40, 40]'; % initial location for central/lead drone
end_loc = [5, 5, 5]'; % Desired end location

%% Setup the waypoint object
dist_thresh = c_dist_thresh * 2; % The distance threshold the swarm must be within of the waypoint
                           % to make the waypoint change to the new one
droneWaypoints = [];
copyWaypoints = [];

for i = 1:Nd
    wypt = Waypoints( dist_thresh );
    
    % Add waypoint structure to drone waypoint structure
    % The drone waypoint structure represents individual waypoints for each
    % drone, so they don't have to move in the same direction. 
    droneWaypoints(i).w = wypt;
    
    wypt2 = Waypoints( dist_thresh ); % copy waypoints for use in plotting waypoints
    
    % Add waypoint structure to copy waypoint structure
    copyWaypoints(i).w = wypt2;
    
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
pos2 = [0.5; 35; 0.0];
pos3 = [2.0; 15; 5.0];
pos4 = [10.0; 5; 5.0];
% 
obst = [Obstacle(pos1,radius, c_obst_pot_c), Obstacle(pos2,radius, c_obst_pot_c), ...
    Obstacle(pos3,radius, c_obst_pot_c), Obstacle(pos4,radius, c_obst_pot_c)];


%% Create drones and obstacle arrays
i = 1;
drones = [];
while( i <= Nd )
    if( i == ind_c )
        % Add lead drone to drone array
        drones = [drones, Drone(radius, xc , 2)];
    else
        % Add a follower drone to drone array
        DEL = [xc - 20, xc + 20]; % make swarm initialize around lead drone randomly
        pos = rand(3,1).*( DEL(:,2)-DEL(:,1) ) + DEL(:,1);
        drones = [drones, Drone(radius, pos, 1, c_d_pot_c, c_d_pot_out_r_c)];
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
if visual
    grid on
    view(az, el)
    axis(dims2(:))
end


xpos = [];

%% Do iterations of the drone moving to final location
it = 1;
count = 0;
done = 0;
while( done == 0 )
    i = 1;
    
    % Compute new locations
    while( i <= Nd )
        pt = droneWaypoints(i).w.getWaypoint( drones(i).pos );
        drones(i).pos = drones(i).pos + GradientDescentUpdate(drones(i).pos, i, drones, obst, pt, c_mag, c_loc_pot_c );
        
        
        
        if( count > 20 || it >= 150 )
            done = 1;
        end
        i = i + 1;
    end
    
    %%%%%% WRITE 
    % all drones <--> obstacle 1
    dist_1 = norm(drones(1).pos - obst(1).pos);
   dist_2 = norm(drones(2).pos - obst(1).pos);
   dist_3 = norm(drones(3).pos - obst(1).pos);
   dist_4 = norm(drones(4).pos - obst(1).pos);
    
   crash_thr = 2.00;
   check_crash_1 = (dist_1 < crash_thr);
   check_crash_2 = (dist_2 < crash_thr);
   check_crash_3 = (dist_3 < crash_thr);
   check_crash_4 = (dist_4 < crash_thr);

   % x, y, z, dist, t/f (crash?)
   dlmwrite('coord.csv',[drones(1).pos', drones(2).pos', drones(3).pos', drones(4).pos',...
       dist_1, dist_2, dist_3, dist_4, ...
       check_crash_1, check_crash_2, check_crash_3, check_crash_4] ,'delimiter',',','-append');

    
    % Draw drones in new position
    i = 1;
    
    
    if (visual)
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
            drawWaypoints( copyWaypoints(i).w );
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
    %     pause(.01)

        % Add frame to movie, if you are wanting to make the movie
        if( make_movie && it > 3 )
            ind = 1;
            if( N > 1 )
                subplot(1, N, 2)
                ind = 1;
                for ii = 1:rx
                    for jj = 1:cx
                        Z(ii, jj) = getTotalPotential( [X(ii,jj); Y(ii, jj); z(1)], ind, drones, obst, end_loc, c_loc_pot_c );
                    end
                end

                surf(X,Y,Z)
                axis( [x(1), x(end), y(1), y(end)] )
            end


            frame = getframe(gcf);
    %         writeVideo(writerObj,frame);
        end
    end
    
    
    it = it + 1;
end

%% Close movie file, if you are recording a movie
% if( make_movie )
%     close(writerObj);
% end