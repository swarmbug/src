

for order = 999
    disp('##########################################');
    fprintf('case order: %i  \n', order);
    disp("###### TIME start######");
    disp(datestr(now,'HH:MM:SS.FFF'));
    %initial value
    
    c_dist_thresh = 1;% 2;%	0.1		10
    c_obst_pot_c = 1;% 1000;%	200		2000
    c_mag = 1;% 10;%	2		20
    c_loc_pot_c = 1;% 	10;%	2		20
    c_d_pot_c = 1;% 1000;%	200		2000
    c_d_pot_out_r_c = 1;% 1000;%	200		2000
    
    num_sim = 2;
    
    coef_small = 0.2;
    coef_big = 8.0;
    
    switch(order)
        case 1
            c_dist_thresh	= coef_small;%	0.1		10
        case 2
            c_dist_thresh	= coef_big;%	0.1		10
        case 3
            c_obst_pot_c= coef_small;%	200		2000
        case 4
            c_obst_pot_c= coef_big;%	200		2000
        case 5
            c_mag	= coef_small;%	2		20
        case 6
            c_mag	= coef_big;%	2		20
        case 7
            c_loc_pot_c =	coef_small;%	2		20
        case 8
            c_loc_pot_c =	coef_big;%	2		20
        case 9
            c_d_pot_c	= coef_small;%	200		2000
        case 10
            c_d_pot_c	= coef_big;%	200		2000
        case 11
            c_d_pot_out_r_c = coef_small;%	200		2000
        case 12
            c_d_pot_out_r_c = coef_big;%	200		2000
        case 999
            disp('normal');
    end

   for idx_sim = 1 : num_sim
       
       disp('##########################################');
       fprintf('case order: %i  \n', order);
       fprintf('idx_sim: %i  \n', idx_sim);
       
       dlmwrite('coord.csv','order','delimiter',',','-append');
       dlmwrite('coord.csv',order,'delimiter',',','-append');
       dlmwrite('coord.csv','idx_sim','delimiter',',','-append');
       dlmwrite('coord.csv',idx_sim,'delimiter',',','-append');
       
       [obst, drones] = ccc_fct_profiling_Test_Swarm_3d_IndepWaypoints(c_dist_thresh, c_obst_pot_c, c_mag, c_loc_pot_c, c_d_pot_c, c_d_pot_out_r_c);
       
       disp("###### TIME end######");
       disp(datestr(now,'HH:MM:SS.FFF'));
       % x, y, z, dist, t/f (crash?)
       
       
        
   end
   
   
   
end

disp('all end');
disp('##########################################');
disp('##########################################');