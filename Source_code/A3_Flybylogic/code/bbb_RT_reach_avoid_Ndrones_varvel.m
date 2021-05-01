%% start reach avoid example for a multiquads with eth tracker++



for idx_sim = 1 : 1000
    fprintf('this is %i th iteration \n', idx_sim);
    
    % Later, for recording whether crash or not, 
    % In this time, record all traj at once, then check whether there is
    % crash or not.
%     fid = fopen('crash_rt.txt', 'at') ;
%     fprintf(fid, 'this is %i th iteration \n', idx_sim) ;
%     fclose(fid) ;

    coor_d1 = readmatrix('data/randomized_a1.csv');
    coor_d2 = readmatrix('data/randomized_a2.csv');
    coor_d3 = readmatrix('data/randomized_a3.csv');
    coor_d4 = readmatrix('data/randomized_a4.csv');
    
    temp = [coor_d1(idx_sim,:)',coor_d2(idx_sim,:)',coor_d3(idx_sim,:)',coor_d4(idx_sim,:)'];
    
    disp("Inserted coordinates:");
    disp(temp);


%     temp =
% 
%    -1.2205    1.4403   -0.2671    0.2994
%    -1.1952   -1.2953    1.4854    1.2279
%     0.6557    0.4277    1.4206    1.3124

% temp(1,:)
% 
% ans =
% 
%    -1.2205    1.4403   -0.2671    0.2994

    current_xx = temp(1,:);
    current_yy = temp(2,:);
    current_zz = temp(3,:);
    
    [xx_1, yy_1, zz_1] = bbb_fct_rt_reach_avoid_Ndrones_varvel(N_drones, current_xx, current_yy, current_zz, does_obstacle_exist, (3 * .751));
    
    if(1)
    dlmwrite('Result_RT.csv','idx_sim' ,'delimiter',',','-append'); %1:1 로하니까 하나씩 만나오는구
    dlmwrite('Result_RT.csv',idx_sim,'delimiter',',','-append'); %1:1 로하니까 하나씩 만나오는구
    dlmwrite('Result_RT.csv','This is xx' ,'delimiter',',','-append'); %1:1 로하니까 하나씩 만나오는구
    dlmwrite('Result_RT.csv',xx_1','delimiter',',','-append'); %1:1 로하니까 하나씩 만나오는구
    dlmwrite('Result_RT.csv','This is yy' ,'delimiter',',','-append'); %1:1 로하니까 하나씩 만나오는구
    dlmwrite('Result_RT.csv',yy_1','delimiter',',','-append'); %1:1 로하니까 하나씩 만나오는구
    dlmwrite('Result_RT.csv','This is zz' ,'delimiter',',','-append'); %1:1 로하니까 하나씩 만나오는구
    dlmwrite('Result_RT.csv',zz_1','delimiter',',','-append'); %1:1 로하니까 하나씩 만나오는구   
    end

end