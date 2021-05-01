

for order = 37%31:31
    disp('##########################################');
    fprintf('case order: %i  \n', order);
    disp("###### TIME start######");
    disp(datestr(now,'HH:MM:SS.FFF'));
    %initial value
    c_max_per_axis = 1.0;
    c_max_vel = 1.0;
    c_max_accl = 1.0;
    c_M1 = 1.0;
    c_K1_T = 1.0;
    c_aa = 1.0;
    c_bb = 1.0;
    c_cc = 1.0;
    c_tp1 = 1.0;
    c_tp2 = 1.0;
    c_t_prime = 1.0;
    c_C = 1.0;
    c_C1 = 1.0; %not used
    c_C2 = 1.0;
    c_Ib = 1.0;
    c_Jb = 1.0;

    c_da = 1.0;
    c_dv = 1.0;
    c_Ja = 1.0;
    c_Ia = 1.0;
    
    c_da = 1.0;
    c_dv = 1.0;
            
        
    
    num_sim = 100;
    
    switch(order)
        case 1
            c_max_per_axis = 0.2;
        case 2
            c_max_per_axis = 2.0;
        case 3
            c_max_vel = 0.2; % error
        case 4
            c_max_vel = 2.0; % infeasible
        case 5
            c_max_accl = 0.2;
        case 6
            c_max_accl = 2.0;
        case 7
            c_M1 = 0.2;
            dlmwrite('res.csv','rt = c_M1' ,'delimiter',',','-append'); %1:1 로하니까 하나씩 만나오는구
        
        case 8
            c_M1 = 2.0;
        case 9
            c_K1_T = 0.2;
        case 10
            c_K1_T = 2.0;
        case 11
            c_aa = 0.2;
        case 12
            c_aa = 2.0;
        case 13
            c_bb = 0.2;
        case 14
            c_bb = 2.0;
        case 15
            c_cc = 0.2;
        case 16
            c_cc = 2.0;
        case 17
            c_tp1 = 0.2;
        case 18
            c_tp1 = 2.0;
        case 19
            c_tp2 = 0.2;
        case 20
            c_tp2 = 2.0;
        case 21
            c_t_prime = 0.2;
        case 22
            c_t_prime = 2.0;
            
            
            %%%%%%%
        case 23
            c_C = 0.2;
            dlmwrite('res.csv','rt = c_C' ,'delimiter',',','-append'); %1:1 로하니까 하나씩 만나오는구
        case 24
            c_C = 2.0;
%         case 25
%             c_C1 = 0.2; %not used
%         case 26
%             c_C1 = 2.0; %not used
        case 25
            c_C2 = 0.2;
        case 26
            c_C2 = 2.0;
        case 27
            c_Ib = 0.2;
        case 28
            c_Ib = 2.0; % error
        case 29
            c_Jb = 0.2;
        case 30
            c_Jb = 2.0; %error
        
        case 31
            c_da = 0.2;
            
        case 32
            c_da = 2.0;
            
        case 33
            c_dv = 0.2;
            
        case 34
            c_dv = 2.0;
            
        case 35
            c_Ja = 0.2;
            
        case 36 % error
            c_Ja = 2.0;
            
        case 37
            c_Ia = 0.2;
            dlmwrite('res.csv','rt = c_Ia' ,'delimiter',',','-append'); %1:1 로하니까 하나씩 만나오는구

            
        case 38
            c_Ia = 2.0;
        
        case 39
            c_da = 0.2;
        
        case 40
            c_da = 2.0;
        
        case 41
            c_dv = 0.2;
        
        case 42
            c_dv = 2.0;
        
            
        case 43
            disp("normal");
    end

   for idx_sim = 1 : num_sim
       disp('##########################################');
       fprintf('case order: %i  \n', order);
       fprintf('idx_sim: %i  \n', idx_sim);
       disp("###### TIME start idx_sim ######");
       disp(datestr(now,'HH:MM:SS.FFF'));
       
       [xx, yy, zz] = ccc_fct_profiling_reach_avoid_Ndrones_varvel(...
        c_max_per_axis, c_max_vel, c_max_accl, c_M1, c_K1_T, c_aa, c_bb, c_cc, ...
        c_tp1, c_tp2, c_t_prime, c_C, c_C1, c_C2, c_Ib, c_Jb, c_Ja, c_Ia, c_da, c_dv);
        
        dlmwrite('res.csv','idx_sim' ,'delimiter',',','-append'); %1:1 로하니까 하나씩 만나오는구
        dlmwrite('res.csv',idx_sim ,'delimiter',',','-append'); %1:1 로하니까 하나씩 만나오는구
%         dlmwrite('original_coor.csv','This is xx' ,'delimiter',',','-append'); %1:1 로하니까 하나씩 만나오는구
%         dlmwrite('original_coor.csv',xx,'delimiter',',','-append'); %1:1 로하니까 하나씩 만나오는구
%         dlmwrite('original_coor.csv','This is yy' ,'delimiter',',','-append'); %1:1 로하니까 하나씩 만나오는구
%         dlmwrite('original_coor.csv',yy,'delimiter',',','-append'); %1:1 로하니까 하나씩 만나오는구
%         dlmwrite('original_coor.csv','This is zz' ,'delimiter',',','-append'); %1:1 로하니까 하나씩 만나오는구
%         dlmwrite('original_coor.csv',zz,'delimiter',',','-append'); %1:1 로하니까 하나씩 만나오는구
        
        for tick = 1:length(xx)
            drone_1 = [xx(tick,1), yy(tick,1), zz(tick,1)];
            drone_2 = [xx(tick,2), yy(tick,2), zz(tick,2)];
            drone_3 = [xx(tick,3), yy(tick,3), zz(tick,3)];
            drone_4 = [xx(tick,4), yy(tick,4), zz(tick,4)];

            f_drone_1 = [drone_1, ccc_fct_crash_checker(drone_1)];
            f_drone_2 = [drone_2, ccc_fct_crash_checker(drone_2)];
            f_drone_3 = [drone_3, ccc_fct_crash_checker(drone_3)];
            f_drone_4 = [drone_4, ccc_fct_crash_checker(drone_4)];

            final_data = [f_drone_1, f_drone_2, f_drone_3, f_drone_4];

            dlmwrite('res.csv',final_data ,'delimiter',',','-append'); %1:1 로하니까 하나씩 만나오는구
        end
   end
   
   
   
end

disp('all end');
disp('##########################################');
disp('##########################################');