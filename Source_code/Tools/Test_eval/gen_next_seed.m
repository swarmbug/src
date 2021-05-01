function table = gen_next_seed(seed_pool_txt, option) %Nd = 4 as example
    % refer: https://www.mathworks.com/help/matlab/import_export/ways-to-import-text-files.html
    
    % input_seed = [x1, y1, z1, x2, y2, z2, x3, y3, z3]
        
    temp_table_full = readmatrix(seed_pool_txt); %<- read last one column
    temp_table = temp_table_full(end,:);
    alpha_big = 10;
    beta_big = 2;
    
    alpha_small = 2;
    beta_small = 1;
    
    if strcmpi(option,'big')
        len_rand = alpha_big + beta_big * rand(3,1);
        theta_rand1 = 2*pi()*rand(3,1);
        theta_rand2 = 2*pi()*rand(3,1);
    else
        len_rand = alpha_small + beta_small * rand(3,1);
        theta_rand1 = 2*pi()*rand(3,1);
        theta_rand2 = 2*pi()*rand(3,1);
    end
    temp_len_mid = len_rand * cos(theta_rand2);
    new_coor1 = [temp_len_mid * cos(theta_rand1), temp_len_mid * sin(theta_rand1), len_rand * sin(theta_rand2)];
    
    if strcmpi(option,'big')
        len_rand = alpha_big + beta_big * rand(3,1);
        theta_rand1 = 2*pi()*rand(3,1);
        theta_rand2 = 2*pi()*rand(3,1);
    else
        len_rand = alpha_small + beta_small * rand(3,1);
        theta_rand1 = 2*pi()*rand(3,1);
        theta_rand2 = 2*pi()*rand(3,1);
    end
    temp_len_mid = len_rand * cos(theta_rand2);
    new_coor2 = [temp_len_mid * cos(theta_rand1), temp_len_mid * sin(theta_rand1), len_rand * sin(theta_rand2)];
    
    
    if strcmpi(option,'big')
        len_rand = alpha_big + beta_big * rand(3,1);
        theta_rand1 = 2*pi()*rand(3,1);
        theta_rand2 = 2*pi()*rand(3,1);
    else
        len_rand = alpha_small + beta_small * rand(3,1);
        theta_rand1 = 2*pi()*rand(3,1);
        theta_rand2 = 2*pi()*rand(3,1);
    end
    temp_len_mid = len_rand * cos(theta_rand2);
    new_coor3 = [temp_len_mid * cos(theta_rand1), temp_len_mid * sin(theta_rand1), len_rand * sin(theta_rand2)];
    
    %... add more for additional agent
        
    table = [temp_table(1:3)'+new_coor1,temp_table(4:6)'+new_coor2,temp_table(7:9)'+new_coor3];
    
end
