max_iteration_exp = 100;

for idx = 1:max_iteration_exp

    
    % pick one from seed_cand
    current_seed = pick_one('seed_cand.txt');
    
    % current_seed
    % x1 x2 x3
    % y1 y2 y3
    % z1 z2 z3
    % input_seed_matrix(:,1)
    % x1 
    % y1 
    % z1 

    % execute the one instance
    % executeXXX(current_seed); % this store current dcc as file
    
    fuzz_fct_Test_Swarm_3d_IndepWaypoints(current_seed);

    % load current dcc that is stored in xxx folder
    current_dcc = load(current_dcc.txt);
    max_dist = 0;
    
    

    for i = 1:idx
        dist = compare(current_dcc, dcc_i); %in compare, interpolation is.

        if max_dist < dist
            max_dist = dist;
        end
    end

    if max_dist < threshold
        % ignore current seed
        % generate next seed from pool then put this into seed_cand
        next_seed = gen_next_seed(seed_pool.txt, 'big');

        put_into_seed(seed_cand.txt, next_seed);

    else

        put_into_seed(seed_pool.txt, current_seed);

        next_seed = gen_next_seed(seed_pool.txt, 'small');

        put_into_seed(seed_cand.txt, next_seed);


    end
    
    put_into_dcc(current_dcc); %it stores current_dcc in folder
    
    
end

