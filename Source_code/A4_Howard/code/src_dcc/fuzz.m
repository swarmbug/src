max_iteration_exp = 3;

for idx = 1:max_iteration_exp

    
    % pick one from seed_cand
    current_seed = pick_one('seed_cand.csv');
    
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
    
    fuzz_fct_Test_Swarm_3d_IndepWaypoints(idx, current_seed);

    % load current dcc that is stored in xxx folder
    path = "dcc_storage/" + idx + "/dcc.csv";
    current_dcc = load_dcc(path);
    max_dist = 0;
    threshold = 0.4;
    

    for i = 1:idx
        path = "data/" + idx + "/dcc.csv";
        dcc_i = load_dcc(path);
        
        dist = compare(current_dcc, dcc_i); %in compare, interpolation is.

        if max_dist < dist
            max_dist = dist;
        end
    end

    if max_dist < threshold
        % ignore current seed
        % generate next seed from pool then put this into seed_cand
        next_seed = gen_next_seed('seed_pool.csv', 'big');

        put_into_seed('seed_cand.csv', next_seed);

    else

        put_into_seed('seed_pool.csv', current_seed);

        next_seed = gen_next_seed('seed_pool.csv', 'small');

        put_into_seed('seed_cand.csv', next_seed);


    end
    
    put_into_dcc(idx, current_dcc); %it stores current_dcc in folder
    
    
end

