function dist = compare(dcc_current, dcc_standard) %Nd = 4 as example
    % refer: https://www.mathworks.com/help/matlab/import_export/ways-to-import-text-files.html
    
    % input_seed = [x1, y1, z1, x2, y2, z2, x3, y3, z3]
    
    % interpolation: https://www.mathworks.com/help/matlab/ref/interp1.html
    
    fitted_dcc_current = interpolation_dcc(dcc_current, dcc_standard);
    
    % simple norm
    %norm(dcc_standard - fitted_dcc_current)
    
    temp_dif = dcc_standard - fitted_dcc_current; % error part of MSE
    temp_dif_sq = temp_dif.*temp_dif; %elementary wise multiplication
    sum_all_temp_dif_sq = sum(temp_dif_sq, 'all');
    
    size_temp_dif_sq = size(temp_dif_sq);
    divider_temp_dif_sq = size_temp_dif_sq(1) * size_temp_dif_sq(2);
    
    dist = sum_all_temp_dif_sq / divider_temp_dif_sq;
    
    % MSE
    
    %temp_table = temp_table_full(end,:);
    
    %table = [temp_table(1:3)',temp_table(4:6)',temp_table(7:9)'];
    
end
