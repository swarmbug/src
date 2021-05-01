function fitted_dcc = interpolation_dcc(dcc_current, dcc_standard) %Nd = 4 as example
    % refer: https://www.mathworks.com/help/matlab/import_export/ways-to-import-text-files.html
    
    % input_seed = [x1, y1, z1, x2, y2, z2, x3, y3, z3]
    
    % interpolation: https://www.mathworks.com/help/matlab/ref/interp1.html
    length_current = length(dcc_current);
    length_standard = length(dcc_standard);
    
    fitted_dcc = [];
    
    x = 1:1:length_current;
    
    
    xq = 0:(length_current/length_standard):length_standard;
    
    for col_idx = 1:size(dcc_standard, 2)

        v = dcc_current(:,col_idx); % first column
        
        vq1 = interp1(x,v,xq,'pchip');
        
        fitted_dcc = [fitted_dcc, vq1']; %output of interpolation

    end
    
    %temp_table = temp_table_full(end,:);
    
    %table = [temp_table(1:3)',temp_table(4:6)',temp_table(7:9)'];
    
end
