function put_into_seed(csv_file_to_store, input_seed) %Nd = 4 as example
    % refer: https://www.mathworks.com/help/matlab/import_export/ways-to-import-text-files.html
    
    % input_seed = [x1, y1, z1, x2, y2, z2, x3, y3, z3]
    
    dlmwrite(csv_file_to_store,input_seed,'delimiter',',');

    
    %temp_table_full = readmatrix(input_seed); %<- read last one column
    %temp_table = temp_table_full(end,:);
    
    %table = [temp_table(1:3)',temp_table(4:6)',temp_table(7:9)'];
    
end
