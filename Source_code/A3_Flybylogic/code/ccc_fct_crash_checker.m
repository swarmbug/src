function res = ccc_fct_crash_checker(drone_coord)

res = 0;

x_coord = drone_coord(1,1);
y_coord = drone_coord(1,2);
z_coord = drone_coord(1,3);

if x_coord >= 1.0 && x_coord <= 1.3
    if y_coord >= 0.5 && y_coord <= 1.8
        if z_coord >= 0 && z_coord <= 1.5
        res = 1;
        end
    end
end

