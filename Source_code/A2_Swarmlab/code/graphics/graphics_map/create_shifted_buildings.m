function map = create_shifted_buildings(map)
% CREATE_SHIFTED_BUILDINGS - Create the city buildings in a shifted grid.
% The heights are fixed.
%     disp('map.nb_blocks');
%     disp(map.nb_blocks);
    
    for i = 1:map.nb_blocks
        buildings_north(i) = 0.5*map.width/map.nb_blocks*(2*(i-1)+1);
    end
    buildings_north = buildings_north';

    map.buildings_east = [];
    map.buildings_north = [];
    for i = 1:map.nb_blocks
        if mod(i,2) == 1
            map.buildings_north = [map.buildings_north; repmat(buildings_north(i),map.nb_blocks,1)];
            map.buildings_east = [map.buildings_east; buildings_north];
        else
            map.buildings_north = [map.buildings_north; repmat(buildings_north(i),map.nb_blocks-1,1)];
            map.buildings_east = [map.buildings_east; ...
                buildings_north(1:(end-1))+(map.building_width+map.street_width)/2];
        end
    end
    %disp('map.buildings_east(1:11)');
    %disp(map.buildings_east(1:11));
    
    map.buildings_east = map.buildings_east(1:11); %(CJ) added for removing last row
    map.buildings_north = map.buildings_north(1:11); %(CJ) added for removing last row
    
    nb_buildings = length(map.buildings_east);
    map.buildings_heights = map.max_height*ones(nb_buildings,1);
    
%     disp('disp(length(map.buildings_east)');
%     disp(length(map.buildings_east));
%     disp('map.buildings_north');
% 	disp(map.buildings_north)
% 	disp('map.buildings_east');
% 	disp(map.buildings_east)
    
end