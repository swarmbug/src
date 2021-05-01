function potential = FinalLocationPotential( drone_pos, final_pos, c_loc_pot_c )
    del = drone_pos-final_pos;
    r2 = dot(del,del);
    r = sqrt(r2);

    potential = c_loc_pot_c * 100*r*r;

end