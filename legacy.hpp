// Illustrate Pheromone map
    // int n_cols = instance.m_num_of_cols * 2 -1;
    // int n_rows = instance.m_num_of_rows * 2 -1;
    // std::vector<double> heat_map(n_cols * n_rows, 0);
    // for(int i = 0; i < instance.m_map_size; ++i)
    // {
    //     int x_org = instance.get_col_coordinate(i);
    //     int y_org = instance.get_row_coordinate(i);

    //     int x_org_nr = x_org + 1;
    //     int y_org_nr = y_org;
    //     int id_org_nr = instance.linearize_coordinate(y_org_nr, x_org_nr);

    //     int x_org_nd = x_org;
    //     int y_org_nd = y_org + 1;
    //     int id_org_nd = instance.linearize_coordinate(y_org_nd, x_org_nd);


    //     int x = x_org * 2;
    //     int y = y_org * 2;

    //     int x_nr = x + 1;
    //     int y_nr = y;
    //     int x_nd = x;
    //     int y_nd = y + 1;

    //     int id = n_cols * y + x;
    //     heat_map[id] = 0;

    //     if((y_nr < n_rows) && (x_nr < n_cols))
    //     {
    //         int id_nr = n_cols * y_nr + x_nr;
    //         double val_r = aco.m_ant_systems[0].m_pheromones[&instance.m_my_graph[i]][&instance.m_my_graph[id_org_nr]];
    //         heat_map[id_nr] = val_r;
    //     }
    //     if((y_nd < n_rows) && (x_nd < n_cols))
    //     {
    //         int id_nd = n_cols * y_nd + x_nd;
    //         double val_d = aco.m_ant_systems[0].m_pheromones[&instance.m_my_graph[i]][&instance.m_my_graph[id_org_nd]];
    //         heat_map[id_nd] = val_d;
    //     }
    // }
    // double heat_max = *std::max_element(heat_map.begin(), heat_map.end());
    // for(double & d : heat_map)
    //     d = d / heat_max;
    // save_to_heat_img("../h.png", n_cols, n_rows, heat_map);