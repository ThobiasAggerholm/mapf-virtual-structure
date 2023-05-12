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


// Max Pheromone map
    // auto const & pheromone_map_0 = aco.m_ant_systems[0].m_pheromones;
    // int pos = instance.m_start_locations[0];
    // max_path_0.push_back(pos);
    // while(pos != instance.m_goal_locations[0])
    // {
    //     Node const* curr = &instance.m_my_graph[pos];
    //     std::vector<int> ns = instance.get_neighbors(pos);
    //     double max_pheromone = 0;
    //     int next = ns[0];
    //     for(int i = 0; i < ns.size(); ++i)
    //     {
    //         Node const* cand = &instance.m_my_graph[ns[i]];
    //         if(cand->vertex_id == instance.m_goal_locations[0])
    //         {
    //             next = cand->vertex_id;
    //             break;
    //         }
    //         if(max_pheromone < pheromone_map_0.at(curr).at(cand))
    //         {
    //             max_pheromone = pheromone_map_0.at(curr).at(cand);
    //             next = cand->vertex_id;
    //         }
    //     }
    //     pos = next;
    //     max_path_0.push_back(pos);
    // }
    // instance.map_route_to_image("../output_data/mapf-map/my_map_ant_path_0_max.png", max_path_0);

    // std::vector<int> max_path_1;
    // max_path_1.reserve(aco.m_as_paths[1].size());

    // auto const & pheromone_map_1 = aco.m_ant_systems[1].m_pheromones;
    // pos = instance.m_start_locations[1];
    // max_path_1.push_back(pos);
    // while(pos != instance.m_goal_locations[1])
    // {
    //     Node const* curr = &instance.m_my_graph[pos];
    //     std::vector<int> ns = instance.get_neighbors(pos);
    //     double max_pheromone = 0;
    //     int next = ns[0];
    //     for(int i = 0; i < ns.size(); ++i)
    //     {
    //         Node const* cand = &instance.m_my_graph[ns[i]];
    //         if(cand->vertex_id == instance.m_goal_locations[1])
    //         {
    //             next = cand->vertex_id;
    //             break;
    //         }
    //         if(max_pheromone < pheromone_map_1.at(curr).at(cand))
    //         {
    //             max_pheromone = pheromone_map_1.at(curr).at(cand);
    //             next = cand->vertex_id;
    //         }
    //     }
    //     pos = next;
    //     max_path_1.push_back(pos);
    // }
    // instance.map_route_to_image("../output_data/mapf-map/my_map_ant_path_1_max.png", max_path_1);


//ELITE ANTS
    // std::vector<Ant> elite_ants;
    // elite_ants.reserve(n_ant_systems);

    //For each instance start goal pair run AStar and build ant for injection into AS population
    // AStar astar(instance);
    // for(int i = 0; i < instance.m_start_locations.size(); ++i)
    // {
    //     assert(instance.m_start_locations[i] != instance.m_goal_locations[i]);
    //     std::vector<Cell> cell_details;
    //     m_a_star.search(cell_details, instance.m_start_locations[i], instance.m_goal_locations[i]);
    //     std::vector<int> path = m_a_star.trace_path(cell_details, instance.m_goal_locations[i]); // Path is backtraced from goal to start

    //     // Build ant for injection ant with path into AS population
    //     elite_ants.push_back(Ant());
    //     Ant & ant = elite_ants.back();
    //     ant.reset();
    //     ant.move(&(*m_as_params.graph)[path.back()], 0);

    //     for(int i_path = path.size()-1; i_path > 0; --i_path)
    //     {
    //         int id_curr = path[i_path];
    //         int id_next = path[i_path - 1];
    //         Node const* curr = &(*m_as_params.graph)[id_curr];
    //         Node const* next = &(*m_as_params.graph)[id_next];

    //         double transition_cost = m_instance.m_my_graph.at(curr->vertex_id).edges.at(next);
    //         ant.move(next, transition_cost);
    //     }
    //     ant.return_home();
    // }        // Inject elite ants into AS population
        // m_ant_systems[i].inject_ant(elite_ants[i]);
            // repair();
    // for(int i = 0; i < n_ant_systems; ++i)
    // {
    //     m_ant_systems[i].compute_choice_information();
    // }