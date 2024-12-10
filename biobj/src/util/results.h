#include <iomanip>
#include <algorithm>

template<class LABEL>
class results_biobj
{
public:
    results_biobj(){};
    double_t time_elapsed_init_sec;
    double_t time_elapsed_search_sec;
    cost_t obj1_lb;
    cost_t obj1_ub;
    cost_t obj2_lb;
    cost_t obj2_ub;
    uint32_t num_sols_b;
    uint32_t num_sols_f;
    size_t memory_KB;
    std::vector<std::pair<cost_t, cost_t>> solutions_costs;
    std::vector<std::vector<sn_id_t>> solutions_verticess;

    void print_stats(experiment exp)
    {
        // priniting stats
        std::cout << std::fixed;
        std::cout.precision(0);
        if (!exp.suppress_header)
            std::cerr << "alg\t  queue  map  start_id    goal_id   min_cost1   max_cost1   min_cost2   max_cost2   #sol   runtime(s)   search_malloc(KB)\n";
        std::cout << exp.alg_name << "  " << exp.queue_type << "  " << exp.map_name << " "
                  << std::setw(8) << exp.start + 1 << "  "
                  << std::setw(8) << exp.goal + 1 << "  "
                  << std::setw(8) << obj1_lb << "  "
                  << std::setw(8) << obj1_ub << "  "
                  << std::setw(9) << obj2_lb << "  "
                  << std::setw(9) << obj2_ub << "  "
                    // << std::setw(9) << num_sols_f << "  "
                    // << std::setw(9) << num_sols_b << "  "
                  << std::setw(9) << num_sols_f + num_sols_b << "  ";

        std::cout.precision(6);
        std::cout
            // << std::setw(9) << time_elapsed_init_sec << "  "
            // << std::setw(9) << time_elapsed_search_sec << "  "
        << std::setw(11) << time_elapsed_init_sec + time_elapsed_search_sec << "  ";
        std::cout.precision(0);
        std::cout
            << std::setw(9) << memory_KB
            << std::endl;
    }

    ////////////////////////////////////////
    virtual void
    print_paths( )
    {
        // Now print the path details
        for (size_t index = 0; index < solutions_costs.size(); index++)
        {
            std::vector<sn_id_t> path_vertices = solutions_verticess.at(index);
            std::cout << std::setfill('-') << std::setw(80) << "-" << std::endl;
            std::cout << "Path #" << index + 1;
            std::cout << " with " << path_vertices.size() << " nodes ->";
            std::cout << "  cost1: " << (solutions_costs.at(index)).first;
            std::cout << "  cost2: " << (solutions_costs.at(index)).second << std::endl;
            std::cout << "Full path: [" << path_vertices[0] + 1;
            for (uint i = 1; i < path_vertices.size(); i++)
            {
                std::cout << "," << path_vertices[i] + 1;
            }
            std::cout << "]" << std::endl;
        }
    }

    ////////////////////////////////////////
    void store_paths(linkedlist<LABEL *> Solutions, graph *G_rev
            , Parent_list *Paths, sn_id_t *parent_array
            , cost_t *h, cost_t *ub
            , std::string order, std::string dir)
    {
        edge_type **Edge_data = G_rev->Edge_data;

        LABEL *solution = Solutions.front();
        while (solution)
        {
            // first recover the first segement via the solution label
            std::vector<sn_id_t> path_vertices;

            sn_id_t current_vertex = solution->get_id();
            path_arr_size path_id = solution->get_path_id();
            edge_cap_t incoming_link = solution->get_incoming_edge();

            // find the actual (f1,f2) costs
            cost_t path_cost1 = 0;
            cost_t path_cost2 = 0;

            // just follow the path ids stored in the arrays
            while (true)
            {
                if (incoming_link == EDGE_CAP_MAX)
                {
                    break;
                }
                edge_type edge_data = Edge_data[current_vertex][incoming_link];
                path_cost1 += Edge_data[current_vertex][incoming_link].cost_1;
                path_cost2 += Edge_data[current_vertex][incoming_link].cost_2;

                // Reading from the node of opposite side
                current_vertex = edge_data.tail;
                incoming_link = Paths[current_vertex][path_id].first;
                path_id = Paths[current_vertex][path_id].second;

                path_vertices.push_back(current_vertex);
            }

            // reverse the nodes to to achieve start to link node
            std::reverse(path_vertices.begin(), path_vertices.end());

            current_vertex = solution->get_id();
            // match with best dist

            if (order == "f1f2")
            {
                path_cost1 += h[current_vertex];
                path_cost2 += ub[current_vertex];
            }
            else
            {
                path_cost2 += h[current_vertex];
                path_cost1 += ub[current_vertex];
            }

            // Now recover the second segement via the shortest path
    
            while (current_vertex != SN_ID_MAX)
            {
                path_vertices.push_back(current_vertex);
                current_vertex = parent_array[current_vertex];
            }

            // reverse vertex orders for the backward direction
            if (dir == "backward")
            {
                std::reverse(path_vertices.begin(), path_vertices.end());
            }

            solutions_costs.push_back(std::make_pair(path_cost1, path_cost2));
            solutions_verticess.push_back(path_vertices);

            solution = solution->get_next();
        }

        // reorder solutions
        if (dir == "forward")
        {
            std::reverse(solutions_verticess.begin(), solutions_verticess.end());
            std::reverse(solutions_costs.begin(), solutions_costs.end());
        }
    }
};
////////////////////////////////////////////////////////////
template<class LABEL>
class results_rcsp
{
public:
    results_rcsp(){};
    double_t time_elapsed_init_sec;
    double_t time_elapsed_search_sec;
    cost_t obj1_lb;
    cost_t obj1_ub;
    cost_t obj2_lb;
    cost_t obj2_ub;
    cost_t budget;
    cost_t sol_cost1;
    cost_t sol_cost2;
    std::vector<sn_id_t> path_vertices;
    
    size_t memory_KB;
    
    void print_stats(experiment exp)
    {
        // priniting stats
        std::cout << std::fixed;
        std::cout.precision(0);
        if (!exp.suppress_header)
            std::cerr << "alg\t  queue  map  start_id  goal_id  constraint  min_cost1  min_cost2  budget  sol_cost1  sol_cost2  runtime(s)  search_malloc(KB)\n";
        std::cout << exp.alg_name << "  " << exp.queue_type << "  "  << exp.map_name << " "
                  << std::setw(8) << exp.start + 1 << "  "
                  << std::setw(8) << exp.goal + 1 << "  "
                  << std::setw(8) << exp.constraint << "  "
                  << std::setw(8) << obj1_lb << "  "
                  << std::setw(8) << obj2_lb << "  "
                  << std::setw(9) << budget << "  "
                  << std::setw(9) << sol_cost1 << "  "
                  << std::setw(9) << sol_cost2 << "  ";

        std::cout.precision(6);
        std::cout
            // << std::setw(9) << time_elapsed_init_sec << "  "
            // << std::setw(9) << time_elapsed_search_sec << "  "
        << std::setw(11) << time_elapsed_init_sec + time_elapsed_search_sec << "  ";
        std::cout.precision(0);
        std::cout
            << std::setw(9) << memory_KB
            << std::endl;
    }

    ////////////////////////////////////////
    virtual void
    print_paths()
    {
        // Now print the path details
        std::cout << std::setfill('-') << std::setw(80) << "-" << std::endl;
        std::cout << "Path with " << path_vertices.size() << " nodes";
        std::cout << " and  cost1: " << sol_cost1;
        std::cout << "  cost2: " << sol_cost2 << "\n";
        if (path_vertices.size() < 1) return;
        std::cout << "Full path: [" << path_vertices[0] + 1;
        for (uint i = 1; i < path_vertices.size(); i++)
        {
            std::cout << "," << path_vertices[i] + 1;
        }
        std::cout << "]" << std::endl;
    }

    ////////////////////////////////////////
    void store_paths(graph *G_rev, LABEL * solution, sn_id_t link_vertex,  Parent_list *Paths, sn_id_t *parent_array, std::string order, std::string dir)
    {
        edge_type **Edge_data = G_rev->Edge_data;

        if (solution->get_id() != SN_ID_MAX)
        {
            // recover one segement via the solution label (compact approach)
            sn_id_t current_vertex = solution->get_id();
            path_arr_size path_id = solution->get_path_id();
            edge_cap_t incoming_link = solution->get_incoming_edge();

            // find the actual (f1,f2) costs
            cost_t path_cost1 = 0;
            cost_t path_cost2 = 0;

            // just follow the path ids stored in the arrays
            while (true)
            {
                if (incoming_link == EDGE_CAP_MAX)
                {
                    break;
                }
                edge_type edge_data = Edge_data[current_vertex][incoming_link];

                if (order == "f1f2")
                {
                    path_cost1 += edge_data.cost_1;
                    path_cost2 += edge_data.cost_2;
                }
                else
                {
                    path_cost2 += edge_data.cost_1;
                    path_cost1 += edge_data.cost_2;
                }
                // Reading from the node of opposite side
                current_vertex = edge_data.tail;
                incoming_link = Paths[current_vertex][path_id].first;
                path_id = Paths[current_vertex][path_id].second;

                path_vertices.push_back(current_vertex);
            }

            // std::cerr<<path_cost1 << " "<< path_cost2<< std::endl;

            current_vertex = solution->get_id();
        }
        else
        {
            // Now recover the second segement via the shortest path
            // Ignore the link vertex
            sn_id_t current_vertex = parent_array[link_vertex];

            while (current_vertex != SN_ID_MAX)
            {
                path_vertices.push_back(current_vertex);
                current_vertex = parent_array[current_vertex];
            }

        }

        // reorder solutions
        if (dir == "forward")
        {
            std::reverse(path_vertices.begin(), path_vertices.end());
            // Now add back the link vertex
            path_vertices.push_back(link_vertex);
        }
    }
};
