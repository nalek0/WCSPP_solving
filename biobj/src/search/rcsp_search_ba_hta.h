#ifndef WC_BA_HTA_H
#define WC_BA_HTA_H

// Bi-directional A* Search (Based on BOBA*) using Heuristic Tuning with all expanded paths (HTA)
// for the Weight Constrined Shortest Path Problem (WCSPP)
// @author: sahmadi

template <class LABEL, class Q, class H>
class WC_BA_HTA
{
public:
    WC_BA_HTA(graph *G, graph *G_rev, experiment exp)
    {
        initialise_parameters(G->Num_vertices);

        // generate two candidate solution labels
        solution_label_f = new LABEL(COST_MAX, COST_MAX, SN_ID_MAX, EDGE_CAP_MAX, PATH_ARR_SIZE_MAX);
        solution_label_b = new LABEL(COST_MAX, COST_MAX, SN_ID_MAX, EDGE_CAP_MAX, PATH_ARR_SIZE_MAX);

        // Initialise global upper bounds
        cost1_ub = COST_MAX;
        cost2_ub = COST_MAX;

        // set timers
        double time_start; 
        double time_end;

        // set timer for the first round
        time_start = omp_get_wtime();
        
        ////////////////////////////////////////////////////
        // obtain lower bounds using A*
        #pragma omp parallel sections num_threads(2)
        {
            // obtaining backward heuristics by running forward A* on cost_1
            #pragma omp section 
            {Bounded_Astar_f1f2_Par<H>(G,     *open_1_b, h1_b, ub2_b, parent_b, exp.start, exp.goal, exp.constraint, expanded1, expanded2);}
            
            // obtaining forward heuristics by running backward A* on cost_2
            #pragma omp section 
            {Bounded_Astar_f2f1_Par<H>(G_rev, *open_2_f, h2_f, ub1_f, NULL, exp.goal, exp.start, exp.constraint, expanded2, expanded1);}
        }
        // Stop timer
        time_end = omp_get_wtime(); 
        double time_init = time_end - time_start;

        // Obtain full heuristics if the initial shortest path is not feasible
        if (ub2_b[exp.goal] > cost2_ub)
        {
            // set timer for the second round
            time_start = omp_get_wtime();
            #pragma omp parallel sections num_threads(2)
            {
                // obtaining forward heuristics by running backward informed A* on cost_1
                #pragma omp section 
                Bounded_Astar_Informed_Prune_f1f2(G_rev, *open_1_f, h1_f, ub2_f, h1_b, ub2_b, parent_f, exp.goal, exp.start, expanded1, expanded2);
            
                // obtaining backward heuristics by running forward informed A* on cost_2
                #pragma omp section 
                Bounded_Astar_Informed_Prune_f2f1(G, *open_2_b, h2_b, ub1_b, h2_f, ub1_f, parent_b, exp.start, exp.goal, expanded2, expanded1);
            }
            // Stop timer
            time_end = omp_get_wtime(); 
            time_init += time_end - time_start;
        }
        ////////////////////////////////////////////////////
        
        // delete auxiliary arrays and structures
        delete_aux_parameters();

        // initialising search time and memory use
        double time_search = 0;
        size_t search_mem = 0;

        // store budget
        cost_t budget = cost2_ub;

        // Check Case 1: the first shortest path is feasible
        if (ub2_b[exp.goal] <= cost2_ub)
        {
            solution_label_b->set_id(exp.goal);
            sol_cost1 = h1_b[exp.goal];
            sol_cost2 = ub2_b[exp.goal];
        }
        // Check Case 2: the second shortest path (after graph reduction) is feasible
        else if (ub2_f[exp.start] <= cost2_ub)
        {
            solution_label_f->set_id(exp.start);
            sol_cost1 = h1_f[exp.start];
            sol_cost2 = ub2_f[exp.start];
        }
        // Case 3: Searching via Constrained A*
        else
        {
            // initialising the label pool
            size_t label_pool_sz = 1024;
            label_pool<LABEL> Pool_f(label_pool_sz);
            label_pool<LABEL> Pool_b(label_pool_sz);

            // Initialising bucket queues based on the range of cost_1
            cost_t bucket_width = 1;
            Q open_f(bucket_width, h1_f[exp.start], cost1_ub); //forward queue
            Q open_b(bucket_width, h2_b[exp.goal], cost2_ub); // backward queue

            // set cost2 of the solution
            sol_cost2 = COST_MAX;

            // set number of expansions
            uint64_t num_expansions_f = 0;
            uint64_t num_expansions_b = 0;
            uint64_t total_expansions = 0;

            // set timer for the first round
            time_start = omp_get_wtime();
        
            #pragma omp parallel sections num_threads(2)
            {
                #pragma omp section
                {Main_search_f1f2(G, Pool_f, solution_label_f, solution_label_b, cost1_ub, cost2_ub, open_f
                , h1_f, ub1_f, h2_f, ub2_f
                , h1_b, ub1_b
                , ub2_b
                , g_min_f, g_min_b
                , exp.start, Paths_f
                , exp_path_cost_f, exp_path_cost_b
                , last_checked_id_f
                , num_expansions_f);}
                
                #pragma omp section
                {Main_search_f2f1(G_rev, Pool_b, solution_label_b, solution_label_f, cost2_ub, cost1_ub, open_b
                , h2_b, ub2_b, h1_b, ub1_b
                , h2_f, ub2_f
                , ub1_f
                , g_min_b, g_min_f
                , exp.goal, Paths_b
                , exp_path_cost_b, exp_path_cost_f
                , last_checked_id_b
                , num_expansions_b);}
            }

            // Stop search timer
            time_end = omp_get_wtime(); 
            time_search = time_end - time_start;
            
            // Calculate memory used for all labels + priority queue
            search_mem = Pool_f.mem() + Pool_b.mem() + open_f.mem() + open_b.mem();
            // Also add memory used for storing path costs in HTA 
            if (exp_path_cost_f && exp_path_cost_b)
            {
                for (sn_id_t index = 0; index < G->Num_vertices; index++)
                {
                    search_mem += exp_path_cost_f[index].capacity()*sizeof(std::pair<cost_t, cost_t>);
                    search_mem += exp_path_cost_b[index].capacity()*sizeof(std::pair<cost_t, cost_t>);
                }
            }

            // get the total forward+backward expansions
            total_expansions = num_expansions_f + num_expansions_b;
        }

        // Calculate memory used for saving all paths
        size_t paths_mem = this->mem(G->Num_vertices);

        // Store results
        results_rcsp<LABEL> res;
        res.obj1_lb = h1_b[exp.goal]; res.obj1_ub = ub1_f[exp.start];
        res.obj2_lb = h2_f[exp.start]; res.obj2_ub = ub2_b[exp.goal];
        res.sol_cost1 = sol_cost1; res.sol_cost2 = sol_cost2;
        res.budget = budget;
        res.time_elapsed_init_sec = time_init;
        res.time_elapsed_search_sec = time_search;
        res.memory_KB = (search_mem + paths_mem)/1024; // Memory of labels + Memory of paths stored
        

        // Print results
        res.print_stats(exp);
        
        // Capture and then print path details if requested
        #ifdef PATH
        if(exp.path)
        {
            // find the intermediate vertex
            sn_id_t link_vertex = solution_label_f->get_id() != SN_ID_MAX ? solution_label_f->get_id() : solution_label_b->get_id();
            // recover paths
            res.store_paths(G_rev, solution_label_f, link_vertex, Paths_f, parent_b, "f1f2", "forward");
            res.store_paths(G,     solution_label_b, link_vertex, Paths_b, parent_f, "f2f1", "backward");
            res.print_paths();
        }
        #else
        if(exp.path)
        {std::cerr<<"Please compile with 'make path' for path details.\n";}
        #endif

        
    }
    
    ~WC_BA_HTA()
    {
        delete [] h1_f; delete [] ub1_f;
        delete [] h2_f; delete [] ub2_f;
        delete [] h1_b; delete [] ub1_b;
        delete [] h2_b; delete [] ub2_b;
        delete [] g_min_f; delete [] g_min_b;
        delete [] exp_path_cost_f; delete [] exp_path_cost_b;
        delete [] last_checked_id_f; delete [] last_checked_id_b;
        #ifdef PATH
        delete [] Paths_f; delete [] Paths_b;
        delete [] parent_f; delete [] parent_b;
        #endif
    }

    size_t
    mem(sn_id_t num_vertices)
    {
        size_t bytes = 0;
        if (Paths_f)
        {
            for (sn_id_t index = 0; index < num_vertices; index++)
            {bytes += Paths_f[index].capacity()*sizeof(std::pair<edge_cap_t, path_arr_size>);}
        }
        if (Paths_b)
        {
            for (sn_id_t index = 0; index < num_vertices; index++)
            {bytes += Paths_b[index].capacity()*sizeof(std::pair<edge_cap_t, path_arr_size>);}
        }
        return bytes;
    }

//////////////////////////////////////////////////////
private:
    cost_t *h1_f, *ub1_f, *h2_f, *ub2_f, *g_min_f, *f_val_1_f, *f_val_2_f;
    cost_t *h1_b, *ub1_b, *h2_b, *ub2_b, *g_min_b, *f_val_1_b, *f_val_2_b;
    bool *expanded1, *expanded2;
    sn_id_t *parent_f, *parent_b;
    pqueue *open_1_f, *open_2_f, *open_1_b, *open_2_b;
    cost_t cost1_ub, cost2_ub, sol_cost1, sol_cost2;
    Parent_list *Paths_f, *Paths_b;
    LABEL *solution_label_f, *solution_label_b;
    // Vectors keeping track of path costs obtained for each vertex
    std::vector<std::pair<cost_t,cost_t>> *exp_path_cost_f;
    std::vector<std::pair<cost_t,cost_t>> *exp_path_cost_b;
            
    // integers keeping track of the last path id checked in HTA for each vertex
    uint64_t *last_checked_id_f;
    uint64_t *last_checked_id_b;

//////////////////////////////////////////////////////
    void
    initialise_parameters(sn_id_t num_vertices)
    {
        h1_f = new cost_t[num_vertices](); ub1_f = new cost_t[num_vertices]();
        h2_f = new cost_t[num_vertices](); ub2_f = new cost_t[num_vertices]();
        h1_b = new cost_t[num_vertices](); ub1_b = new cost_t[num_vertices]();
        h2_b = new cost_t[num_vertices](); ub2_b = new cost_t[num_vertices]();
        f_val_1_f = new cost_t[num_vertices](); f_val_2_f = new cost_t[num_vertices]();
        f_val_1_b = new cost_t[num_vertices](); f_val_2_b = new cost_t[num_vertices]();
        g_min_f = new cost_t[num_vertices](); g_min_b = new cost_t[num_vertices]();
        expanded1 = new bool[num_vertices](); expanded2 = new bool[num_vertices]();
        
        for (size_t id = 0; id < num_vertices; id++)
        {
            h1_f[id] = COST_MAX; h2_f[id] = COST_MAX;
            h1_b[id] = COST_MAX; h2_b[id] = COST_MAX;
            g_min_b[id] = COST_MAX; g_min_f[id] = COST_MAX;
        }

        open_1_f = new pqueue(1024, f_val_1_f, num_vertices);
        open_2_f = new pqueue(1024, f_val_2_f, num_vertices);
        open_1_b = new pqueue(1024, f_val_1_b, num_vertices);
        open_2_b = new pqueue(1024, f_val_2_b, num_vertices);

        // Vectors keeping track of path costs obtained for each vertex
        exp_path_cost_f = new std::vector<std::pair<cost_t,cost_t>>[num_vertices]();
        exp_path_cost_b = new std::vector<std::pair<cost_t,cost_t>>[num_vertices]();
            
        // integers keeping track of the last path id checked in HTA for each vertex
        last_checked_id_f = new uint64_t[num_vertices]();
        last_checked_id_b = new uint64_t[num_vertices]();

        #ifdef PATH
        Paths_f = new Parent_list[num_vertices]();
        Paths_b = new Parent_list[num_vertices]();
        parent_f = new sn_id_t[num_vertices]();
        parent_b = new sn_id_t[num_vertices]();
        #else
        Paths_f = 0; Paths_b = 0;
        #endif
    }
//////////////////////////////////////////////////////
    void
    delete_aux_parameters()
    {
        delete open_1_f; delete open_2_f;
        delete open_1_b; delete open_2_b;
        delete[] f_val_1_f; delete[] f_val_2_f;
        delete[] f_val_1_b; delete[] f_val_2_b;
        delete[] expanded1; delete[] expanded2;
    }
//////////////////////////////////////////////////////
    void *Main_search_f1f2(
        graph *G
        , label_pool<LABEL> &label_pool
        , LABEL *solution_label
        , LABEL *solution_label_op
        , cost_t &primary_ub, cost_t &secondary_ub
        , Q &open_
        , cost_t *h_pri, cost_t *ub_pri
        , cost_t *h_sec, cost_t *ub_sec
        , cost_t *h_pri_op, cost_t *ub_pri_op
        , cost_t *ub_sec_op
        , cost_t *g_min_sec, cost_t *g_min_sec_op
        , sn_id_t source
        , Parent_list *&Paths
        , std::vector<std::pair<cost_t,cost_t>> *exp_path_cost
        , std::vector<std::pair<cost_t,cost_t>> *exp_path_cost_op
        , uint64_t *last_checked_id
        , uint64_t &num_expansions)
    {
        // Load graph data
        edge_cap_t *Out_deg = G->Out_deg;
        edge_type **Edge_data = G->Edge_data;
        
        // generating the first label for the source node
        LABEL *current_label = label_pool.get_label();
        *current_label = LABEL(h_pri[source], 0, source, EDGE_CAP_MAX, PATH_ARR_SIZE_MAX);

        //inserting new labl into the queue
        open_.push(current_label);

        // Search while the queue is not empty
        while (open_.size())
        {
            // Extract (pop) the least-cost label
            LABEL *current_label = open_.pop(); 
            
            // recover vertex_id from the label
            sn_id_t current_vertex = current_label->get_id();

            // break the main while loop if the label violates the primary upper bound
            if (current_label->get_f_pri() > primary_ub)
            {break;}

            // extracting the g-values from the label
            cost_t current_label_g_pri = current_label->get_f_pri() - h_pri[current_vertex];
            cost_t current_label_g_sec = current_label->get_f_sec(); 

            if (current_label_g_sec >= g_min_sec[current_vertex] 
                || current_label_g_sec + h_sec[current_vertex] >  secondary_ub )
            {
                // dominated label, ignore
                label_pool.save_label(current_label);
                continue;
            }
            else
            {
                // Heuristic tuning heppens here
                if (current_label_g_sec == COST_MAX)
                {
                    // if this is the first visit, update the primal min cost
                    // Note that this changes the f2 values if tie breaker uses f2
                    h_pri_op[current_vertex] = current_label_g_pri;
                    ub_sec_op[current_vertex] = current_label_g_sec;
                }
                // undominated label, so update g2_min
                g_min_sec[current_vertex] = current_label_g_sec;
                
                // Heuristic Tuning using all expanded paths of the opposite direction
                size_t id = last_checked_id[current_vertex];
                size_t id_max = exp_path_cost_op[current_vertex].size();
                for (size_t last_path = id; last_path < id_max; last_path++)
                {
                    h_sec[current_vertex] = exp_path_cost_op[current_vertex].at(last_path).first;
                    if(current_label_g_pri + exp_path_cost_op[current_vertex].at(last_path).second < primary_ub)
                    break;

                    last_checked_id[current_vertex]++;
                }
                // Can also prune with the updated heuristic
                // if (current_label_g_sec + h_sec[current_vertex] >= secondary_ub)
                // {
                //     label_pool.save_label(current_label);
                //     continue;
                // }
                ////////////////////////////////////////////////////
            }

            // Check for a feasible solution: early solution strategy
            if (current_label_g_sec + ub_sec[current_vertex] <= secondary_ub)
            {
                // respecting the lexicographical order in solutions without tie breaker
                if (current_label->get_f_pri() < primary_ub || current_label_g_sec + ub_sec[current_vertex] < sol_cost2)
                {
                    // capture new costs
                    primary_ub = current_label->get_f_pri();
                    sol_cost1 = primary_ub;
                    sol_cost2 = current_label_g_sec + ub_sec[current_vertex];
                    secondary_ub = sol_cost2;
                    
                    // Store solution labels
                    *solution_label = *current_label;
                    *solution_label_op = LABEL(COST_MAX, COST_MAX, SN_ID_MAX, EDGE_CAP_MAX, PATH_ARR_SIZE_MAX);
                }

                // check if the complementary path is unique and does not offer alternative path
                if (h_pri[current_vertex] == ub_pri[current_vertex])
                {
                    // recycle the label as the solution is already stored
                    label_pool.save_label(current_label);
                    continue;
                }
            }
            // Now match with shortest path on cost_2
            else if (current_label_g_pri + ub_pri[current_vertex] < primary_ub)
            {
                // Update upper bound and reset the secondary cost of the best solution or sol_cost2
                primary_ub = current_label_g_pri + ub_pri[current_vertex];
                sol_cost2 = COST_MAX;
            }

            // keep a flag so as to know whether the label has been successfully expanded
            bool expanded = false;
            #ifdef PATH
            path_arr_size path_id = Paths[current_vertex].size(); // retrieve path id if needed (flag is set)
            #endif

            // num_expansions++;
            // Expand successors
            for (edge_cap_t edge_id = 0; edge_id < Out_deg[current_vertex]; edge_id++)
            {
                // recover successor edge and its data
                edge_type edge_data = Edge_data[current_vertex][edge_id];
                sn_id_t tail = edge_data.tail;
                
                // Check against pruning rule1: g_sec value of last expansion
                cost_t g_tail_sec = current_label_g_sec + edge_data.cost_2;
                if (g_tail_sec >= g_min_sec[tail]){continue;}
                
                // Check against pruning rule2: Upper bound on cost2 of tail
                if (g_tail_sec > ub_sec_op[tail]){continue;}

                // Check against pruning rule3: Upper bound on cost1 of tail
                cost_t g_tail_pri = current_label_g_pri + edge_data.cost_1;
                if (g_tail_pri > ub_pri_op[tail]){continue;}

                // Check against pruning rule2: secondary global upper bound
                cost_t f_tail_sec = g_tail_sec + h_sec[tail];
                if (h_sec[tail] > secondary_ub || f_tail_sec > secondary_ub){continue;}

                // Check against pruning rule3: primary global upper bound
                cost_t f_tail_pri = g_tail_pri + h_pri[tail];
                if (h_pri[tail] > primary_ub || f_tail_pri > primary_ub){continue;}

                // the extened path seems to be non-dominated, so generte a label with path info and add it to the queue
                // Using (f1,g2) with g2 as a tie breaker (if needed)
                LABEL *new_label = label_pool.get_label();

                #ifdef PATH
                *new_label = LABEL(f_tail_pri, g_tail_sec, tail, edge_data.tail_incoming, path_id); // keep backtracking information
                #else
                *new_label = LABEL(f_tail_pri, g_tail_sec, tail);
                #endif

                // expansion was successful, so set the flag
                expanded = true;
                // Add new label to queue
                open_.push(new_label);
            }

            if (expanded)
            {
                // Store path costs for HTA if the label is successfully expanded
                exp_path_cost[current_vertex].push_back(std::make_pair(current_label_g_pri, current_label_g_sec));

                // Store path if: the path flag is set and the label has been expanded
                #ifdef PATH 
                // CAUTION:: Max parent_array_id is UINT16_MAX
                Paths[current_vertex].push_back(std::make_pair(current_label->get_incoming_edge(), current_label->get_path_id()));
                #endif
            }
            // recycle the label
            label_pool.save_label(current_label);
        }
        secondary_ub = 0; // THIS WILL MAKE THE OPPOSITE SEARCH TERMINATE 
        return NULL;
    }
//////////////////////////////////////////////////////
    void *Main_search_f2f1(
        graph *G
        , label_pool<LABEL> &label_pool
        , LABEL *solution_label
        , LABEL *solution_label_op
        , cost_t &primary_ub, cost_t &secondary_ub
        , Q &open_
        , cost_t *h_pri, cost_t *ub_pri
        , cost_t *h_sec, cost_t *ub_sec
        , cost_t *h_pri_op, cost_t *ub_pri_op
        , cost_t *ub_sec_op
        , cost_t *g_min_sec, cost_t *g_min_sec_op
        , sn_id_t source
        , Parent_list *&Paths
        , std::vector<std::pair<cost_t,cost_t>> *exp_path_cost
        , std::vector<std::pair<cost_t,cost_t>> *exp_path_cost_op
        , uint64_t *last_checked_id
        , uint64_t &num_expansions)
    {
        // Load graph data
        edge_cap_t *Out_deg = G->Out_deg;
        edge_type **Edge_data = G->Edge_data;
        
        // generating the first label for the initial node
        LABEL *current_label = label_pool.get_label();
        *current_label = LABEL(h_pri[source], 0, source, EDGE_CAP_MAX, PATH_ARR_SIZE_MAX);

        //inserting new labl into the queue
        open_.push(current_label);

        // Search while the queue is not empty
        while (open_.size())
        {
            // Extract (pop) the least-cost label
            LABEL *current_label = open_.pop(); 
            
            // recover vertex_id from the label
            sn_id_t current_vertex = current_label->get_id();

            // break the main while loop if the label violates the primary upper bound
            if (current_label->get_f_pri() > primary_ub)
            {break;}

            // extracting the g-values from the label
            cost_t current_label_g_pri = current_label->get_f_pri() - h_pri[current_vertex];
            cost_t current_label_g_sec = current_label->get_f_sec();

            if (current_label_g_sec >= g_min_sec[current_vertex]
                || current_label_g_sec + h_sec[current_vertex] > secondary_ub)
            {
                // dominated label, ignore
                label_pool.save_label(current_label);
                continue;
            }
            else
            {
                // Heuristic tuning heppens here
                if (current_label_g_sec == COST_MAX)
                {
                    // if this is the first visit, update the primal min cost
                    // Note that this changes the f2 values if tie breaker uses f2
                    h_pri_op[current_vertex] = current_label_g_pri;
                    ub_sec_op[current_vertex] = current_label_g_sec;
                }
                // undominated label, so update g2_min
                g_min_sec[current_vertex] = current_label_g_sec;
                
                // Heuristic Tuning using all expanded paths of the opposite direction
                size_t id = last_checked_id[current_vertex];
                size_t id_max = exp_path_cost_op[current_vertex].size();
                for (size_t last_path = id; last_path < id_max; last_path++)
                {
                    h_sec[current_vertex] = exp_path_cost_op[current_vertex].at(last_path).first;
                    if(current_label_g_pri + exp_path_cost_op[current_vertex].at(last_path).second < primary_ub)
                    break;

                    last_checked_id[current_vertex]++;
                }
                // Can also prune with the updated heuristic
                // if (current_label_g_sec + h_sec[current_vertex] >= secondary_ub)
                // {
                //     label_pool.save_label(current_label);
                //     continue;
                // }
                ////////////////////////////////////////////////////
            }

            // Check for a feasible solution: early solution strategy
            if (current_label_g_sec + ub_sec[current_vertex] <= secondary_ub)
            {
                // respecting the lexicographical order in solutions without tie breaker
                if (current_label->get_f_pri() < sol_cost2 || current_label_g_sec + ub_sec[current_vertex] < secondary_ub)
                {
                    // capture new costs
                    secondary_ub = current_label_g_sec + ub_sec[current_vertex];
                    sol_cost1 = secondary_ub;
                    sol_cost2 = current_label->get_f_pri();
                    
                    // Store solution labels
                    *solution_label = *current_label;
                    *solution_label_op = LABEL(COST_MAX, COST_MAX, SN_ID_MAX, EDGE_CAP_MAX, PATH_ARR_SIZE_MAX);
                }

                // check if the complementary path is unique and does not offer alternative path
                if (h_pri[current_vertex] == ub_pri[current_vertex])
                {
                    // recycle the label as the solution is already stored
                    label_pool.save_label(current_label);
                    continue;
                }
            }
            // Now match with shortest path on cost_1
            else if (current_label_g_pri + ub_pri[current_vertex] <= primary_ub && current_label_g_sec + h_sec[current_vertex] < secondary_ub)
            {
                // Update upper bound and reset the secondary cost of the best solution or sol_cost2
                secondary_ub = current_label_g_sec + h_sec[current_vertex];
                sol_cost2 = COST_MAX;
            }

            // keep a flag so as to know whether the label has been successfully expanded
            bool expanded = false; 
            #ifdef PATH
            path_arr_size path_id = Paths[current_vertex].size(); // retrieve path id if needed (flag is set)
            #endif

            // num_expansions++;
            // Expand successors
            for (edge_cap_t edge_id = 0; edge_id < Out_deg[current_vertex]; edge_id++)
            {
                // recover successor edge and its data
                edge_type edge_data = Edge_data[current_vertex][edge_id];
                sn_id_t tail = edge_data.tail;
                
                // Check against pruning rule1: g_sec value of last expansion
                cost_t g_tail_sec = current_label_g_sec + edge_data.cost_1;
                if (g_tail_sec >= g_min_sec[tail]){continue;}

                // Check against pruning rule2: Upper bound on cost2 of tail
                if (g_tail_sec > ub_sec_op[tail]){continue;}

                // Check against pruning rule3: Upper bound on cost1 of tail
                cost_t g_tail_pri = current_label_g_pri + edge_data.cost_2;
                if (g_tail_pri > ub_pri_op[tail]){continue;}

                // Check against pruning rule2: secondary global upper bound
                cost_t f_tail_sec = g_tail_sec + h_sec[tail];
                if (h_sec[tail] > secondary_ub || f_tail_sec > secondary_ub){continue;}

                // Check against pruning rule3: primary global upper bound
                cost_t f_tail_pri = g_tail_pri + h_pri[tail];
                if (h_pri[tail] > primary_ub || f_tail_pri > primary_ub){continue;}

                // the extened path seems to be non-dominated, so generte a label with path info and add it to the queue
                // Using (f1,g2) with g2 as a tie breaker (if needed)
                LABEL *new_label = label_pool.get_label();

                #ifdef PATH
                *new_label = LABEL(f_tail_pri, g_tail_sec, tail, edge_data.tail_incoming, path_id); // keep backtracking information
                #else
                *new_label = LABEL(f_tail_pri, g_tail_sec, tail);
                #endif
                
                // expansion was successful, so set the flag
                expanded = true; 
                // Add new label to the queue
                open_.push(new_label);
            }
             
            if (expanded)   
            {
                // Store path if: the path flag is set and the label has been expanded
                exp_path_cost[current_vertex].push_back(std::make_pair(current_label_g_pri, current_label_g_sec));
                
                // Store path costs for HTA if the label is successfully expanded
                #ifdef PATH
                // CAUTION:: Max parent_array_id is UINT16_MAX
                Paths[current_vertex].push_back(std::make_pair(current_label->get_incoming_edge(), current_label->get_path_id()));
                #endif
            }
            // recycle the label
            label_pool.save_label(current_label);
        }
        secondary_ub = 0; // THIS WILL MAKE THE OPPOSITE SEARCH TERMINATE 
        return NULL;
    }
//////////////////////////////////////////////////////
//// CUSTOMISED A* SEARCHES
//////////////////////////////////////////////////////   
    template <class HEUR> void 
    Bounded_Astar_f1f2_Par(graph *G, pqueue &Queue
    , cost_t *h, cost_t *ub
    , sn_id_t *parent, sn_id_t source, sn_id_t target, double_t constraint, bool *expanded, bool *expanded_op)
    {
        edge_cap_t *Out_deg = G->Out_deg;
        edge_type **Edge_data = G->Edge_data;
        coord_pair *xy_co = G->xy_co;
        int32_t goal_x = xy_co[target].first;
        int32_t goal_y = xy_co[target].second;
        HEUR heuristic(goal_x, goal_y);

        // load cost array
        cost_t *f_vals = Queue.get_cost_array();
        
        h[source] = 0;
        ub[source] = 0;
        f_vals[source] = heuristic.get(xy_co[source].first, xy_co[source].second);
        #ifdef PATH
        parent[source] = SN_ID_MAX;
        #endif
        Queue.push(source);

        while (Queue.size() > 0)
        {
            sn_id_t current_vertex = Queue.pop();

            if ( f_vals[current_vertex] > cost1_ub)
                { break;}
            
            if (current_vertex == target && expanded_op[source])
            {
                // calculate budget
                cost1_ub = ub1_f[source];
                cost2_ub = std::floor(constraint*(ub2_b[target] - h2_f[source])/100) + h2_f[source];
                // If the shortest path is already optimal 
                if(ub2_b[target] <= cost2_ub)
                    {break;}
            }

            expanded[current_vertex] = true;
        
            for (edge_cap_t edge_id = 0; edge_id < Out_deg[current_vertex]; edge_id++)
            {
                edge_type edge_data = Edge_data[current_vertex][edge_id];
                sn_id_t tail = edge_data.tail;
                cost_t g1_tail = h[current_vertex] + edge_data.cost_1;
                cost_t g2_tail = ub[current_vertex] + edge_data.cost_2;

                if (g1_tail < h[tail] )
                {
                    if (h[tail] == COST_MAX)
                        f_vals[tail] = g1_tail + heuristic.get(xy_co[tail].first, xy_co[tail].second);
                    else
                        f_vals[tail] += g1_tail - h[tail];

                    h[tail] = g1_tail;
                    ub[tail] = g2_tail;
                    #ifdef PATH
                    parent[tail] = current_vertex;
                    #endif
                    
                    if (Queue.contains(tail))
                        Queue.decrease_key(tail);
                    else
                        Queue.push(tail);
                }
                else if (g1_tail == h[tail] && g2_tail < ub[tail])
                {
                    ub[tail] = g2_tail;
                    #ifdef PATH
                    parent[tail] = current_vertex;
                    #endif
                    
                    if (!Queue.contains(tail))
                        Queue.push(tail);
                }

            }
        }
    }
////////////////////////////////////////////////////// 
    template <class HEUR> void 
    Bounded_Astar_f2f1_Par(graph *G, pqueue &Queue
    , cost_t *h, cost_t *ub
    , sn_id_t *parent, sn_id_t source, sn_id_t target, double_t constraint, bool *expanded, bool *expanded_op)
    {
        edge_cap_t *Out_deg = G->Out_deg;
        edge_type **Edge_data = G->Edge_data;
        coord_pair *xy_co = G->xy_co;
        int32_t goal_x = xy_co[target].first;
        int32_t goal_y = xy_co[target].second;
        HEUR heuristic(goal_x, goal_y);

        // load cost array
        cost_t *f_vals = Queue.get_cost_array();
        
        h[source] = 0;
        ub[source] = 0;
        f_vals[source] = heuristic.get(xy_co[source].first, xy_co[source].second);
        Queue.push(source);

        while (Queue.size() > 0)
        {
            sn_id_t current_vertex = Queue.pop();

            if ( f_vals[current_vertex] > cost2_ub)
                { break;}

            if (current_vertex == target && expanded_op[source])
            {
                // calculate budget
                cost1_ub = ub1_f[target];
                cost2_ub = std::floor(constraint*(ub2_b[source] - h2_f[target])/100) + h2_f[target];
                // If the shortest path is already optimal 
                if(ub2_b[source] <= cost2_ub)
                    {break;}
            }
            
            expanded[current_vertex] = true;
        
            for (edge_cap_t edge_id = 0; edge_id < Out_deg[current_vertex]; edge_id++)
            {
                edge_type edge_data = Edge_data[current_vertex][edge_id];
                sn_id_t tail = edge_data.tail;
                cost_t g1_tail = h[current_vertex] + edge_data.cost_2;
                cost_t g2_tail = ub[current_vertex] + edge_data.cost_1;

                if (g1_tail < h[tail] )
                {
                    if (h[tail] == COST_MAX)
                        f_vals[tail] = g1_tail + heuristic.get(xy_co[tail].first, xy_co[tail].second);
                    else
                        f_vals[tail] += g1_tail - h[tail];

                    h[tail] = g1_tail;
                    ub[tail] = g2_tail;
                    
                    if (Queue.contains(tail))
                        Queue.decrease_key(tail);
                    else
                        Queue.push(tail);
                }
                else if (g1_tail == h[tail] && g2_tail < ub[tail])
                {
                    ub[tail] = g2_tail;
                    
                    if (!Queue.contains(tail))
                        Queue.push(tail);
                }

            }
        }
    }
////////////////////////////////////////////////////// 
    void 
    Bounded_Astar_Informed_Prune_f2f1(graph *G, pqueue &Queue
    , cost_t *h, cost_t *ub
    , cost_t *h_op, cost_t *ub_op
    , sn_id_t *parent, sn_id_t source, sn_id_t target, bool *expanded, bool *expanded_op)
    {
        edge_cap_t *Out_deg = G->Out_deg;
        edge_type **Edge_data = G->Edge_data;
        
        // load cost array
        cost_t *f_vals = Queue.get_cost_array();
        
        h[source] = 0;
        ub[source] = 0;
        f_vals[source] = h_op[source];
        #ifdef PATH
        parent[source] = SN_ID_MAX;
        #endif
        Queue.push(source);

        while (Queue.size() > 0)
        {
            sn_id_t current_vertex = Queue.pop();

            if ( f_vals[current_vertex] > cost2_ub)
            {break;}

            //skip those not expanded
            if (!expanded[current_vertex] || !expanded_op[current_vertex])
                {continue;}

            if (h1_b[current_vertex] + ub_op[current_vertex] < cost1_ub && 
                     ub2_b[current_vertex] + h_op[current_vertex] <= cost2_ub)
                cost1_ub = h1_b[current_vertex] + ub_op[current_vertex];
            else if (ub[current_vertex] + ub_op[current_vertex] < cost1_ub)
                cost1_ub = ub[current_vertex] + ub_op[current_vertex];
            
            for (edge_cap_t edge_id = 0; edge_id < Out_deg[current_vertex]; edge_id++)
            {
                edge_type edge_data = Edge_data[current_vertex][edge_id];
                sn_id_t tail = edge_data.tail;
                cost_t g1_tail = h[current_vertex] + edge_data.cost_2;
                cost_t g2_tail = ub[current_vertex] + edge_data.cost_1;

                //skip those not expanded
                if (h_op[tail] == COST_MAX)
                    {continue;}
                
                if (g1_tail < h[tail] )
                {
                    f_vals[tail] = g1_tail + h_op[tail];
                    h[tail] = g1_tail;
                    ub[tail] = g2_tail;
                    #ifdef PATH
                    parent[tail] = current_vertex;
                    #endif
                    
                    if (Queue.contains(tail))
                        Queue.decrease_key(tail);
                    else
                        Queue.push(tail);
                }
                else if (g1_tail == h[tail] && g2_tail < ub[tail])
                {
                    ub[tail] = g2_tail;
                    #ifdef PATH
                    parent[tail] = current_vertex;
                    #endif
                    
                    if (!Queue.contains(tail))
                        Queue.push(tail);
                }

            }
        }
    }
////////////////////////////////////////////////////// 
    void 
    Bounded_Astar_Informed_Prune_f1f2(graph *G, pqueue &Queue
    , cost_t *h, cost_t *ub
    , cost_t *h_op, cost_t *ub_op
    , sn_id_t *parent, sn_id_t source, sn_id_t target, bool *expanded, bool *expanded_op)
    {
        edge_cap_t *Out_deg = G->Out_deg;
        edge_type **Edge_data = G->Edge_data;
        
        // load cost array
        cost_t *f_vals = Queue.get_cost_array();
        
        h[source] = 0;
        ub[source] = 0;
        f_vals[source] = h_op[source];
        #ifdef PATH
        parent[source] = SN_ID_MAX;
        #endif
        Queue.push(source);

        while (Queue.size() > 0)
        {
            sn_id_t current_vertex = Queue.pop();

            if ( f_vals[current_vertex] > cost1_ub)
            {break;}

            // skip those not expanded
            if (!expanded[current_vertex] || !expanded_op[current_vertex])
                {continue;}
            
            // partial path matching
            if (ub[current_vertex] + ub_op[current_vertex] <= cost2_ub)
                cost1_ub = h[current_vertex] + h_op[current_vertex];
            else
            if ( h2_f[current_vertex] + ub_op[current_vertex] <= cost2_ub && 
                 ub1_f[current_vertex] + h_op[current_vertex] < cost1_ub   )
                cost1_ub = ub1_f[current_vertex] + h_op[current_vertex];

            for (edge_cap_t edge_id = 0; edge_id < Out_deg[current_vertex]; edge_id++)
            {
                edge_type edge_data = Edge_data[current_vertex][edge_id];
                sn_id_t tail = edge_data.tail;
                cost_t g1_tail = h[current_vertex] + edge_data.cost_1;
                cost_t g2_tail = ub[current_vertex] + edge_data.cost_2;

                //skip those not expanded
                if (h_op[tail] == COST_MAX)
                    {continue;}
                
                if (g1_tail < h[tail] )
                {
                    f_vals[tail] = g1_tail + h_op[tail];
                    h[tail] = g1_tail;
                    ub[tail] = g2_tail;
                    #ifdef PATH
                    parent[tail] = current_vertex;
                    #endif

                    if (Queue.contains(tail))
                        Queue.decrease_key(tail);
                    else
                        Queue.push(tail);
                }
                else if (g1_tail == h[tail] && g2_tail < ub[tail])
                {
                    ub[tail] = g2_tail;
                    #ifdef PATH
                    parent[tail] = current_vertex;
                    #endif

                    if (!Queue.contains(tail))
                        Queue.push(tail);
                }

            }
        }
    }
////////////////////////////////////////////////////// 

};


#endif