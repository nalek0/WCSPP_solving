#ifndef WC_A_H
#define WC_A_H

// Uni-directional A* Search
// for the Weight Constrined Shortest Path Problem (WCSPP)
// @author: sahmadi

template <class LABEL, class Q, class H>
class WC_A
{
public:
    WC_A(graph *G, graph *G_rev, experiment exp)
    {
        initialise_parameters(G->Num_vertices);

        // generate two candidate solution labels
        solution_label_f = new LABEL(COST_MAX, COST_MAX, SN_ID_MAX, EDGE_CAP_MAX, PATH_ARR_SIZE_MAX);
        solution_label_b = new LABEL(COST_MAX, COST_MAX, SN_ID_MAX, EDGE_CAP_MAX, PATH_ARR_SIZE_MAX);

        // start heusristic search time
        timer mytimer;
        mytimer.start();

        ////////////////////////////////////////////////////
        // obtain lower bounds using A* - SWITCH BASED ON CONSTRAINT
        if (exp.constraint > 30)
        {
            // execute a point-to-point backward A* from scratch on cost1 and keep track of expanded vertices via expanded1
            Bounded_Astar_f1f2<H>(G_rev, *open_1_f, h1_f, ub2_f, parent_f, exp.goal, exp.start, h1_f[exp.start], true, expanded1);
            // execute a point-to-point backward A* from scratch on cost2
            Bounded_Astar_f2f1<H>(G_rev, *open_2_f, h2_f, ub1_f, NULL, exp.goal, exp.start, h2_f[exp.start], true, NULL);
        }
        else
        {
            // execute a point-to-point backward A* from scratch on cost1 
            Bounded_Astar_f1f2<H>(G_rev, *open_1_f, h1_f, ub2_f, parent_f, exp.goal, exp.start, h1_f[exp.start], true, NULL);
            // execute a point-to-point backward A* from scratch on cost2 and keep track of expanded vertices via expanded1
            Bounded_Astar_f2f1<H>(G_rev, *open_2_f, h2_f, ub1_f, NULL, exp.goal, exp.start, h2_f[exp.start], true, expanded1);
        }
        // calculate budget and global upper bounds
        cost_t cost1_ub = ub1_f[exp.start];
        cost_t cost2_ub = (exp.constraint*(ub2_f[exp.start] - h2_f[exp.start])/100.0) + h2_f[exp.start];
        
        // Obtain full heuristics if the initial shortest path is not feasible
        if (ub2_f[exp.start] > cost2_ub)
        {
            if (exp.constraint > 30)
            {
                // resume the point-to-point backward A* on cost1 and keep track of expanded vertices via expanded1
                Bounded_Astar_f1f2<H>(G_rev, *open_1_f, h1_f, ub2_f, parent_f, exp.goal, exp.start, cost1_ub, false, expanded1);
                // resume the point-to-point backward A* on cost2 and prune vertices not expanded in cost1 search via expanded1
                Bounded_Astar_Prune_f2f1<H>(G_rev, *open_2_f, h2_f, ub1_f, NULL, exp.goal, exp.start, cost2_ub, false, NULL, expanded1);
            }
            else
            {
                // resume the point-to-point backward A* on cost2 and keep track of expanded vertices via expanded1
                Bounded_Astar_f2f1<H>(G_rev, *open_2_f, h2_f, ub1_f, NULL, exp.goal, exp.start, cost2_ub, false, expanded1);
                // resume the point-to-point backward A* on cost1 and prune vertices not expanded in cost1 search via expanded1
                Bounded_Astar_Prune_f1f2<H>(G_rev, *open_1_f, h1_f, ub2_f, parent_f, exp.goal, exp.start, cost1_ub, false, NULL, expanded1);
            }
        }
        ////////////////////////////////////////////////////
        
        // capture heusristic search time
        mytimer.stop();
        double time_init = mytimer.elapsed_time_second();
        
        // delete auxiliary arrays and structures
        delete_aux_parameters();

        // initialising search time and memory use
        double time_search = 0;
        size_t search_mem = 0;

        // to track the number of expansions
        uint64_t num_exp = 0;

        // Check Case 1: the first shortest path is feasible
        if (ub2_f[exp.start] <= cost2_ub)
        {
            solution_label_f->set_id(exp.start);
            cost1_ub = h1_f[exp.start];
            sol_cost2 = ub2_f[exp.start];
        }
        else
        {
            // initialising the label pool
            size_t label_pool_sz = 1024;
            label_pool<LABEL> Pool_f(label_pool_sz);

            // Initialising bucket queues based on the range of cost_1
            cost_t bucket_width = exp.nruns;//1;
            Q open_f(bucket_width, h1_f[exp.start], cost1_ub); //forward queue

            // set cost2 of the solution
            sol_cost2 = COST_MAX;

            //reset timer for the main search
            mytimer.reset();
            mytimer.start();
            
            // Execute the constrained search
            Main_search_f1f2(G, Pool_f, solution_label_f, cost1_ub, cost2_ub, open_f, h1_f, ub1_f, h2_f, ub2_f, g_min_f, exp.start, Paths_f, num_exp);

            // Stop timer and read memory
            mytimer.stop();
            time_search = mytimer.elapsed_time_second();
            
            // Calculate memory used for all labels + priority queue
            search_mem = Pool_f.mem() + open_f.mem();

        }

        // Calculate memory used for saving all paths
        size_t paths_mem = this->mem(G->Num_vertices);

        // Store results
        results_rcsp<LABEL> res;
        res.obj1_lb = h1_f[exp.start]; res.obj1_ub = ub1_f[exp.start];
        res.obj2_lb = h2_f[exp.start]; res.obj2_ub = ub2_f[exp.start];
        res.sol_cost1 = cost1_ub; res.sol_cost2 = sol_cost2;
        res.budget = cost2_ub;
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
            sn_id_t link_vertex = solution_label_f->get_id();
            // recover paths
            res.store_paths(G_rev, solution_label_f, link_vertex, Paths_f, NULL, "f1f2", "forward");
            res.store_paths(G,     solution_label_b, link_vertex, NULL, parent_f, "f1f2", "backward");
            res.print_paths();
        }
        #else
        if(exp.path)
        {std::cerr<<"Please compile with 'make path' for path details.\n";}
        #endif
    }
    
    ~WC_A()
    {
        delete [] h1_f; delete [] ub1_f;
        delete [] h2_f; delete [] ub2_f;
        delete [] g_min_f;
        #ifdef PATH
        delete [] Paths_f;
        delete [] parent_f;
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
        return bytes;
    }

    //////////////////////////////////////////////////////
private:
    cost_t *h1_f, *ub1_f, *h2_f, *ub2_f, *g_min_f, *f_val_1, *f_val_2;
    cost_t sol_cost2;
    bool *expanded1;
    sn_id_t *parent_f;
    pqueue *open_1_f, *open_2_f;
    Parent_list *Paths_f;
    LABEL *solution_label_f, *solution_label_b;

//////////////////////////////////////////////////////
    void
    initialise_parameters(sn_id_t num_vertices)
    {
        h1_f = new cost_t[num_vertices](); ub1_f = new cost_t[num_vertices]();
        h2_f = new cost_t[num_vertices](); ub2_f = new cost_t[num_vertices]();
        f_val_1 = new cost_t[num_vertices](); f_val_2 = new cost_t[num_vertices]();
        g_min_f = new cost_t[num_vertices]();
        expanded1 = new bool[num_vertices]();
        
        for (size_t id = 0; id < num_vertices; id++)
        {
            g_min_f[id] = COST_MAX;
            h1_f[id] = COST_MAX;
            h2_f[id] = COST_MAX;
        }

        open_1_f = new pqueue(1024, f_val_1, num_vertices);
        open_2_f = new pqueue(1024, f_val_2, num_vertices);
        
        #ifdef PATH
        Paths_f = new Parent_list[num_vertices]();
        parent_f = new sn_id_t[num_vertices]();
        #else
        Paths_f = 0;
        #endif
    }
//////////////////////////////////////////////////////
    void
    delete_aux_parameters()
    {
        delete open_1_f;
        delete open_2_f;
        delete[] f_val_1;
        delete[] f_val_2;
        delete[] expanded1;
    }
//////////////////////////////////////////////////////
    void *Main_search_f1f2(
        graph *G
        , label_pool<LABEL> &label_pool
        , LABEL *&solution_label
        , cost_t &primary_ub, cost_t &secondary_ub
        , Q &open_
        , cost_t *h_pri, cost_t *ub_pri
        , cost_t *h_sec, cost_t *ub_sec
        , cost_t *g_min_sec
        , sn_id_t source
        , Parent_list *&Paths
        , uint64_t &num_exp)
    {
        // Load graph data
        edge_cap_t *Out_deg = G->Out_deg;
        edge_type **Edge_data = G->Edge_data;

        // generating the first label for the initial node
        LABEL *current_label = label_pool.get_label();
        *current_label = LABEL(h_pri[source], h_sec[source], source, EDGE_CAP_MAX, PATH_ARR_SIZE_MAX);

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
            cost_t current_label_g_sec = current_label->get_f_sec() - h_sec[current_vertex];

            if (current_label_g_sec >= g_min_sec[current_vertex])
            {
                // dominated label, ignore
                label_pool.save_label(current_label);
                continue;
            }
            else
            {
                // undominated label, so update g2_min
                g_min_sec[current_vertex] = current_label_g_sec;
            }

            // Check for a feasible solution: early solution strategy
            if (current_label_g_sec + ub_sec[current_vertex] <= secondary_ub)
            {
                // respecting the lexicographical order in solutions without tie breaker
                if (current_label->get_f_pri() < primary_ub || current_label_g_sec + ub_sec[current_vertex] < sol_cost2)
                {
                    // capture new costs
                    primary_ub = current_label->get_f_pri();
                    sol_cost2 = current_label_g_sec + ub_sec[current_vertex];

                    // Store solution labels
                    *solution_label = *current_label;
                    // *solution_label_op = LABEL(COST_MAX, COST_MAX, SN_ID_MAX, EDGE_CAP_MAX, PATH_ARR_SIZE_MAX);
                }

                // check if the complementary path is unique and does not offer alternative path
                if (h_pri[current_vertex] == ub_pri[current_vertex])
                {
                    // recycle the label as the solution is already stored
                    label_pool.save_label(current_label);
                    return NULL;
                }
            }
            // Now match with shortest path on cost_2
            else if (current_label_g_pri + ub_pri[current_vertex] < primary_ub)
            {
                // Update upper bound and reset the secondary cost of the best solution or sol_cost2
                primary_ub = current_label_g_pri + ub_pri[current_vertex];
                sol_cost2 = COST_MAX;
            }

            #ifdef PATH
            bool expanded = false; // keep a flag so as to know whether the label has been expanded
            path_arr_size path_id = Paths[current_vertex].size(); // retrieve path id if needed (flag is set)
            #endif
            
            // num_exp++;

            // Expand successors
            for (edge_cap_t edge_id = 0; edge_id < Out_deg[current_vertex]; edge_id++)
            {
                // recover successor edge and its data
                edge_type edge_data = Edge_data[current_vertex][edge_id];
                sn_id_t tail = edge_data.tail;
                
                // Check against pruning rule1: g_sec value of last expansion
                cost_t g_tail_sec = current_label_g_sec + edge_data.cost_2;
                if (g_tail_sec >= g_min_sec[tail]){continue;}

                // Check against pruning rule2: secondary global upper bound
                cost_t f_tail_sec = g_tail_sec + h_sec[tail];
                if (f_tail_sec > secondary_ub){continue;}

                // Check against pruning rule3: primary global upper bound
                cost_t f_tail_pri = current_label_g_pri + edge_data.cost_1 + h_pri[tail];
                if (h_pri[tail] > primary_ub || f_tail_pri > primary_ub){continue;}

                // the extened path seems to be non-dominated, so generte a label with path info and add it to the queue
                // Using (f1,f2) with f2 as a tie breaker (if needed)
                LABEL *new_label = label_pool.get_label();
                
                #ifdef PATH
                *new_label = LABEL(f_tail_pri, f_tail_sec, tail, edge_data.tail_incoming, path_id); // keep backtracking information
                expanded = true; // expansion was successful, so set the flag
                #else
                *new_label = LABEL(f_tail_pri, f_tail_sec, tail);
                #endif
                
                open_.push(new_label);
            }

            #ifdef PATH
            if (expanded) // Store path if: the path flag is set and the label has been expanded
            {
                // CAUTION:: Max parent_array_id is UINT16_MAX
                Paths[current_vertex].push_back(std::make_pair(current_label->get_incoming_edge(), current_label->get_path_id()));
            }
            #endif

            label_pool.save_label(current_label);
        }
        return NULL;
    }
//////////////////////////////////////////////////////

};


#endif