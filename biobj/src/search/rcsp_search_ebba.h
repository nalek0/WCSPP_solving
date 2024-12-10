#ifndef WC_EBBA_H
#define WC_EBBA_H

// Enhanced Biased Bidirectional A* Search
// for the Weight Constrined Shortest Path Problem (WCSPP)
// @author: sahmadi

template <class LABEL, class Q, class H>
class WC_EBBA
{
public:
    WC_EBBA(graph *G, graph *G_rev, experiment exp)
    {
        initialise_parameters(G->Num_vertices);

        // generate two candidate solution labels
        solution_label_f = new LABEL(COST_MAX, COST_MAX, SN_ID_MAX, EDGE_CAP_MAX, PATH_ARR_SIZE_MAX);
        solution_label_b = new LABEL(COST_MAX, COST_MAX, SN_ID_MAX, EDGE_CAP_MAX, PATH_ARR_SIZE_MAX);

        // start heusristic search time
        timer mytimer;
        mytimer.start();

        // obtain lower bounds using A*
        // execute a point-to-point forward A* from scratch on cost1
        Bounded_Astar_f1f2<H>(G, *open_1_b, h1_b, ub2_b, parent_b, exp.start, exp.goal, h1_b[exp.goal], true, NULL);
        // execute a point-to-point backward A* from scratch on cost2
        Bounded_Astar_f2f1<H>(G_rev, *open_2_f, h2_f, ub1_f, NULL, exp.goal, exp.start, h2_f[exp.start], true, NULL);
        // calculate budget and global upper bounds
        cost1_ub = ub1_f[exp.start];
        cost2_ub = std::floor(exp.constraint * (ub2_b[exp.goal] - h2_f[exp.start]) / 100) + h2_f[exp.start];
        // Obtain full heuristics if the initial shortest path is not feasible
        if (ub2_b[exp.goal] > cost2_ub)
        {
            // Switch orders based on the constraint
            if (exp.constraint > 30)
            {
                // resume the point-to-point forward A* on cost1
                Bounded_Astar_f1f2<H>(G, *open_1_b, h1_b, ub2_b, parent_b, exp.start, exp.goal, cost1_ub, false, NULL);
                // execute a backward A* from scratch on cost1 and keep track of expanded ones via expanded1
                Bounded_Astar_Informed_f1f2(G_rev, *open_1_f, h1_f, ub2_f, h1_b, ub2_b, parent_f, exp.goal, exp.start, expanded1, NULL);
                // resume the point-to-point backward A* on cost2 and prune vertices not expanded in cost1 search via expanded1
                Bounded_Astar_Prune_f2f1<H>(G_rev, *open_2_f, h2_f, ub1_f, NULL, exp.goal, exp.start, cost2_ub, false, NULL, expanded1);
                // execute a forward A* from scratch on cost2 and prune vertices not expanded in cost1 search via expanded1
                Bounded_Astar_Informed_Prune_f2f1(G, *open_2_b, h2_b, ub1_b, h2_f, ub1_f, NULL, exp.start, exp.goal, NULL, expanded1);
            }
            else
            {
                // resume the point-to-point backward A* on cost2
                Bounded_Astar_f2f1<H>(G_rev, *open_2_f, h2_f, ub1_f, NULL, exp.goal, exp.start, cost2_ub, false, NULL);
                // execute a forward A* from scratch on cost2 and keep track of expanded ones via expanded2
                Bounded_Astar_Informed_f2f1(G, *open_2_b, h2_b, ub1_b, h2_f, ub1_f, NULL, exp.start, exp.goal, expanded2, NULL);
                // resume the point-to-point forward A* on cost1 and prune vertices not expanded in cost2 search via expanded2
                Bounded_Astar_Prune_f1f2<H>(G, *open_1_b, h1_b, ub2_b, parent_b, exp.start, exp.goal, cost1_ub, false, NULL, expanded2);
                // execute a backward A* from scratch on cost1 and prune vertices not expanded in cost2 search via expanded2
                Bounded_Astar_Informed_Prune_f1f2(G_rev, *open_1_f, h1_f, ub2_f, h1_b, ub2_b, parent_f, exp.goal, exp.start, expanded2);
            }
        }

        // // obtain lower bounds using A* - APPROACH 3
        // // execute a point-to-point forward A* from scratch on cost1 and keep track of expanded vertices via expanded1
        // Bounded_Astar_f1f2<H>(G, *open_1_b, h1_b, ub2_b, parent_b, exp.start, exp.goal, h1_b[exp.goal], true, expanded1);
        // // execute a point-to-point backward A* from scratch on cost2
        // Bounded_Astar_f2f1<H>(G_rev, *open_2_f, h2_f, ub1_f, NULL, exp.goal, exp.start, h2_f[exp.start], true, NULL);
        // // calculate budget and global upper bounds
        // cost1_ub = ub1_f[exp.start];
        // cost2_ub = std::floor(exp.constraint * (ub2_b[exp.goal] - h2_f[exp.start]) / 100) + h2_f[exp.start];
        // // Obtain full heuristics if the initial shortest path is not feasible
        // if (ub2_b[exp.goal] > cost2_ub)
        // {
        //     // resume the point-to-point forward A* on cost1 and keep track of expanded vertices via expanded1
        //     Bounded_Astar_f1f2<H>(G, *open_1_b, h1_b, ub2_b, parent_b, exp.start, exp.goal, cost1_ub, false, expanded1);
        //     // resume the point-to-point backward A* on cost1 and prune vertices not expanded in cost1 search via expanded1
        //     Bounded_Astar_Prune_f2f1<H>(G_rev, *open_2_f, h2_f, ub1_f, NULL, exp.goal, exp.start, cost2_ub, false, NULL, expanded1);
        //     // execute a forward A* from scratch on cost2 and prune vertices not expanded in cost1 search via expanded1-but keep track of expanded ones via expanded2
        //     Bounded_Astar_Informed_Prune_f2f1(G, *open_2_b, h2_b, ub1_b, h2_f, ub1_f, NULL, exp.start, exp.goal, expanded2, expanded1);
        //     // execute a backward A* from scratch on cost1 and prune vertices not expanded in cost2 search via expanded2
        //     Bounded_Astar_Informed_Prune_f1f2(G_rev, *open_1_f, h1_f, ub2_f, h1_b, ub2_b, parent_f, exp.goal, exp.start, expanded2);
        // }
        ////////////////////////////////////////////////////

        // capture heusristic search time
        mytimer.stop();
        double time_init = mytimer.elapsed_time_second();
        
        // delete auxiliary arrays and structures
        delete_aux_parameters();

        // initialising search time and memory use
        double time_search = 0;
        size_t search_mem = 0;

        // Check Case 1: the first shortest path is feasible
        if (ub2_b[exp.goal] <= cost2_ub)
        {
            solution_label_b->set_id(exp.goal);
            cost1_ub = h1_b[exp.goal];
            sol_cost2 = ub2_b[exp.goal];
            
        }
        // Check Case 2: the second shortest path (after graph reduction) is feasible
        else if (ub2_f[exp.start] <= cost2_ub)
        {
            solution_label_f->set_id(exp.start);
            cost1_ub = h1_f[exp.start];
            sol_cost2 = ub2_f[exp.start];
        }
        // Case 3: Searching via Constrained A*
        else
        {
            // initialising the label pool (one pool is enough for sequential search)
            size_t label_pool_sz = 1024;
            label_pool<LABEL> Pool_f(label_pool_sz);

            // Initialising bucket queues based on the range of cost_1
            cost_t bucket_width = 1;
            Q open_f(bucket_width, h1_f[exp.start], cost1_ub); // forward queue
            Q open_b(bucket_width, h1_b[exp.goal], cost1_ub);  // backward queue

            // generating the first label for the start vertex
            LABEL *current_label = Pool_f.get_label();
            *current_label = LABEL(h1_f[exp.start], h2_f[exp.start], exp.start, EDGE_CAP_MAX, PATH_ARR_SIZE_MAX);

            //inserting label into the queue
            open_f.push(current_label);

            // generating the first label for the goal vertex
            current_label = Pool_f.get_label();
            *current_label = LABEL(h1_b[exp.goal], h2_b[exp.goal], exp.goal, EDGE_CAP_MAX, PATH_ARR_SIZE_MAX);

            //inserting label into the queue
            open_b.push(current_label);

            // set global upper bounds and search budget
            sol_cost2 = COST_MAX;
            cost_t budget_f = alpha_f * cost2_ub;
            cost_t budget_b = cost2_ub - budget_f;

            // Load Graph info
            Out_deg_f = G->Out_deg;
            Out_deg_b = G_rev->Out_deg;
            Edge_data_f = G->Edge_data;
            Edge_data_b = G_rev->Edge_data;

            //reset timer for the main search
            mytimer.reset();
            mytimer.start();

            // Execute the constrained search - Sequential
            while (open_f.size() || open_b.size())
            {
                // determine search direction (smallest of two direction)
                cost_t f_min_f = open_f.size() ? (open_f.peek())->get_f_pri() : COST_MAX;
                cost_t f_min_b = open_b.size() ? (open_b.peek())->get_f_pri() : COST_MAX;
                bool forward = (f_min_f < f_min_b); 
                
                if (forward)
                {
                    if (f_min_f > cost1_ub)
                        break;
                    Main_search_f(Pool_f, open_f, solution_label_f, solution_label_b, budget_f);
                }
                else
                {
                    if (f_min_b > cost1_ub)
                        break;
                    Main_search_b(Pool_f, open_b, solution_label_b, solution_label_f, budget_b);
                }
            }

            // Stop timer and then read memory
            mytimer.stop();
            time_search = mytimer.elapsed_time_second();
                       
            // Calculate memory used for all labels + priority queue
            search_mem = Pool_f.mem() + open_f.mem() + open_b.mem();
        }

        // Calculate memory used for saving all paths
        size_t paths_mem = this->mem(G->Num_vertices);
        
        // Store results
        results_rcsp<LABEL> res;
        res.obj1_lb = h1_b[exp.goal]; res.obj1_ub = ub1_f[exp.start];
        res.obj2_lb = h2_f[exp.start]; res.obj2_ub = ub2_b[exp.goal];
        res.sol_cost1 = cost1_ub; res.sol_cost2 = sol_cost2;
        res.budget = cost2_ub;
        res.time_elapsed_init_sec = time_init;
        res.time_elapsed_search_sec = time_search;
        res.memory_KB = (search_mem + paths_mem)/1024; // Memory of labels + Memory of paths stored
        
        // Print results
        res.print_stats(exp);
        
        #ifdef PATH 
        if(exp.path) // Capture and then print path details if requested
        {
            // find the intermediate vertex
            sn_id_t link_vertex = solution_label_f->get_id() != SN_ID_MAX ? solution_label_f->get_id() : solution_label_b->get_id();
            // recover paths
            res.store_paths(G_rev, solution_label_f, link_vertex, Paths_f, parent_b, "f1f2", "forward");
            res.store_paths(G,     solution_label_b, link_vertex, Paths_b, parent_f, "f1f2", "backward");
            res.print_paths();
        }
        #else
        if(exp.path)
        {std::cerr<<"Please compile with 'make path' for path details.\n";}
        #endif

    }
    
    ~WC_EBBA()
    {
        delete [] h1_f; delete [] ub1_f;
        delete [] h2_f; delete [] ub2_f;
        delete [] h1_b; delete [] ub1_b;
        delete [] h2_b; delete [] ub2_b;
        delete [] g_min_f; delete [] g_min_b;
        delete solution_label_f; delete solution_label_b;
        delete [] Expanded_labels_f; delete [] Expanded_labels_b;

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
    Parent_list *Paths_f, *Paths_b;
    double_t alpha_f, alpha_b;
    cost_t cost1_ub, cost2_ub, sol_cost2;
    linkedlist<LABEL*> *Expanded_labels_f;
    linkedlist<LABEL*> *Expanded_labels_b;
    edge_cap_t *Out_deg_f, *Out_deg_b;
    edge_type **Edge_data_f, **Edge_data_b;
    LABEL *solution_label_f, *solution_label_b;
    
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

        Expanded_labels_f = new linkedlist<LABEL*>[num_vertices]();
        Expanded_labels_b = new linkedlist<LABEL*>[num_vertices]();

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
        delete [] f_val_1_f; delete [] f_val_2_f;
        delete [] f_val_1_b; delete [] f_val_2_b;
        delete[] expanded1; delete[] expanded2;
    }
//////////////////////////////////////////////////////
    void *Main_search_f(
        label_pool<LABEL> &label_pool
        , Q &open
        , LABEL * solution_label
        , LABEL * solution_label_op
        , cost_t &budget
        )
    {
        // Extract (pop) the least-cost label
        LABEL *current_label = open.pop(); 
        

        // recover vertex_id from the label
        sn_id_t current_vertex = current_label->get_id();

        // extracting the g-values from the label
        cost_t current_label_g_pri = current_label->get_f_pri() - h1_f[current_vertex];
        cost_t current_label_g_sec = current_label->get_f_sec() - h2_f[current_vertex];

        if (current_label_g_sec >= g_min_f[current_vertex])
        {
            // dominated label, ignore
            label_pool.save_label(current_label);
            return NULL;
        }
        else
        {
            // undominated label, so update g2_min
            g_min_f[current_vertex] = current_label_g_sec;
        }

        // Check for a feasible solution: early solution strategy
        // Match with shortest path on cost_1
        if (current_label_g_sec + ub2_f[current_vertex] <= cost2_ub)
        {
            // respecting the lexicographical order in solutions without tie breaker
            if (current_label->get_f_pri() < cost1_ub || current_label_g_sec + ub2_f[current_vertex] < sol_cost2)
            {
                // capture new costs
                cost1_ub = current_label->get_f_pri();
                sol_cost2 = current_label_g_sec + ub2_f[current_vertex];

                // Store solution labels
                *solution_label = *current_label;
                *solution_label_op = LABEL(COST_MAX, COST_MAX, SN_ID_MAX, EDGE_CAP_MAX, PATH_ARR_SIZE_MAX);
            }

            // check if the complementary path is unique and does not offer alternative path
            if (h1_f[current_vertex] == ub1_f[current_vertex])
            {
                // recycle the label as the solution is already stored
                label_pool.save_label(current_label);
                return NULL;
            }
        }
        // Now match with shortest path on cost_2
        else if (current_label_g_pri + ub1_f[current_vertex] < cost1_ub)
        {
            // Update upper bound and reset the secondary cost of the best solution or sol_cost2
            cost1_ub = current_label_g_pri + ub1_f[current_vertex];
            sol_cost2 = COST_MAX;
        }

        // retrieve path id if path is needed (if path flag is set by user)
        #ifdef PATH
        bool expanded = false; // keep a flag so as to know whether the label has been expanded
        path_arr_size path_id = Paths_f[current_vertex].size(); // retrieve path id if needed (flag is set)
        #endif

        if (current_label_g_sec <= budget)
        {
            // Expand successors
            for (edge_cap_t edge_id = 0; edge_id < Out_deg_f[current_vertex]; edge_id++)
            {
                // recover successor edge and its data
                edge_type edge_data = Edge_data_f[current_vertex][edge_id];
                sn_id_t tail = edge_data.tail;
                
                // Check against pruning rule1: g_sec value of last expansion
                cost_t g_tail_sec = current_label_g_sec + edge_data.cost_2;
                if (g_tail_sec >= g_min_f[tail]){continue;}
                
                // Check against pruning rule2: Upper bound on cost2 of tail
                if (g_tail_sec > ub2_b[tail]){continue;}

                // Check against pruning rule3: Upper bound on cost1 of tail
                cost_t g_tail_pri = current_label_g_pri + edge_data.cost_1;
                if (g_tail_pri > ub1_b[tail]){continue;}

                // Check against pruning rule4: primary global upper bound
                cost_t f_tail_pri = g_tail_pri + h1_f[tail];
                if (h1_f[tail] > cost1_ub || f_tail_pri > cost1_ub){continue;}

                // Check against pruning rule5: secondary global upper bound
                cost_t f_tail_sec = g_tail_sec + h2_f[tail];
                if (f_tail_sec > cost2_ub){continue;}

                // the extened path seems to be non-dominated, so generte a label with path info and add it to the queue
                // Using (f1,f2) with f2 as a tie breaker (if needed)
                LABEL *new_label = label_pool.get_label();
                
                #ifdef PATH
                *new_label = LABEL(f_tail_pri, f_tail_sec, tail, edge_data.tail_incoming, path_id);
                expanded = true; // expansion was successful, so set the flag
                #else
                *new_label = LABEL(f_tail_pri, f_tail_sec, tail);
                #endif
                
                open.push(new_label);
            }

            #ifdef PATH
            if (expanded) // Store path if: the path flag is set and the label has been expanded
            {
                // CAUTION:: Max parent_array_id is UINT16_MAX
                Paths_f[current_vertex].push_back(std::make_pair(current_label->get_incoming_edge(), current_label->get_path_id()));
            }
            #endif
        }

        // obtain the top label candidate of the reverse direction
        LABEL* rev_label = Expanded_labels_b[current_vertex].front();
        
        // Ignore matching for the current label if it is not in the coupling area
        if (h2_f[current_vertex] > cost2_ub - budget && !rev_label)
        {
            // recycle the label and return it to the pool for next expansions
            label_pool.save_label(current_label);
            return NULL;
        }

        // Uncomment this to improve space usage by detecting the last dominated label
        // LABEL *tmp_label = Expanded_labels_f[current_vertex].back();
        // if (tmp_label && tmp_label->get_f_pri() == current_label->get_f_pri())
        // {
        //     Expanded_labels_f[current_vertex].pop_back();
        //     label_pool.save_label(tmp_label);
        // }
        /////////////////////
        
        // Store label for partial path matching
        Expanded_labels_f[current_vertex].push_back(current_label);

        // Path Matching with candidates of opposite direction
        while (rev_label != NULL)
        {
            cost_t f1_sum = current_label_g_pri + rev_label->get_f_pri() - h1_b[current_vertex];
            cost_t f2_sum = current_label_g_sec + rev_label->get_f_sec() - h2_b[current_vertex];
            
            // Break as there will not be any better solution path
            if (f1_sum > cost1_ub)
            {
                break;
            }
            else if (f2_sum <= cost2_ub)
            {
                // A feasible solution found
                if (f1_sum < cost1_ub || f2_sum < sol_cost2)
                {
                    // capture new costs
                    cost1_ub = f1_sum;
                    sol_cost2 = f2_sum;

                    // update the information in the solution set
                    *solution_label = *current_label;
                    *solution_label_op = *rev_label;
                }

                // Uncomment if Solution is not needed to be lexicographically smallest
                // break;
            }
            rev_label = rev_label->get_next();
        }

        return NULL;
    }

//////////////////////////////////////////////////////
void *Main_search_b(
        label_pool<LABEL> &label_pool
        , Q &open
        , LABEL * solution_label
        , LABEL * solution_label_op
        , cost_t &budget
        )
    {
        // Extract (pop) the least-cost label
        LABEL *current_label = open.pop(); 
        

        // recover vertex_id from the label
        sn_id_t current_vertex = current_label->get_id();

        // extracting the g-values from the label
        cost_t current_label_g_pri = current_label->get_f_pri() - h1_b[current_vertex];
        cost_t current_label_g_sec = current_label->get_f_sec() - h2_b[current_vertex];

        if (current_label_g_sec >= g_min_b[current_vertex])
        {
            // dominated label, ignore
            label_pool.save_label(current_label);
            return NULL;
        }
        else
        {
            // undominated label, so update g2_min
            g_min_b[current_vertex] = current_label_g_sec;
        }

        // Check for a feasible solution: early solution strategy
        // Match with shortest path on cost_1
        if (current_label_g_sec + ub2_b[current_vertex] <= cost2_ub)
        {
            // respecting the lexicographical order in solutions without tie breaker
            if (current_label->get_f_pri() < cost1_ub || current_label_g_sec + ub2_b[current_vertex] < sol_cost2)
            {
                // capture new costs
                cost1_ub = current_label->get_f_pri();
                sol_cost2 = current_label_g_sec + ub2_b[current_vertex];

                // Store solution labels
                *solution_label = *current_label;
                *solution_label_op = LABEL(COST_MAX, COST_MAX, SN_ID_MAX, EDGE_CAP_MAX, PATH_ARR_SIZE_MAX);
            }

            // check if the complementary path is unique and does not offer alternative path
            if (h1_b[current_vertex] == ub1_b[current_vertex])
            {
                // recycle the label as the solution is already stored
                label_pool.save_label(current_label);
                return NULL;
            }
        }
        // Now match with shortest path on cost_2
        else if (current_label_g_pri + ub1_b[current_vertex] < cost1_ub)
        {
            // Update upper bound and reset the secondary cost of the best solution or sol_cost2
            cost1_ub = current_label_g_pri + ub1_b[current_vertex];
            sol_cost2 = COST_MAX;
        }

        // retrieve path id if path is needed (if path flag is set by user)
        #ifdef PATH
        bool expanded = false; // keep a flag so as to know whether the label has been expanded
        path_arr_size path_id = Paths_b[current_vertex].size(); // retrieve path id if needed (flag is set)
        #endif

        if (current_label_g_sec <= budget)
        {
            // Expand successors
            for (edge_cap_t edge_id = 0; edge_id < Out_deg_b[current_vertex]; edge_id++)
            {
                // recover successor edge and its data
                edge_type edge_data = Edge_data_b[current_vertex][edge_id];
                sn_id_t tail = edge_data.tail;
                
                // Check against pruning rule1: g_sec value of last expansion
                cost_t g_tail_sec = current_label_g_sec + edge_data.cost_2;
                if (g_tail_sec >= g_min_b[tail]){continue;}
                
                // Check against pruning rule2: Upper bound on cost2 of tail
                if (g_tail_sec > ub2_f[tail]){continue;}

                // Check against pruning rule3: Upper bound on cost1 of tail
                cost_t g_tail_pri = current_label_g_pri + edge_data.cost_1;
                if (g_tail_pri > ub1_f[tail]){continue;}

                // Check against pruning rule4: primary global upper bound
                cost_t f_tail_pri = g_tail_pri + h1_b[tail];
                if (h1_b[tail] > cost1_ub || f_tail_pri > cost1_ub){continue;}

                // Check against pruning rule5: secondary global upper bound
                cost_t f_tail_sec = g_tail_sec + h2_b[tail];
                if (f_tail_sec > cost2_ub){continue;}

                // the extened path seems to be non-dominated, so generte a label with path info and add it to the queue
                // Using (f1,f2) with f2 as a tie breaker (if needed)
                LABEL *new_label = label_pool.get_label();
                
                #ifdef PATH
                *new_label = LABEL(f_tail_pri, f_tail_sec, tail, edge_data.tail_incoming, path_id);
                expanded = true; // expansion was successful, so set the flag
                #else
                *new_label = LABEL(f_tail_pri, f_tail_sec, tail);
                #endif
                
                open.push(new_label);
            }

            #ifdef PATH
            if (expanded) // Store path if: the path flag is set and the label has been expanded
            {
                // CAUTION:: Max parent_array_id is UINT16_MAX
                Paths_b[current_vertex].push_back(std::make_pair(current_label->get_incoming_edge(), current_label->get_path_id()));
            }
            #endif
        }

        // obtain the top label candidate of the reverse direction
        LABEL* rev_label = Expanded_labels_f[current_vertex].front();
        
        // Ignore matching for the current label if it is not in the coupling area
        if (h2_b[current_vertex] > cost2_ub - budget && !rev_label)
        {
            // recycle the label and return it to the pool for next expansions
            label_pool.save_label(current_label);
            return NULL;
        }

        // Uncomment this to improve space usage by detecting the last dominated label
        // LABEL *tmp_label = Expanded_labels_b[current_vertex].back();
        // if (tmp_label && tmp_label->get_f_pri() == current_label->get_f_pri())
        // {
        //     Expanded_labels_b[current_vertex].pop_back();
        //     label_pool.save_label(tmp_label);
        // }
        /////////////////////

        // Store label for partial path matching
        Expanded_labels_b[current_vertex].push_back(current_label);

        // Path Matching with candidates of opposite direction
        while (rev_label != NULL)
        {
            cost_t f1_sum = current_label_g_pri + rev_label->get_f_pri() - h1_f[current_vertex];
            cost_t f2_sum = current_label_g_sec + rev_label->get_f_sec() - h2_f[current_vertex];
            
            // Break as there will not be any better solution path
            if (f1_sum > cost1_ub)
            {
                break;
            }
            else if (f2_sum <= cost2_ub)
            {
                // A feasible solution found
                if (f1_sum < cost1_ub || f2_sum < sol_cost2)
                {
                    // capture new costs
                    cost1_ub = f1_sum;
                    sol_cost2 = f2_sum;

                    // update the information in the solution set
                    *solution_label = *current_label;
                    *solution_label_op = *rev_label;
                }

                // Uncomment if Solution is not needed to be lexicographically smallest
                // break;
            }
            rev_label = rev_label->get_next();
        }

        return NULL;
    }
//////////////////////////////////////////////////////
//// CUSTOMISED A* SEARCHES
//////////////////////////////////////////////////////   
    void 
    Bounded_Astar_Informed_f2f1(graph *G, pqueue &Queue
    , cost_t *h, cost_t *ub
    , cost_t *h_op, cost_t *ub_op
    , sn_id_t *parent , sn_id_t source, sn_id_t goal, bool *expanded, bool *is_valid)
    {
        edge_cap_t *Out_deg = G->Out_deg;
        edge_type **Edge_data = G->Edge_data;
        
        // load cost array
        cost_t *f_vals = Queue.get_cost_array();
        h[source] = 0;
        ub[source] = 0;
        f_vals[source] = h_op[source];
        Queue.push(source);
        
        while (Queue.size() > 0)
        {
            sn_id_t current_vertex = Queue.pop();

            if ( f_vals[current_vertex] > cost2_ub)
            {break;}

            //skip those not expanded before
            // if (is_valid && !is_valid[current_vertex])
            //     {continue;}

            // Early solution update- partial path matching
            if (h1_b[current_vertex] != COST_MAX && h1_b[current_vertex] + ub_op[current_vertex] < cost1_ub && 
                     ub2_b[current_vertex] + h_op[current_vertex] <= cost2_ub)
                cost1_ub = h1_b[current_vertex] + ub_op[current_vertex];
            else 
            if (ub[current_vertex] + ub_op[current_vertex] < cost1_ub)
                cost1_ub = ub[current_vertex] + ub_op[current_vertex];
            
            expanded[current_vertex] = true;

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
    Bounded_Astar_Informed_Prune_f1f2(graph *G, pqueue &Queue
    , cost_t *h, cost_t *ub
    , cost_t *h_op, cost_t *ub_op
    , sn_id_t *parent, sn_id_t source, sn_id_t goal, bool *is_valid)
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

        double_t sum_h_f = 0;
        double_t sum_h_b = 0;
        
        while (Queue.size() > 0)
        {
            sn_id_t current_vertex = Queue.pop();

            if ( f_vals[current_vertex] > cost1_ub)
            {break;}

            //skip those not expanded before
            if (!is_valid[current_vertex])
                {continue;}
            
            // Early solution update- partial path matching
            if (ub[current_vertex] + ub_op[current_vertex] <= cost2_ub)
                cost1_ub = h[current_vertex] + h_op[current_vertex];
            else
            if ( ub[current_vertex] + h2_b[current_vertex] <= cost2_ub && 
                 h[current_vertex] + ub1_b[current_vertex] < cost1_ub   )
                cost1_ub = h[current_vertex] + ub1_b[current_vertex];
            
            // track accumulative distances
            sum_h_f += h[current_vertex]; 
            sum_h_b += h_op[current_vertex];

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

        // calculate budget factors
        if (sum_h_b > sum_h_f)
        {
            alpha_f = std::min(1.0, 0.5 * sum_h_b / sum_h_f);
            alpha_b = 1 - alpha_f;
        }
        else
        {
            alpha_b = std::min(1.0, 0.5 * sum_h_f / sum_h_b);
            alpha_f = 1 - alpha_b;
        }
    }
//////////////////////////////////////////////////////   
    void 
    Bounded_Astar_Informed_Prune_f2f1(graph *G, pqueue &Queue
    , cost_t *h, cost_t *ub
    , cost_t *h_op, cost_t *ub_op
    , sn_id_t *parent , sn_id_t source, sn_id_t goal, bool *expanded, bool *is_valid)
    {
        edge_cap_t *Out_deg = G->Out_deg;
        edge_type **Edge_data = G->Edge_data;
        
        // load cost array
        cost_t *f_vals = Queue.get_cost_array();
        h[source] = 0;
        ub[source] = 0;
        f_vals[source] = h_op[source];
        Queue.push(source);

        double_t sum_h_f = 0;
        double_t sum_h_b = 0;
        
        while (Queue.size() > 0)
        {
            sn_id_t current_vertex = Queue.pop();

            if ( f_vals[current_vertex] > cost2_ub)
            {break;}

            //skip those not expanded before
            if (is_valid && !is_valid[current_vertex])
                {continue;}

            // Early solution update- partial path matching
            if ( h1_f[current_vertex] + ub[current_vertex] < cost1_ub && 
                     ub2_f[current_vertex] + h[current_vertex] <= cost2_ub)
                cost1_ub = h1_f[current_vertex] + ub[current_vertex];
            else 
            if (ub[current_vertex] + ub_op[current_vertex] < cost1_ub)
                cost1_ub = ub[current_vertex] + ub_op[current_vertex];
            
            // expanded[current_vertex] = true;

            // track accumulative distances
            sum_h_f += h1_f[current_vertex]; 
            sum_h_b += h1_b[current_vertex];

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
        // calculate budget factors
        if (sum_h_b > sum_h_f)
        {
            alpha_f = std::min(1.0, 0.5 * sum_h_b / sum_h_f);
            alpha_b = 1 - alpha_f;
        }
        else
        {
            alpha_b = std::min(1.0, 0.5 * sum_h_f / sum_h_b);
            alpha_f = 1 - alpha_b;
        }
    }
//////////////////////////////////////////////////////
    void 
    Bounded_Astar_Informed_f1f2(graph *G, pqueue &Queue
    , cost_t *h, cost_t *ub
    , cost_t *h_op, cost_t *ub_op
    , sn_id_t *parent, sn_id_t source, sn_id_t goal, bool *expanded, bool *is_valid)
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

            // Early solution update- partial path matching
            if (ub[current_vertex] + ub_op[current_vertex] <= cost2_ub)
                cost1_ub = h[current_vertex] + h_op[current_vertex];
            else
            if (  h2_f[current_vertex] != COST_MAX && ub_op[current_vertex] + h2_f[current_vertex] <= cost2_ub && 
                 h_op[current_vertex] + ub1_f[current_vertex] < cost1_ub   )
                cost1_ub = h_op[current_vertex] + ub1_f[current_vertex];
            
            expanded[current_vertex] = true;

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