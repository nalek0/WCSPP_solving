#ifndef BOBA_H
#define BOBA_H
#include <omp.h>
// Bi-objective Bi-directional A* Search
// @author: sahmadi

template <class LABEL, class Q, class H>
class BOBA
{
public:
    BOBA(graph *G, graph *G_rev, experiment exp)
    {
        initialise_parameters(G->Num_vertices);

        // set timers
        double time_start; 
        double time_end;
        
        // Initialise upper bounds
        ub1_f[exp.start] = COST_MAX;
        ub2_b[exp.goal] = COST_MAX;

        // set timer for the first round
        time_start = omp_get_wtime();
        
        #pragma omp parallel sections num_threads(2)
        {
            // obtaining backward heuristics by running forward A* on cost_1
            #pragma omp section 
            {Bounded_Astar_f1f2<H>(G,     *open_1_b, h1_b , ub2_b, parent_b, exp.start, exp.goal, ub1_f[exp.start], true, expanded1);}
            
            // obtaining forward heuristics by running backward A* on cost_2
            #pragma omp section 
            {Bounded_Astar_f2f1<H>(G_rev, *open_2_f, h2_f, ub1_f, NULL, exp.goal, exp.start, ub2_b[exp.goal], true, expanded2);}
            
        }
        // Stop timer
        time_end = omp_get_wtime(); 
        double time_init = time_end - time_start;

        // reset timer for the second round
        time_start = omp_get_wtime();

        #pragma omp parallel sections num_threads(2)
        {
            // obtaining forward heuristics by running backward informed A* on cost_1
            #pragma omp section 
            {Bounded_Astar_Informed_Prune_f1f2(G_rev, *open_1_f, h1_f, ub2_f, h1_b, parent_f, exp.goal, exp.start, ub1_f[exp.start], expanded1, expanded2);}
            
            // obtaining backward heuristics by running forward informed A* on cost_2
            #pragma omp section 
            {Bounded_Astar_Informed_Prune_f2f1(G,     *open_2_b, h2_b, ub1_b, h2_f, parent_b, exp.start, exp.goal, ub2_b[exp.goal], expanded2, expanded1);}
        }
        // Stop timer
        time_end = omp_get_wtime(); 
        time_init += time_end - time_start;

        
        // delete auxiliary arrays and structures
        delete_aux_parameters();
        
        // initialise label pool
        size_t label_pool_sz = 1024;
        label_pool<LABEL> Pool_f(label_pool_sz);
        label_pool<LABEL> Pool_b(label_pool_sz);

        // Initialising bucket queues based on the range of cost_1, cost_2
        cost_t bucket_width = 1;
        Q open_f(bucket_width, h1_f[exp.start], ub1_f[exp.start]); // forward queue
        Q open_b(bucket_width, h2_b[exp.goal], ub2_b[exp.goal]); // backward queue
        
        // Initialising solution sets
        linkedlist<LABEL *> Sol_set_f;
        linkedlist<LABEL *> Sol_set_b;
        
        // set global upper bounds
        cost_t cost1_ub = ub1_f[exp.start] + 1;
        cost_t cost2_ub = ub2_b[exp.goal] + 1;

        // set timer for the first round
        time_start = omp_get_wtime();

        #pragma omp parallel sections num_threads(2)
        {
            // Execute a forward search on (f1,f2)
            #pragma omp section 
            Main_search_f1f2(G,      Pool_f, Sol_set_f, cost1_ub, cost2_ub, open_f, h1_f, ub1_f, h2_f, ub2_f, h1_b, g_min_f, exp.start, Paths_f);
            
            // Execute a backward search on (f2,f1)
            #pragma omp section 
            Main_search_f2f1(G_rev,  Pool_b, Sol_set_b, cost2_ub, cost1_ub, open_b, h2_b, ub2_b, h1_b, ub1_b, h2_f, g_min_b, exp.goal, Paths_b);
        }
        // Stop search timer
        time_end = omp_get_wtime(); 
        double time_search = time_end - time_start;

        // Calculate memory used for all labels and paths + priority queue
        size_t search_mem = Pool_f.mem() + Pool_b.mem() + open_f.mem() + open_b.mem();
        size_t paths_mem = this->mem(G->Num_vertices);

        // Store results
        results_biobj<LABEL> res;
        res.obj1_lb = h1_f[exp.start];
        res.obj1_ub = ub1_f[exp.start];
        res.obj2_lb = h2_f[exp.start];
        res.obj2_ub = ub2_f[exp.start];
        res.time_elapsed_init_sec = time_init;
        res.time_elapsed_search_sec = time_search;
        res.memory_KB = (search_mem + paths_mem)/1024; // Memory of labels + Memory of paths stored
        res.num_sols_f = Sol_set_f.size();
        res.num_sols_b = Sol_set_b.size();

        // Print results
        res.print_stats(exp);
        #ifdef PATH
        if(exp.path)
        {
            res.store_paths(Sol_set_f, G_rev, Paths_f, parent_f, h1_f, ub2_f, "f1f2", "forward");
            res.store_paths(Sol_set_b, G, Paths_b, parent_b, h2_b, ub1_b, "f2f1", "backward");
            res.print_paths();
        }
        #else
        if(exp.path)
        {std::cerr<<"Please compile with 'make path' for paths details.\n";}
        #endif
    }
    
    ~BOBA()
    {
        delete [] h1_f; delete [] ub1_f;
        delete [] h2_f; delete [] ub2_f;
        delete [] h1_b; delete [] ub1_b;
        delete [] h2_b; delete [] ub2_b;
        delete [] g_min_f; delete [] g_min_b;
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
    void 
    *Main_search_f1f2(
        graph *G
        , label_pool<LABEL> &label_pool
        , linkedlist<LABEL *> &Solutions
        , cost_t &primary_ub, cost_t &secondary_ub
        , Q &open_
        , cost_t *h_pri, cost_t *ub_pri
        , cost_t *h_sec, cost_t *ub_sec
        , cost_t *h_pri_opp
        , cost_t *g_min_sec
        , sn_id_t source
        , Parent_list *&Paths
        )
    {
        // Load graph data
        edge_cap_t *Out_deg = G->Out_deg;
        edge_type **Edge_data = G->Edge_data;

        // generating the first label for the source node
        LABEL *current_label = label_pool.get_label();
        *current_label = LABEL(h_pri[source], 0, source, EDGE_CAP_MAX, PATH_ARR_SIZE_MAX);

        //inserting new labl into the queue
        open_.push(current_label);

        cost_t f_last_sol = COST_MAX;

        // Search while the queue is not empty
        while (open_.size())
        {
            // Extract (pop) the least-cost label
            LABEL *current_label = open_.pop(); 
            
            // recover vertex_id from the label
            sn_id_t current_vertex = current_label->get_id();

            // extracting the g-values from the label
            cost_t current_label_g_pri = current_label->get_f_pri() - h_pri[current_vertex];
            cost_t current_label_g_sec = current_label->get_f_sec();

            // break the main while loop if the label violates the primary upper bound
            if (current_label->get_f_pri() >= primary_ub)
            {break;}

            if (current_label_g_sec >= g_min_sec[current_vertex] || current_label_g_sec + h_sec[current_vertex] >= secondary_ub)
            {
                // dominated label, ignore
                label_pool.save_label(current_label);
                continue;
            }
            else
            {
                // Heuristic tuning heppens here
                if(g_min_sec[current_vertex] == COST_MAX)
                {
                    // if this is the first visit, update the primal min cost
                    // Note that this changes the f2 values if tie breaker uses f2
                    // our tie breaking in heap is done using g2 (not f2)
                    h_pri_opp[current_vertex] = current_label_g_pri;
                }

                // undominated label, so update g2_min
                g_min_sec[current_vertex] = current_label_g_sec;
            }

            // Check for feasible a solution: early solution strategy
            if (current_label_g_sec + ub_sec[current_vertex] < secondary_ub)
            {
                // First, update the global upper bound
                secondary_ub = current_label_g_sec + ub_sec[current_vertex];
                
                // respecting the lexicographical order in solutions -> in the absence of the tie breaker
                // if the primary cost is unchanged, the previous is dominated
                if (current_label->get_f_pri() == f_last_sol)
                {
                    // update the information in the solution set, remove dominated solution
                    LABEL *tmp = Solutions.front();
                    Solutions.pop_front();
                    // recycle the dominated solution
                    label_pool.save_label(tmp);
                }
                else
                {
                    // Keep track of the f_value of the last solution
                    f_last_sol = current_label->get_f_pri();
                }

                // new solution, create a new label
                // and link it to the last solution
                LABEL *new_label = label_pool.get_label();
                *new_label = *current_label;
                Solutions.push_front(new_label);

                // check if the complementary path is unique and does not offer alternative path
                if (h_pri[current_vertex] == ub_pri[current_vertex])
                {
                    // the solution is already stored, so the label can be recycled
                    label_pool.save_label(current_label);
                    continue;
                }
            }

            #ifdef PATH
            bool expanded = false; // keep a flag so as to know whether the label has been expanded
            path_arr_size path_id = Paths[current_vertex].size(); // retrieve path id if needed (flag is set)
            #endif

            // Expand successors
            for (edge_cap_t edge_id = 0; edge_id < Out_deg[current_vertex]; edge_id++)
            {
                // recover successor edge and its data
                edge_type edge_data = Edge_data[current_vertex][edge_id];
                sn_id_t tail = edge_data.tail;
                
                // Check against pruning rule1: g_sec value of last expansion
                // cost_t g_tail_sec = f1f2 ? current_label_g_sec + edge_data.cost_2 : current_label_g_sec + edge_data.cost_1;
                cost_t g_tail_sec =  current_label_g_sec + edge_data.cost_2;
                if (g_tail_sec >= g_min_sec[tail]){continue;}

                // Check against pruning rule2: secondary global upper bound
                cost_t f_tail_sec = g_tail_sec + h_sec[tail];
                if (f_tail_sec >= secondary_ub){continue;}

                // Check against pruning rule3: primary global upper bound
                // cost_t f_tail_pri = f1f2 ? current_label_g_pri + edge_data.cost_1 + h_pri[tail] : current_label_g_pri + edge_data.cost_2 + h_pri[tail];
                cost_t f_tail_pri =  current_label_g_pri + edge_data.cost_1 + h_pri[tail];
                if (h_pri[tail] > primary_ub || f_tail_pri > primary_ub){continue;}

                // the extened path seems to be non-dominated, so generte a label with path info and add it to the queue
                // Using (f_pri, g_sec) with g_sec as a tie breaker (if needed)
                LABEL *new_label = label_pool.get_label();
                
                #ifdef PATH
                *new_label = LABEL(f_tail_pri, g_tail_sec, tail, edge_data.tail_incoming, path_id); // keep backtracking information
                expanded = true; // expansion was successful, so set the flag
                #else
                *new_label = LABEL(f_tail_pri, g_tail_sec, tail);
                #endif
                
                open_.push(new_label);
            }
            
            #ifdef PATH
            // Store path if: the path flag is set and the label has been expanded
            if (expanded)
            {
                // CAUTION:: Max parent_array_id is UINT16_MAX
                Paths[current_vertex].push_back(std::make_pair(current_label->get_incoming_edge(), current_label->get_path_id()));
            }
            #endif

            label_pool.save_label(current_label);
        }
        secondary_ub = 0; // THIS WILL MAKE THE OPPOSITE SEARCH TERMINATE 
        return NULL;
    }
//////////////////////////////////////////////////////
    void 
    *Main_search_f2f1(
        graph *G
        , label_pool<LABEL> &label_pool
        , linkedlist<LABEL *> &Solutions
        , cost_t &primary_ub, cost_t &secondary_ub
        , Q &open_
        , cost_t *h_pri, cost_t *ub_pri
        , cost_t *h_sec, cost_t *ub_sec
        , cost_t *h_pri_opp
        , cost_t *g_min_sec
        , sn_id_t source
        , Parent_list *&Paths)
    {
        // Load graph data
        edge_cap_t *Out_deg = G->Out_deg;
        edge_type **Edge_data = G->Edge_data;

        // generating the first label for the source node
        LABEL *current_label = label_pool.get_label();
        *current_label = LABEL(h_pri[source], 0, source, EDGE_CAP_MAX, PATH_ARR_SIZE_MAX);

        //inserting new labl into the queue
        open_.push(current_label);

        cost_t f_last_sol = COST_MAX;

        // Search while the queue is not empty
        while (open_.size())
        {
            // Extract (pop) the least-cost label
            LABEL *current_label = open_.pop(); 
            
            // recover vertex_id from the label
            sn_id_t current_vertex = current_label->get_id();

            // extracting the g-values from the label
            cost_t current_label_g_pri = current_label->get_f_pri() - h_pri[current_vertex];
            cost_t current_label_g_sec = current_label->get_f_sec();

            // break the main while loop if the label violates the primary upper bound
            if (current_label->get_f_pri() >= primary_ub)
            {break;}

            if (current_label_g_sec >= g_min_sec[current_vertex] || current_label_g_sec + h_sec[current_vertex] >= secondary_ub)
            {
                // dominated label, ignore
                label_pool.save_label(current_label);
                continue;
            }
            else
            {
                // Heuristic tuning heppens here
                if(g_min_sec[current_vertex] == COST_MAX)
                {
                    // if this is the first visit, update the primal min cost
                    // Note that this changes the f2 values if tie breaker uses f2
                    // our tie breaking in heap is done using g2 (not f2)
                    h_pri_opp[current_vertex] = current_label_g_pri;
                }

                // undominated label, so update g2_min
                g_min_sec[current_vertex] = current_label_g_sec;
            }

            // Check for feasible a solution: early solution strategy
            if (current_label_g_sec + ub_sec[current_vertex] < secondary_ub)
            {
                // First, update the global upper bound
                secondary_ub = current_label_g_sec + ub_sec[current_vertex];
                
                // respecting the lexicographical order in solutions -> in the absence of the tie breaker
                // if the primary cost is unchanged, the previous is dominated
                if (current_label->get_f_pri() == f_last_sol)
                {
                    // update the information in the solution set, remove dominated solution
                    LABEL *tmp = Solutions.front();
                    Solutions.pop_front();
                    // recycle the dominated solution
                    label_pool.save_label(tmp);
                }
                else
                {
                    // Keep track of the f_value of the last solution
                    f_last_sol = current_label->get_f_pri();
                }

                // new solution, create a new label
                // and link it to the last solution
                LABEL *new_label = label_pool.get_label();
                *new_label = *current_label;
                Solutions.push_front(new_label);

                // check if the complementary path is unique and does not offer alternative path
                if (h_pri[current_vertex] == ub_pri[current_vertex])
                {
                    // the solution is already stored, so the label can be recycled
                    label_pool.save_label(current_label);
                    continue;
                }
            }

            #ifdef PATH
            bool expanded = false; // keep a flag so as to know whether the label has been expanded
            path_arr_size path_id = Paths[current_vertex].size(); // retrieve path id if needed (flag is set)
            #endif

            // Expand successors
            for (edge_cap_t edge_id = 0; edge_id < Out_deg[current_vertex]; edge_id++)
            {
                // recover successor edge and its data
                edge_type edge_data = Edge_data[current_vertex][edge_id];
                sn_id_t tail = edge_data.tail;
                
                // Check against pruning rule1: g_sec value of last expansion
                cost_t g_tail_sec = current_label_g_sec + edge_data.cost_1;
                if (g_tail_sec >= g_min_sec[tail]){continue;}

                // Check against pruning rule2: secondary global upper bound
                cost_t f_tail_sec = g_tail_sec + h_sec[tail];
                if (f_tail_sec >= secondary_ub){continue;}

                // Check against pruning rule3: primary global upper bound
                cost_t f_tail_pri = current_label_g_pri + edge_data.cost_2 + h_pri[tail];
                if (h_pri[tail] > primary_ub || f_tail_pri > primary_ub){continue;}

                // the extened path seems to be non-dominated, so generte a label with path info and add it to the queue
                // Using (f_pri, g_sec) with g_sec as a tie breaker (if needed)
                LABEL *new_label = label_pool.get_label();
                
                #ifdef PATH
                *new_label = LABEL(f_tail_pri, g_tail_sec, tail, edge_data.tail_incoming, path_id); // keep backtracking information
                expanded = true; // expansion was successful, so set the flag
                #else
                *new_label = LABEL(f_tail_pri, g_tail_sec, tail);
                #endif
                
                open_.push(new_label);
            }

            #ifdef PATH
            // Store path if: the path flag is set and the label has been expanded
            if (expanded)
            {
                // CAUTION:: Max parent_array_id is UINT16_MAX
                Paths[current_vertex].push_back(std::make_pair(current_label->get_incoming_edge(), current_label->get_path_id()));
            }
            #endif

            label_pool.save_label(current_label);
        }
        secondary_ub = 0; // THIS WILL MAKE THE OPPOSITE SEARCH TERMINATE 
        return NULL;
    }
//////////////////////////////////////////////////////
//// CUSTOMISED A* SEARCHES
//////////////////////////////////////////////////////   
    void 
    Bounded_Astar_Informed_Prune_f1f2(graph *G, pqueue &Queue
    , cost_t *h, cost_t *ub
    , cost_t *h_op, sn_id_t* parent
    , sn_id_t source, sn_id_t target, cost_t upper_bound, bool *expanded, bool *expanded_op)
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

            if ( f_vals[current_vertex] > upper_bound)
            {break;}

            for (edge_cap_t edge_id = 0; edge_id < Out_deg[current_vertex]; edge_id++)
            {
                edge_type edge_data = Edge_data[current_vertex][edge_id];
                sn_id_t tail = edge_data.tail;
                cost_t g1_tail = h[current_vertex] + edge_data.cost_1;
                cost_t g2_tail = ub[current_vertex] + edge_data.cost_2;

                //skip those not expanded
                if (!expanded[current_vertex] || !expanded_op[current_vertex])
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
    Bounded_Astar_Informed_Prune_f2f1(graph *G, pqueue &Queue
    , cost_t *h, cost_t *ub
    , cost_t *h_op, sn_id_t* parent
    , sn_id_t source, sn_id_t target, cost_t upper_bound, bool *expanded, bool *expanded_op)
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

            if ( f_vals[current_vertex] > upper_bound)
            {break;}

            for (edge_cap_t edge_id = 0; edge_id < Out_deg[current_vertex]; edge_id++)
            {
                edge_type edge_data = Edge_data[current_vertex][edge_id];
                sn_id_t tail = edge_data.tail;
                cost_t g1_tail = h[current_vertex] + edge_data.cost_2;
                cost_t g2_tail = ub[current_vertex] + edge_data.cost_1;

                //skip those not expanded
                if (!expanded[current_vertex] || !expanded_op[current_vertex])
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