#ifndef BOA_ENH_H
#define BOA_ENH_H

// Enhanced (Unidirectional) Bi-objective A*
// Using Bucket-based Priority Queue
// @author: sahmadi

#include "labels.h"
#include "pqueue.h"
#include "pqueue_label_bucket.h"
#include "pqueue_label_bucket2d.h"
#include "pqueue_label_heap.h"
#include "pqueue_label_hybrid.h"
#include "label_pool.h"
#include "timer.h"
#include "memory.h"
#include "astar.h"
#include "results.h"
#include "spherical_heuristic.h"

template <class LABEL, class Q, class H>
class BOA_Enh
{
public:
    BOA_Enh(graph *G, graph *G_rev, experiment exp)
    {
        initialise_parameters(G->Num_vertices);

        // start heusristic search time
        timer mytimer;
        mytimer.start();

        // execute a point-to-point backward A* from scratch on cost1 and keep track of expanded vertices via expanded1
        Bounded_Astar_f1f2<H>(G_rev, *open_1, h1_f, ub2_f, parent_f, exp.goal, exp.start, h1_f[exp.start], true, expanded);
        // execute a point-to-point backward A* from scratch on cost2
        Bounded_Astar_f2f1<H>(G_rev, *open_2, h2_f, ub1_f, NULL, exp.goal, exp.start, h2_f[exp.start], true, NULL);
        // resume the point-to-point backward A* on cost1 and keep track of expanded vertices via expanded1
        Bounded_Astar_f1f2<H>(G_rev, *open_1, h1_f, ub2_f, parent_f, exp.goal, exp.start, ub1_f[exp.start], false, expanded);
        // resume the point-to-point backward A* on cost1 and prune vertices not expanded in cost1 search via expanded1
        Bounded_Astar_Prune_f2f1<H>(G_rev, *open_2, h2_f, ub1_f, NULL, exp.goal, exp.start, ub2_f[exp.start], false, NULL, expanded);

        // ALTERNATIVE APPROACH
        // Astar_f1f2<H>(G_rev, *open_1, h1_f ,ub2_f, parent_f, exp.goal, exp.start, expanded);
        // Astar_f2f1<H>(G_rev, *open_2, h2_f, ub1_f, NULL,     exp.goal, exp.start, NULL);
        // Bounded_Astar_f1f2<H>(G_rev, *open_1, h1_f ,ub2_f, parent_f, exp.goal, exp.start, ub1_f[exp.start], expanded);
        // Bounded_Astar_Prune_f2f1<H>(G_rev, *open_2, h2_f, ub1_f, NULL,     exp.goal, exp.start, ub2_f[exp.start], expanded);
       
        // capture heusristic search time
        mytimer.stop();
        double time_init = mytimer.elapsed_time_second();
        
        // delete auxiliary arrays and structures
        delete_aux_parameters();

        // initialising the label pool
        size_t label_pool_sz = 1024;
        label_pool<LABEL> Pool_f(label_pool_sz);

        // Initialising bucket queues based on the range of cost_1
        cost_t bucket_width = 1;
        Q open_f(bucket_width, h1_f[exp.start], ub1_f[exp.start]); //forward queue

        // Initialising solution sets
        linkedlist<LABEL *> Sol_set_f;
        
        // set global upper bounds
        cost_t cost1_ub = ub1_f[exp.start];
        cost_t cost2_ub = ub2_f[exp.start] + 1;

        //reset timer for the main search
        mytimer.reset();
        mytimer.start();

        Main_search_f1f2(G, Pool_f, Sol_set_f, cost1_ub, cost2_ub, open_f, h1_f, ub1_f, h2_f, ub2_f, g_min_f, exp.start, Paths_f);

        // Stop timer and read memory
        mytimer.stop();
        double time_search = mytimer.elapsed_time_second();

        // Calculate memory used for all labels and paths + priority queue
        size_t search_mem = Pool_f.mem() + open_f.mem();
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
        res.num_sols_b = 0;

        // Print results
        res.print_stats(exp);
        
        // Capture and then print path details if requested
        #ifdef PATH
        if(exp.path)
        {
            res.store_paths(Sol_set_f, G_rev, Paths_f, parent_f, h1_f, ub2_f, "f1f2", "forward");
            res.print_paths();
        }
        #else
        if(exp.path)
        {std::cerr<<"Please compile with 'make path' for paths details.\n";}
        #endif
    }
    
    ~BOA_Enh()
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
    bool *expanded;
    sn_id_t *parent_f;
    pqueue *open_1, *open_2;
    Parent_list *Paths_f;

//////////////////////////////////////////////////////
    void
    initialise_parameters(sn_id_t num_vertices)
    {
        h1_f = new cost_t[num_vertices](); ub1_f = new cost_t[num_vertices]();
        h2_f = new cost_t[num_vertices](); ub2_f = new cost_t[num_vertices]();
        f_val_1 = new cost_t[num_vertices](); f_val_2 = new cost_t[num_vertices]();
        g_min_f = new cost_t[num_vertices]();
        expanded = new bool[num_vertices]();
        
        for (size_t id = 0; id < num_vertices; id++)
        {
            g_min_f[id] = COST_MAX;
            h1_f[id] = COST_MAX;
            h2_f[id] = COST_MAX;
        }

        open_1 = new pqueue(1024, f_val_1, num_vertices);
        open_2 = new pqueue(1024, f_val_2, num_vertices);
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
        delete open_1;
        delete open_2;
        delete[] f_val_1;
        delete[] f_val_2;
        delete[] expanded;
    }
//////////////////////////////////////////////////////
    void *Main_search_f1f2(
        graph *G
        , label_pool<LABEL> &label_pool
        , linkedlist<LABEL *> &Solutions
        , cost_t &primary_ub, cost_t &secondary_ub
        , Q &open_
        , cost_t *h_pri, cost_t *ub_pri
        , cost_t *h_sec, cost_t *ub_sec
        , cost_t *g_min_sec
        , sn_id_t initial
        , Parent_list *&Paths)
    {
        // Load graph data
        edge_cap_t *Out_deg = G->Out_deg;
        edge_type **Edge_data = G->Edge_data;

        // generating the first label for the initial node
        LABEL *current_label = label_pool.get_label();
        *current_label = LABEL(h_pri[initial], h_sec[initial], initial, EDGE_CAP_MAX, PATH_ARR_SIZE_MAX);

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
            cost_t current_label_g_sec = current_label->get_f_sec()- h_sec[current_vertex];

            if (current_label_g_sec >= g_min_sec[current_vertex] || current_label->get_f_sec() >= secondary_ub)
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
                cost_t g_tail_sec = current_label_g_sec + edge_data.cost_2;
                if (g_tail_sec >= g_min_sec[tail]){continue;}

                // Check against pruning rule2: secondary global upper bound
                cost_t f_tail_sec = g_tail_sec + h_sec[tail];
                if (f_tail_sec >= secondary_ub){continue;}

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
///////////////////////////////////////////////////////////////

};


#endif