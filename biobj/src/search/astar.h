template <class H>
void Bounded_Astar_f1f2(graph *G, pqueue &Queue
    , cost_t *h, cost_t *ub
    , sn_id_t *parent, sn_id_t source, sn_id_t target
    , cost_t & cost_ub, bool from_scratch, bool* expanded)

{
    edge_cap_t *Out_deg = G->Out_deg;
    edge_type **Edge_data = G->Edge_data;
    coord_pair *xy_co = G->xy_co;
    int32_t target_x = xy_co[target].first;
    int32_t target_y = xy_co[target].second;
    H heuristic(target_x, target_y);
    
    // load cost array
    cost_t *f_vals = Queue.get_cost_array();
    
    if(from_scratch)
    {
        h[source] = 0;
        ub[source] = 0;
        f_vals[source] = heuristic.get(xy_co[source].first, xy_co[source].second);
        #ifdef PATH
        parent[source] = SN_ID_MAX;
        #endif
        Queue.push(source);
    }

    while (Queue.size() > 0)
    {
        sn_id_t current_vertex = Queue.pop();

        if (f_vals[current_vertex] > cost_ub)
        {
            Queue.push(current_vertex); //retrun it to queue
            break;
        }

        if(expanded)
            expanded[current_vertex] = true;

        for (edge_cap_t edge_id = 0; edge_id < Out_deg[current_vertex]; edge_id++)
        {
            edge_type edge_data = Edge_data[current_vertex][edge_id];
            sn_id_t tail = edge_data.tail;
            cost_t g1_tail = h[current_vertex] + edge_data.cost_1;
            cost_t g2_tail = ub[current_vertex] + edge_data.cost_2;

            if (g1_tail < h[tail])
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
template <class H>
void Bounded_Astar_f2f1(graph *G, pqueue &Queue
    , cost_t *h, cost_t *ub
    , sn_id_t *parent, sn_id_t source, sn_id_t target
    , cost_t & cost_ub, bool from_scratch, bool* expanded)
{
    edge_cap_t *Out_deg = G->Out_deg;
    edge_type **Edge_data = G->Edge_data;
    coord_pair *xy_co = G->xy_co;
    int32_t target_x = xy_co[target].first;
    int32_t target_y = xy_co[target].second;
    H heuristic(target_x, target_y);

    // load cost array
    cost_t *f_vals = Queue.get_cost_array();
    
    if(from_scratch)
    {
        h[source] = 0;
        ub[source] = 0;
        f_vals[source] = heuristic.get(xy_co[source].first, xy_co[source].second);
        // if(parent) parent[source] = SN_ID_MAX;
        Queue.push(source);
    }

    while (Queue.size() > 0)
    {
        sn_id_t current_vertex = Queue.pop();
        if (f_vals[current_vertex] > cost_ub)
        {
            Queue.push(current_vertex); //retrun it to queue
            break;
        }

        if(expanded)
            expanded[current_vertex] = true;

        for (edge_cap_t edge_id = 0; edge_id < Out_deg[current_vertex]; edge_id++)
        {
            edge_type edge_data = Edge_data[current_vertex][edge_id];
            sn_id_t tail = edge_data.tail;
            cost_t g1_tail = h[current_vertex] + edge_data.cost_2;
            cost_t g2_tail = ub[current_vertex] + edge_data.cost_1;

            if (g1_tail < h[tail])
            {
                if (h[tail] == COST_MAX)
                    f_vals[tail] = g1_tail + heuristic.get(xy_co[tail].first, xy_co[tail].second);
                else
                    f_vals[tail] += g1_tail - h[tail];

                h[tail] = g1_tail;
                ub[tail] = g2_tail;
                // if(parent) parent[tail] = current_vertex;

                if (Queue.contains(tail))
                    Queue.decrease_key(tail);
                else
                    Queue.push(tail);
            }
            else if (g1_tail == h[tail] && g2_tail < ub[tail])
            {
                ub[tail] = g2_tail;
                // if(parent) parent[tail] = current_vertex;

                if (!Queue.contains(tail))
                    Queue.push(tail);
            }
        }
    }
}
//////////////////////////////////////////////////////
template <class H>
void Bounded_Astar_Prune_f1f2(graph *G, pqueue &Queue
    , cost_t *h, cost_t *ub
    , sn_id_t *parent, sn_id_t source, sn_id_t target
    , cost_t & cost_ub, bool from_scratch, bool* expanded, bool *is_valid)
{
    edge_cap_t *Out_deg = G->Out_deg;
    edge_type **Edge_data = G->Edge_data;
    coord_pair *xy_co = G->xy_co;
    int32_t target_x = xy_co[target].first;
    int32_t target_y = xy_co[target].second;
    H heuristic(target_x, target_y);
    
    // load cost array
    cost_t *f_vals = Queue.get_cost_array();
    
    if(from_scratch)
    {
        h[source] = 0;
        ub[source] = 0;
        f_vals[source] = heuristic.get(xy_co[source].first, xy_co[source].second);
        #ifdef PATH
        parent[source] = SN_ID_MAX;
        #endif
        Queue.push(source);
    }

    while (Queue.size() > 0)
    {
        sn_id_t current_vertex = Queue.pop();

        if (f_vals[current_vertex] > cost_ub)
        {
            Queue.push(current_vertex); //retrun it to queue
            break;
        }

        //skip those not expanded before
        if (!is_valid[current_vertex])
            {continue;}

        if(expanded)
            expanded[current_vertex] = true;

        for (edge_cap_t edge_id = 0; edge_id < Out_deg[current_vertex]; edge_id++)
        {
            edge_type edge_data = Edge_data[current_vertex][edge_id];
            sn_id_t tail = edge_data.tail;
            cost_t g1_tail = h[current_vertex] + edge_data.cost_1;
            cost_t g2_tail = ub[current_vertex] + edge_data.cost_2;

            if (g1_tail < h[tail])
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
template <class H>
void Bounded_Astar_Prune_f2f1(graph *G, pqueue &Queue
    , cost_t *h, cost_t *ub
    , sn_id_t *parent, sn_id_t source, sn_id_t target
    , cost_t & cost_ub, bool from_scratch, bool* expanded, bool *is_valid)
{
    edge_cap_t *Out_deg = G->Out_deg;
    edge_type **Edge_data = G->Edge_data;
    coord_pair *xy_co = G->xy_co;
    int32_t target_x = xy_co[target].first;
    int32_t target_y = xy_co[target].second;
    H heuristic(target_x, target_y);

    // load cost array
    cost_t *f_vals = Queue.get_cost_array();
    
    if(from_scratch)
    {
        h[source] = 0;
        ub[source] = 0;
        f_vals[source] = heuristic.get(xy_co[source].first, xy_co[source].second);
        // if(parent) parent[source] = SN_ID_MAX;
        Queue.push(source);
    }

    while (Queue.size() > 0)
    {
        sn_id_t current_vertex = Queue.pop();
        if (f_vals[current_vertex] > cost_ub)
        {
            Queue.push(current_vertex); //retrun it to queue
            break;
        }

        //skip those not expanded before
        if (!is_valid[current_vertex])
            {continue;}

        if(expanded)
            expanded[current_vertex] = true;

        for (edge_cap_t edge_id = 0; edge_id < Out_deg[current_vertex]; edge_id++)
        {
            edge_type edge_data = Edge_data[current_vertex][edge_id];
            sn_id_t tail = edge_data.tail;
            cost_t g1_tail = h[current_vertex] + edge_data.cost_2;
            cost_t g2_tail = ub[current_vertex] + edge_data.cost_1;

            if (g1_tail < h[tail])
            {
                if (h[tail] == COST_MAX)
                    f_vals[tail] = g1_tail + heuristic.get(xy_co[tail].first, xy_co[tail].second);
                else
                    f_vals[tail] += g1_tail - h[tail];

                h[tail] = g1_tail;
                ub[tail] = g2_tail;
                // if(parent) parent[tail] = current_vertex;

                if (Queue.contains(tail))
                    Queue.decrease_key(tail);
                else
                    Queue.push(tail);
            }
            else if (g1_tail == h[tail] && g2_tail < ub[tail])
            {
                ub[tail] = g2_tail;
                // if(parent) parent[tail] = current_vertex;

                if (!Queue.contains(tail))
                    Queue.push(tail);
            }
        }
    }
}