void Dijsktra_f1f2(graph *G, pqueue &Queue, cost_t *&ub, sn_id_t* parent, sn_id_t source, sn_id_t target)
{
    edge_cap_t *Out_deg = G->Out_deg;
    edge_type **Edge_data = G->Edge_data;

    // load cost array
    cost_t *h = Queue.get_cost_array();

    h[source] = 0;
    ub[source] = 0;
    #ifdef PATH
    parent[source] = SN_ID_MAX;
    #endif

    Queue.push(source);

    while (Queue.size() > 0)
    {
        sn_id_t current_vertex = Queue.pop();

        // if (current_vertex == target)
        // { break;}

        for (edge_cap_t edge_id = 0; edge_id < Out_deg[current_vertex]; edge_id++)
        {

            edge_type edge_data = Edge_data[current_vertex][edge_id];
            sn_id_t tail = edge_data.tail;
            cost_t g1_tail = h[current_vertex] + edge_data.cost_1;
            cost_t g2_tail = ub[current_vertex] + edge_data.cost_2;

            if (g1_tail < h[tail])
            {
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

void Dijsktra_f2f1(graph *G, pqueue &Queue, cost_t *&ub, sn_id_t* parent, sn_id_t source, sn_id_t target)
{
    edge_cap_t *Out_deg = G->Out_deg;
    edge_type **Edge_data = G->Edge_data;

    // load cost array
    cost_t *h = Queue.get_cost_array();

    h[source] = 0;
    ub[source] = 0;
    
    Queue.push(source);

    while (Queue.size() > 0)
    {
        sn_id_t current_vertex = Queue.pop();

        // if (current_vertex == target)
        // { break;}

        for (edge_cap_t edge_id = 0; edge_id < Out_deg[current_vertex]; edge_id++)
        {
            edge_type edge_data = Edge_data[current_vertex][edge_id];
            sn_id_t tail = edge_data.tail;
            cost_t g1_tail = h[current_vertex] + edge_data.cost_2;
            cost_t g2_tail = ub[current_vertex] + edge_data.cost_1;

            if (g1_tail < h[tail])
            {
                h[tail] = g1_tail;
                ub[tail] = g2_tail;
                // parent[tail] = current_vertex;

                if (Queue.contains(tail))
                    Queue.decrease_key(tail);
                else
                    Queue.push(tail);
            }
            else if (g1_tail == h[tail] && g2_tail < ub[tail])
            {
                ub[tail] = g2_tail;
                // parent[tail] = current_vertex;
                
                if (!Queue.contains(tail))
                    Queue.push(tail);

            }

        }
    }
}