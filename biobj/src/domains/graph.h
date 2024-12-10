#include <cassert>
#include "constants.h"
// #include <iostream>
// #include <sstream>
#include <fstream>
// #include <omp.h>
#include <stdio.h>
#include <iostream>
#include "timer.h"

class graph
{
public:
    graph():
    Num_vertices(0), Num_edges(0), reverse(0)
    {};

    ~graph()
    {
        delete [] Out_deg;
        for (size_t ver = 0; ver < Num_vertices; ver++)
        {
            delete [] Edge_data[ver];
        }
        delete [] Edge_data;
        
        if (!reverse)
        {
            delete [] xy_co;
        }
    };
    
    sn_id_t Num_vertices;
    sn_id_t Num_edges;
    bool reverse;

    coord_pair *xy_co;
    edge_cap_t *Out_deg;
    edge_type **Edge_data;
};

void load_graph(graph *G, graph *G_rev, std::string input_file_name)
{
    sn_id_t num_vertices, num_edges;
    std::ifstream ifs(input_file_name);
    // Read graph info
    while (ifs.good())
    {
        ifs >> std::ws;
        if (ifs.peek() == '#')
        {
            while (ifs.get() != '\n')
                ;
            continue;
        }

        if (ifs.peek() == 'n')
        {
            while (ifs.get() != ' ')
                ;
        } // "nodes" keyword
        ifs >> num_vertices;
        ifs >> std::ws;
        if (ifs.peek() == 'e')
        {
            while (ifs.get() != ' ')
                ;
        } // "edges" keyword
        ifs >> num_edges;
        ifs >> std::ws;
        break;
    }

    coord_pair * xy_co = new coord_pair[num_vertices];
    edge_cap_t * Out_deg = new edge_cap_t[num_vertices];
    edge_cap_t * In_deg = new edge_cap_t[num_vertices];
    
    struct edge_full
    {
        sn_id_t head;
        sn_id_t tail;
	    edge_cap_t head_outgoing;
        edge_cap_t tail_incoming;
        edge_cost_t cost_1;
	    edge_cost_t cost_2;

    };
    edge_full * all_edge_data = new edge_full[num_edges];
    
    

    uint32_t v_added = 0, e_added = 0;
    while (ifs.good())
    {
        // Read vertices
        ifs >> std::ws;
        while (ifs.peek() == 'v')
        {
            sn_id_t id;
            int32_t x, y;
            ifs.get(); // eat the 'v' char
            ifs >> id >> x >> y;
            xy_co[id] = std::make_pair(x,y);
            // y_co[id] = y;
            ifs >> std::ws; // trailing whitespace
            v_added++;
        }

        // Read edges
        while (ifs.peek() == 'e')
        {
            assert(e_added < num_edges);

            sn_id_t from_id, to_id;
            edge_cost_t cost_1, cost_2;

            ifs.get(); // eat the 'e' char
            ifs >> from_id >> to_id >> cost_1 >> cost_2;
            {
                all_edge_data[e_added]={from_id , to_id, Out_deg[from_id], In_deg[to_id], cost_1, cost_2};
                
                Out_deg[from_id]++;
                In_deg[to_id]++;
            
            }
            e_added++;
            ifs >> std::ws;
        }
        
        edge_type ** Succ_edge_data = new edge_type*[num_vertices];
        edge_type ** Pred_edge_data = new edge_type*[num_vertices];
        for (size_t ver = 0; ver < num_vertices; ver++)
        {
            Succ_edge_data[ver] = new edge_type[Out_deg[ver]];
            Pred_edge_data[ver] = new edge_type[In_deg[ver]];
        }

        for (size_t ed = 0; ed < num_edges; ed++)
        {
            edge_full edge_ = all_edge_data[ed];
            Succ_edge_data[edge_.head][edge_.head_outgoing] = {edge_.tail, edge_.tail_incoming, edge_.cost_1, edge_.cost_2};
            Pred_edge_data[edge_.tail][edge_.tail_incoming] = {edge_.head, edge_.head_outgoing, edge_.cost_1, edge_.cost_2};
        }

        delete [] all_edge_data;
        
        // Now build graph G
        G->Num_vertices = v_added;
        G->Num_edges = e_added;
        G->reverse = false;
        G->xy_co = xy_co;
        G->Out_deg = Out_deg;
        G->Edge_data = Succ_edge_data;
        
        // Then build the reversed graph G_rev
        G_rev->Num_vertices = v_added;
        G_rev->Num_edges = e_added;
        G_rev->reverse = true;
        G_rev->xy_co = xy_co;
        G_rev->Out_deg = In_deg;
        G_rev->Edge_data = Pred_edge_data;
        
        // std::cerr << "Graph loaded with " << v_added << " vertices and " << e_added << " edges.\n";

        break;
    }
}

void write_graph(std::ostream& out, graph *G)
{  
    timer mytimer; 
    mytimer.start();
    // comments
    out << "# warthog xy graph\n"
        << "# this file is formatted as follows: [header data] [node data] [edge data]\n"
        << "# header format: nodes [number of nodes] edges [number of edges] \n"
        << "# node data format: v [id] [x] [y]\n"
        << "# edge data format: e [from_node_id] [to_node_id] [distance] [time]\n"
        << "# [distance] may be updated by eudlidean calsulations \n"
        << "#\n" 
        << "# 32bit integer values are used throughout.\n"
        << "# Identifiers are all zero indexed.\n"
        << std::endl;

    out<<std::fixed;
    out.precision(0);
    out 
        << "nodes " << G->Num_vertices << " edges " << G->Num_edges << std::endl;

    // node data
    for(uint32_t i = 0; i < G->Num_vertices; i++)
    {
        int32_t x = G->xy_co[i].first;
        int32_t y = G->xy_co[i].second;
        
        out 
            << "v " << i << " " 
            << x << " " 
            << y << " "
            << std::endl;
    }
    
    out.precision(3);
    for(size_t i = 0; i < G->Num_vertices; i++)
    {
        for(size_t edge_idx = 0; edge_idx < G->Out_deg[i]; edge_idx++)
        {
            edge_type edge_data = G->Edge_data[i][edge_idx];
            out << "e " << i << " " << edge_data.tail << " " << edge_data.cost_1 << " " << edge_data.cost_2 << std::endl;
            // out << "e " << i << " " << edge_data.tail << " " << edge_data.cost_1 << " " << (rand() % 10000) + 1 << std::endl;
            // out << "e " << i << " " << edge_data.tail << " " << (rand() % 10000) << " " << (rand() % 10000) + 1 << std::endl;
            // out << "e " << i << " " << edge_data.tail << " " << (rand() % 10000000)/(float)1000 << " " << (rand() % 10000000)/(float)1000 + 1 << std::endl;
        }
    }
    
    mytimer.stop();
    std::cerr 
        << "wrote xy_graph; time " 
        << ((double)mytimer.elapsed_time_nano() / 1e9) 
        << " s" << std::endl;
}
