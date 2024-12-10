// programs/roadhog.cpp 
//
// Pulls together A*-based algorithms for
// weight constrained shortest path problem 
// and bi-objective search
//
// @author: sahmadi
// @updated: 11/02/22
//

#include "graph.h"
#include "input_parser.h"


#include "biobj_search_boa_enh.h"
#include "biobj_search_boba.h"
#include "rcsp_search_ebba.h"
#include "rcsp_search_ebba_par.h"
#include "rcsp_search_a.h"
#include "rcsp_search_ba.h"
#include "rcsp_search_ba_htl.h"
#include "rcsp_search_ba_hta.h"

#include "spherical_heuristic.h"
#include "euclidean_heuristic.h"
#include "haversine_heuristic.h"
#include "zero_heuristic.h"

// print debugging info during search
int verbose = 0;
// display program help on startup
int print_help = 0;
// reverse nodes
int rev_coor = 0;
// show path
int path = 0;
// for tie breaking
int tie = 0;
// for zero heuristic
int zero_heur = 0;
// number of repeats
int nruns = 1;


void
help()
{
	std::cerr 
    << "==> manual <==\n"
    << "This program solves the point-to-point bi-objective/weight constrained shortest path problem (SPP) on road networks.\n"
    << "Road networks are specified as xy-graphs. \n"
    << "XY format is similar to that used at the 9th DIMACS challenge.\n"
    << "Ref: http://www.diag.uniroma1.it//challenge9/download.shtml\n"
    << "The main differences are: \n"
    << "(i) a single file format consisting both objectives (cf. gr/co files) ; \n"
    << "(ii) node/arc ids are zero indexed (cf. 1-indexed) and; \n"
    << "Objectives are: 1-distance (DIMACS -d.gr files) and 2-time(DIMACS -t.gr files) \n"
    << "You can use our DIMACS Convertor to achieve .XY format. \n"
    << "Valid parameters:\n"
    << "\t--alg [ BOBA, BOA_enh, WC_A, WC_BA, WC_EBBA, WC_EBBA_par ]\n"
    << "\t\tBOBA: Bidirectional A* for the Bi-objective SPP (parallel search scheme). \n"
    << "\t\tBOA_enh: Enhanced A* for the Bi-objective SPP. \n"
    << "\t\tWC_A: A* for the Weight Constrained SPP. \n"
    << "\t\tWC_BA: Bidiretional A* for the Weight Constrained SPP (parallel search scheme). \n"
    << "\t\tWC_BA_HTL: WC_BA with the HTL method (parallel search scheme). \n"
    << "\t\tWC_EBBA: Enhanced Biased Bidiretional A* for the Weight Constrained SPP (sequential version). \n"
    << "\t\tWC_EBBA: Enhanced Biased Bidiretional A* for the Weight Constrained SPP (parallel version)\n"
    << "\t--input [ graph_file.xy ] \n"
    << "\t--start [ start_ID ] (start vertex ID in 1-index) \n"
    << "\t--goal [ goal_ID ] (goal vertex ID in 1-index) \n"
    << "\t--constraint [1-100] (set resource constraint in % for WCSPP) \n"
    << "\t--queue [bucket, hybrid] (priority queue type for the main search, default: bucket)\n"
    << "\t\tbucket: one-level bucket list with a linked list in buckets, works with integer costs\n"
    << "\t\thybrid: bucket list(higher level) and binary heap(lower-level), works with integer/non-integer costs\n"
    << "Valid flags:\n"
    << "\t--noheader (to supress header when printing the results)\n"
    << "\t--tie (to enable tie breaking on the second objective in the hybrid queue)\n"
    << "\t--zero (to disable spherical heuristic, needed in random non-DIMACS instances)\n"
    << "\t--nruns #  (set number of repeats, default = 1)\n"
    << "\t--path  (to enable backtracking and print paths details)\n"
    << "\t--reverse  (to reverse source <-> target)\n";
}


template <class LABEL, class QUEUE, class HEURISTIC>
void
alg_manager(graph *G, graph *G_rev, experiment exp)
{
    for(uint32_t run = 0; run < exp.nruns; run++)
    {
        if(exp.alg_name == "BOA_enh")
        {BOA_Enh<LABEL, QUEUE, HEURISTIC>(G, G_rev, exp);}
        else if(exp.alg_name == "BOBA")
        {BOBA<LABEL, QUEUE, HEURISTIC>(G, G_rev, exp);}
        else if(exp.alg_name == "WC_EBBA")
        {WC_EBBA<LABEL, QUEUE, HEURISTIC>(G, G_rev, exp);}
        else if(exp.alg_name == "WC_EBBA_par")
        {WC_EBBA_Par<LABEL, QUEUE, HEURISTIC>(G, G_rev, exp);}
        else if(exp.alg_name == "WC_A")
        {WC_A<LABEL, QUEUE, HEURISTIC>(G, G_rev, exp);}
        else if(exp.alg_name == "WC_BA")
        {WC_BA<LABEL, QUEUE, HEURISTIC>(G, G_rev, exp);}
        else if(exp.alg_name == "WC_BA_HTL")
        {WC_BA_HTL<LABEL, QUEUE, HEURISTIC>(G, G_rev, exp);}
        else if(exp.alg_name == "WC_BA_HTA")
        {WC_BA_HTA<LABEL, QUEUE, HEURISTIC>(G, G_rev, exp);}
        else
        {std::cerr << "invalid search algorithm "<< exp.alg_name <<std::endl; break;}
    }
    
}

void
run_experiment(experiment& exp)
{
    if(exp.file_name == "")
    {
        std::cerr << "parameter is missing: --input [xy-graph file]\n";
        return;
    }
    
    if(exp.alg_name == "" || exp.start == SN_ID_MAX || exp.goal == SN_ID_MAX)
    {
        std::cerr << "At least one parameter is missing!\n";
        help();
        return;
    }
    
    bool is_rcsp = exp.alg_name.substr(0, exp.alg_name.find('_', 0)) == "WC" ? true : false;
    if( is_rcsp && exp.constraint == UINT32_MAX )
    {
        std::cerr << "--constraint parameter is missing!\n";
        return;
    }
    else if(!is_rcsp || exp.constraint == UINT32_MAX )
    {exp.constraint = 100;}
    
    if (exp.reverse)
    {   sn_id_t tmp = exp.start;
        exp.start = exp.goal;
        exp.goal = tmp;
    }
    
    if (!(exp.queue_type == "hybrid"))
    {exp.queue_type = "bucket";}
    else if(tie) exp.queue_type = "hybrid_tie";
    
    graph *G = new graph();
    graph *G_rev = new graph();
    load_graph(G, G_rev, exp.file_name);

    
    
    if ( exp.start <= G->Num_vertices && exp.goal <= G->Num_vertices) 
        
        if (exp.queue_type == "hybrid" )
        {
            #ifdef PATH
            
            if (tie)
                if (zero_heur)
                    alg_manager<search_label_pri, pqueue_label_hybrid_min_w_tie, zero_heuristic>
                    (G, G_rev, exp);
                else
                    alg_manager<search_label_pri, pqueue_label_hybrid_min_w_tie, spherical_heuristic>
                    (G, G_rev, exp);
            else
                if (zero_heur)
                    alg_manager<search_label_pri, pqueue_label_hybrid_min_wo_tie, zero_heuristic>
                    (G, G_rev, exp);
                else
                    alg_manager<search_label_pri, pqueue_label_hybrid_min_wo_tie, spherical_heuristic>
                    (G, G_rev, exp);
            
            #else
            
            if (tie)
                if (zero_heur)
                    alg_manager<search_label_light_pri, pqueue_label_light_hybrid_min_w_tie, zero_heuristic>
                    (G, G_rev, exp);
                else
                    alg_manager<search_label_light_pri, pqueue_label_light_hybrid_min_w_tie, spherical_heuristic>
                    (G, G_rev, exp);
            else
                if (zero_heur)
                    alg_manager<search_label_light_pri, pqueue_label_light_hybrid_min_wo_tie, zero_heuristic>
                    (G, G_rev, exp);
                else
                    alg_manager<search_label_light_pri, pqueue_label_light_hybrid_min_wo_tie, spherical_heuristic>
                    (G, G_rev, exp);

            #endif
        }
        else
        {
            #ifdef PATH
            if (zero_heur)
                alg_manager<search_label, pqueue_label_bucket_min, zero_heuristic>
                (G, G_rev, exp);
            else
                alg_manager<search_label, pqueue_label_bucket_min, spherical_heuristic>
                (G, G_rev, exp);
            #else

            if (zero_heur)
                alg_manager<search_label_light, pqueue_label_light_bucket_min, zero_heuristic>
                (G, G_rev, exp);
            else
                alg_manager<search_label_light, pqueue_label_light_bucket_min, spherical_heuristic>
                (G, G_rev, exp);
                // can replace "pqueue_label_light_bucket_min" with:
                //          pqueue_label_light_bucket2d_min     ---> enable the 2-level bucket queue implementation, good for inputs with very large costs 
                //          pqueue_label_bucket_cyclic_min      ---> enable the cyclic implementation of the single-level bucket queue; is slower but consumes less memory
            #endif
        }
    else
    {
        if(exp.start > G->Num_vertices){std::cerr<<"start_id does not exist.\n";}
        if(exp.goal  > G->Num_vertices){std::cerr<<"goal_id does not exist.\n";}
    }
    
    delete G;
    delete G_rev;    
}



int 
main(int argc, char** argv)
{
	experiment exp;

    // parse arguments
    const char* const short_args = "a:i:s:g:p:r:q:c:n:";
    const option long_args[] = 
	{
		{"alg",  required_argument, 0, 'a'},
		{"help", no_argument, &print_help, 1},
		{"noheader",  no_argument, &exp.suppress_header, 1},
		{"input",  required_argument, 0, 'i'},
        {"start",  required_argument, 0, 's'},
        {"goal",  required_argument, 0, 'g'},
        {"path",  no_argument, 0, 'p'},
        {"reverse",  no_argument, 0, 'r'},
        {"tie",  no_argument, &tie, 1},
        {"zero",  no_argument, &zero_heur, 1},
        {"queue",  required_argument, 0, 'q'},
        {"constraint",  required_argument, 0, 'c'},
        {"nruns",  required_argument, 0, 'n'},
	};
    
    parse_args(argc, argv, short_args, long_args, exp);
    
    if(argc == 1 || print_help)
    {
		help();
        exit(0);
    }
    
    run_experiment(exp);
}