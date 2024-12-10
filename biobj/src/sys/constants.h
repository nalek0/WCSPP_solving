#ifndef CONSTANTS_H
#define CONSTANTS_H

// constants.h
//
// @author: sahmadi
// @updated: 06/05/2021
//

#include <cfloat>
#include <cmath>
#include <climits>
#include <stdint.h>
#include <vector>
#include <string>
#include <stdint.h>
#include "arraylist.h"
#include "linkedlist.h"

typedef uint64_t sn_id_t; // address space for state identifiers
static const sn_id_t SN_ID_MAX = UINT64_MAX;

// For integer costs
typedef uint32_t cost_t;
static const cost_t COST_MAX = UINT32_MAX;
static const cost_t COST_MIN = 0;
// For floating point costs
// typedef double cost_t;
// static const cost_t COST_MAX = DBL_MAX;
// static const cost_t COST_MIN = DBL_MIN;

typedef uint8_t edge_cap_t;
const edge_cap_t DEG_MAX = 10;
const uint8_t EDGE_CAP_MAX = UINT8_MAX;

typedef cost_t edge_cost_t;
const cost_t EDGE_COST_MAX = UINT32_MAX;

typedef uint16_t path_arr_size;
static const path_arr_size PATH_ARR_SIZE_MAX = UINT16_MAX;
typedef std::vector<std::pair<edge_cap_t, path_arr_size>> Parent_list;
// typedef arraylist<std::pair<edge_cap_t, path_arr_size>> Parent_list;
typedef std::pair<int32_t, int32_t> coord_pair;

struct experiment
{
    sn_id_t start = SN_ID_MAX;
    sn_id_t goal = SN_ID_MAX;
    uint32_t constraint = UINT32_MAX;
    uint32_t path = 0;
    uint32_t reverse = 0;
    uint32_t nruns = 1;
    int suppress_header = 0;
    std::string alg_name;
    std::string file_name;
    std::string map_name;
    std::string queue_type;
};

struct edge
{
    sn_id_t tail;
    edge_cap_t tail_incoming;
    edge_cost_t cost_1;
    edge_cost_t cost_2;
};
typedef struct edge edge_type;

#endif