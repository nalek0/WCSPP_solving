#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cassert>

int main(int argc, char * argv[]) {
    assert(argc == 4);
    
    char * folder = argv[1];
    char * roadmap = argv[2];
    char * output = argv[3];
    std::ifstream coord_file(std::string(folder) + std::string(roadmap) + "/" + std::string("USA-road-d.") + std::string(roadmap) + std::string(".co"));
    std::ifstream dist_file(std::string(folder) + std::string(roadmap) + "/" + std::string("USA-road-d.") + std::string(roadmap) + std::string(".gr"));
    std::ifstream time_file(std::string(folder) + std::string(roadmap) + "/" + std::string("USA-road-t.") + std::string(roadmap) + std::string(".gr"));
    std::ofstream output_file(output);

    output_file << R"(# warthog xy graph
# this file is formatted as follows: [header data] [node data] [edge data]
# header format: nodes [number of nodes] edges [number of edges] 
# node data format: v [id] [x] [y]
# edge data format: e [from_node_id] [to_node_id] [distance] [time]
# [distance] may be updated by eudlidean calsulations 
#
# 32bit integer values are used throughout.
# Identifiers are all zero indexed.

)";

    std::string line;
    while (std::getline(dist_file, line)) {
        if (line[0] == 'p') {
            std::stringstream line_stream(line);
            std::string p, sp, nodes, edges;
            line_stream >> p >> sp >> nodes >> edges;
            output_file << "nodes " << nodes << " edges " << edges << "\n";
            break;
        }
    }
    dist_file.seekg(0);

    while (std::getline(coord_file, line)) {
        if (line[0] == 'v') {
            std::stringstream line_stream(line);
            std::string v, ind, x, y;
            line_stream >> v >> ind >> x >> y;
            int index = stoi(ind) - 1;
            output_file << "v " << index << " " << x << " " << y << "\n";
        }
    }

    std::string line1, line2;
    while (std::getline(dist_file, line1) && std::getline(time_file, line2)) {
        if (line1[0] == 'a') {
            std::stringstream ls1(line1);
            std::stringstream ls2(line2);
            std::string a1, ind11, ind12, dist;
            std::string a2, ind21, ind22, time;
            ls1 >> a1 >> ind11 >> ind12 >> dist;
            ls2 >> a2 >> ind21 >> ind22 >> time;

            if (ind11 != ind21) {
                std::cerr << "Error: " << ind11 << " " << ind21 << std::endl;
                return 1;
            } else if (ind12 != ind22) {
                std::cerr << "Error: " << ind12 << " " << ind22 << std::endl;
                return 1;
            } else {
                long long ind1 = stoll(ind11);
                long long ind2 = stoll(ind12);
                output_file << "e " << ind1 - 1 << " " << ind2 - 1 << " " << dist << " " << time << "\n";
            }
        }
    }

    return 0;
}