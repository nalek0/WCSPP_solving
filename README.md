# WCSPP_solving

## Information

### Data

Data is taken from the [9th DIMACS Implementation Challenge - Shortest Paths](http://diag.uniroma1.it/challenge9/download.shtml)

### Algorithms

Algorithms are taken from [one cool repository](https://bitbucket.org/s-ahmadi/biobj/src/master/src/search/rcsp_search_ba_hta.h)

## Repository preparaion:

### Alogrithms build:

```bash
cd biobj/
make -C ./src
```

### Prepare roadmaps

1. Download _Distance graph_, _Travel time graph_ and _Coordinates_ archives from the DIMACS competition
1. Extract them and put to the [convert/maps/gz/](./convert/maps/gz/) folder (groupped py folders with roadmap short names)
1. Run map converter:

```bash
g++ conv/conv.cpp -o map_converter
./map_converter convert/maps/gz/ NY convert/maps/output/NY.xy
./map_converter convert/maps/gz/ BAY convert/maps/output/BAY.xy
./map_converter convert/maps/gz/ COL convert/maps/output/COL.xy
./map_converter convert/maps/gz/ FLA convert/maps/output/FLA.xy
./map_converter convert/maps/gz/ NW convert/maps/output/NW.xy
./map_converter convert/maps/gz/ NE convert/maps/output/NE.xy
./map_converter convert/maps/gz/ CAL convert/maps/output/CAL.xy
./map_converter convert/maps/gz/ LKS convert/maps/output/LKS.xy
```

This will make result convertered roadmap files at the [convert/maps/output/](./convert/maps/output/) folder
