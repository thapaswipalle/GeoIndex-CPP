# GeoIndex-CPP

Zero-dependency C++17 spatial indexing engine built from scratch for low-latency range queries and k-Nearest Neighbors (k-NN) search on edge devices.

## Core Architecture
- **Quadtree Spatial Partitioning (`main.cpp`):** Recursive 2D axis-aligned bounding box (AABB) subdivision for spatial point indexing and fast rectangular range queries.
- **Branch-Pruning k-NN Search (`knn_main.cpp`):** Max-heap priority queue search with bounding-box distance pruning, reducing query latency from ~541 ms (brute-force scan) down to ~9.13 ms.

## Build & Run
```bash
g++ -std=c++17 -O2 main.cpp -o geo_engine
./geo_engine

g++ -std=c++17 -O2 knn_main.cpp -o knn_engine
./knn_engine
