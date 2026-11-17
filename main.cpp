#include <iostream>
#include <vector>
#include <memory>
#include <chrono>
#include <random>
#include <cmath>
#include <limits>

// 1. Point Struct: Represents geographic coordinates (X, Y)
struct Point {
    int id;
    double x, y;
    Point(int id_val = 0, double x_val = 0.0, double y_val = 0.0) 
        : id(id_val), x(x_val), y(y_val) {}
};

// 2. BoundingBox Class: Handles spatial boundaries and geometric intersection tests
class BoundingBox {
public:
    double cx, cy; // Center coordinates
    double hw, hh; // Half-Width and Half-Height

    BoundingBox(double centerX, double centerY, double halfWidth, double halfHeight)
        : cx(centerX), cy(centerY), hw(halfWidth), hh(halfHeight) {}

    // Check if a point falls strictly inside this box
    bool contains(const Point& p) const {
        return (p.x >= cx - hw && p.x <= cx + hw &&
                p.y >= cy - hh && p.y <= cy + hh);
    }

    // Check if this box overlaps with another search box (The Pruning Gatekeeper)
    bool intersects(const BoundingBox& other) const {
        double this_min_x = cx - hw;
        double this_max_x = cx + hw;
        double this_min_y = cy - hh;
        double this_max_y = cy + hh;

        double other_min_x = other.cx - other.hw;
        double other_max_x = other.cx + other.hw;
        double other_min_y = other.cy - other.hh;
        double other_max_y = other.cy + other.hh;

        if (this_max_x < other_min_x || this_min_x > other_max_x) return false;
        if (this_max_y < other_min_y || this_min_y > other_max_y) return false;

        return true; // Overlap detected
    }
};

// 3. QuadtreeNode Class: Handles storage, subdivision, and pruned range queries
class QuadtreeNode {
private:
    BoundingBox boundary;
    size_t capacity;
    std::vector<Point> points;
    bool divided;

public:
    std::unique_ptr<QuadtreeNode> nw;
    std::unique_ptr<QuadtreeNode> ne;
    std::unique_ptr<QuadtreeNode> sw;
    std::unique_ptr<QuadtreeNode> se;

    QuadtreeNode(BoundingBox box, size_t cap)
        : boundary(box), capacity(cap), divided(false) {}

    void subdivide() {
        double x = boundary.cx;
        double y = boundary.cy;
        double hw = boundary.hw / 2.0;
        double hh = boundary.hh / 2.0;

        nw = std::make_unique<QuadtreeNode>(BoundingBox(x - hw, y + hh, hw, hh), capacity);
        ne = std::make_unique<QuadtreeNode>(BoundingBox(x + hw, y + hh, hw, hh), capacity);
        sw = std::make_unique<QuadtreeNode>(BoundingBox(x - hw, y - hh, hw, hh), capacity);
        se = std::make_unique<QuadtreeNode>(BoundingBox(x + hw, y - hh, hw, hh), capacity);

        divided = true;
    }

    bool insert(const Point& p) {
        if (!boundary.contains(p)) return false;

        if (points.size() < capacity && !divided) {
            points.push_back(p);
            return true;
        }

        if (!divided) {
            subdivide();
        }

        if (nw->insert(p)) return true;
        if (ne->insert(p)) return true;
        if (sw->insert(p)) return true;
        if (se->insert(p)) return true;

        return false;
    }

    // Pruned Range Query: Skips entire branches if bounding boxes don't overlap
    void queryRange(const BoundingBox& range, std::vector<Point>& foundPoints) const {
        // CRITICAL PRUNING STEP: If query box doesn't overlap this node, abort instantly!
        if (!boundary.intersects(range)) {
            return;
        }

        // Scan local points
        for (const auto& p : points) {
            if (range.contains(p)) {
                foundPoints.push_back(p);
            }
        }

        // Recursively check child quadrants
        if (divided) {
            nw->queryRange(range, foundPoints);
            ne->queryRange(range, foundPoints);
            sw->queryRange(range, foundPoints);
            se->queryRange(range, foundPoints);
        }
    }
};

// 4. Brute-Force Baseline Search
std::vector<Point> bruteForceRangeQuery(const std::vector<Point>& dataset, const BoundingBox& range) {
    std::vector<Point> results;
    for (const auto& p : dataset) {
        if (range.contains(p)) {
            results.push_back(p);
        }
    }
    return results;
}

// 5. Benchmark Execution
int main() {
    const int NUM_POINTS = 50000;
    const int NUM_QUERIES = 1000;

    std::cout << "=== GeoIndex-CPP Pruned Benchmark ===\n";
    std::cout << "Generating " << NUM_POINTS << " random spatial points...\n";

    std::vector<Point> dataset;
    dataset.reserve(NUM_POINTS);
    QuadtreeNode quadtree(BoundingBox(0.0, 0.0, 500.0, 500.0), 8);

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-500.0, 500.0);

    for (int i = 0; i < NUM_POINTS; ++i) {
        Point p(i, dist(rng), dist(rng));
        dataset.push_back(p);
        quadtree.insert(p);
    }

    // Define a localized search box (covers a small fraction of the total space)
    BoundingBox queryBox(50.0, 50.0, 25.0, 25.0);

    std::cout << "Running " << NUM_QUERIES << " Range Queries on both systems...\n\n";

    // --- Benchmark Brute-Force ---
    auto startBF = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < NUM_QUERIES; ++i) {
        volatile auto res = bruteForceRangeQuery(dataset, queryBox);
    }
    auto endBF = std::chrono::high_resolution_clock::now();
    auto durationBF = std::chrono::duration_cast<std::chrono::microseconds>(endBF - startBF).count();

    // --- Benchmark Pruned Quadtree ---
    auto startQT = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < NUM_QUERIES; ++i) {
        std::vector<Point> res;
        quadtree.queryRange(queryBox, res);
    }
    auto endQT = std::chrono::high_resolution_clock::now();
    auto durationQT = std::chrono::duration_cast<std::chrono::microseconds>(endQT - startQT).count();

    // --- Results Summary ---
    std::cout << "----------------------------------------\n";
    std::cout << "Dataset Size:     " << NUM_POINTS << " points\n";
    std::cout << "Total Queries:    " << NUM_QUERIES << " searches\n";
    std::cout << "----------------------------------------\n";
    std::cout << "Brute-Force Time: " << durationBF << " microseconds (" << durationBF / 1000.0 << " ms)\n";
    std::cout << "Quadtree Time:    " << durationQT << " microseconds (" << durationQT / 1000.0 << " ms)\n";
    if (durationQT > 0) {
        std::cout << "Speedup Factor:   " << (double)durationBF / durationQT << "x faster!\n";
    }
    std::cout << "----------------------------------------\n";

    return 0;
}