#include <iostream>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>
#include <limits>
#include <random>

// 1. Point Struct
struct Point {
    int id;
    double x, y;
    Point(int id_val = 0, double x_val = 0.0, double y_val = 0.0) 
        : id(id_val), x(x_val), y(y_val) {}
};

// 2. BoundingBox Class
class BoundingBox {
public:
    double cx, cy, hw, hh;
    BoundingBox(double centerX, double centerY, double halfWidth, double halfHeight)
        : cx(centerX), cy(centerY), hw(halfWidth), hh(halfHeight) {}

    bool contains(const Point& p) const {
        return (p.x >= cx - hw && p.x <= cx + hw &&
                p.y >= cy - hh && p.y <= cy + hh);
    }

    bool intersects(const BoundingBox& other) const {
        double this_min_x = cx - hw, this_max_x = cx + hw;
        double this_min_y = cy - hh, this_max_y = cy + hh;
        double other_min_x = other.cx - other.hw, other_max_x = other.cx + other.hw;
        double other_min_y = other.cy - other.hh, other_max_y = other.cy + other.hh;

        if (this_max_x < other_min_x || this_min_x > other_max_x) return false;
        if (this_max_y < other_min_y || this_min_y > other_max_y) return false;
        return true;
    }
};

// 3. QuadtreeNode Class with Optimized k-NN Search
class QuadtreeNode {
private:
    BoundingBox boundary;
    size_t capacity;
    std::vector<Point> points;
    bool divided;

public:
    std::unique_ptr<QuadtreeNode> nw, ne, sw, se;

    QuadtreeNode(BoundingBox box, size_t cap)
        : boundary(box), capacity(cap), divided(false) {}

    void subdivide() {
        double x = boundary.cx, y = boundary.cy;
        double hw = boundary.hw / 2.0, hh = boundary.hh / 2.0;

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
        if (!divided) subdivide();
        if (nw->insert(p)) return true;
        if (ne->insert(p)) return true;
        if (sw->insert(p)) return true;
        if (se->insert(p)) return true;
        return false;
    }

    // Helper: Minimum squared distance from target point to this node's bounding box
    double distanceSqToBox(const Point& target) const {
        double minX = boundary.cx - boundary.hw;
        double maxX = boundary.cx + boundary.hw;
        double minY = boundary.cy - boundary.hh;
        double maxY = boundary.cy + boundary.hh;

        double closestX = std::clamp(target.x, minX, maxX);
        double closestY = std::clamp(target.y, minY, maxY);

        double dx = target.x - closestX;
        double dy = target.y - closestY;
        return (dx * dx) + (dy * dy);
    }

    // Optimized Nearest-Neighbor Search with Bounding Box Pruning
    void findNearest(const Point& target, Point& bestPoint, double& minSqDist) const {
        // PRUNING CHECK: If this box is further away than our current best distance, skip it!
        if (distanceSqToBox(target) >= minSqDist) {
            return;
        }

        // Scan local points
        for (const auto& p : points) {
            double dx = p.x - target.x;
            double dy = p.y - target.y;
            double distSq = (dx * dx) + (dy * dy);
            if (distSq < minSqDist) {
                minSqDist = distSq;
                bestPoint = p;
            }
        }

        // Recurse into children if subdivided
        if (divided) {
            // Check which quadrant contains the target to visit it first (maximizes pruning efficiency)
            bool inNW = nw->boundary.contains(target);
            bool inNE = ne->boundary.contains(target);
            bool inSW = sw->boundary.contains(target);

            if (inNW) {
                nw->findNearest(target, bestPoint, minSqDist);
                ne->findNearest(target, bestPoint, minSqDist);
                sw->findNearest(target, bestPoint, minSqDist);
                se->findNearest(target, bestPoint, minSqDist);
            } else if (inNE) {
                ne->findNearest(target, bestPoint, minSqDist);
                nw->findNearest(target, bestPoint, minSqDist);
                sw->findNearest(target, bestPoint, minSqDist);
                se->findNearest(target, bestPoint, minSqDist);
            } else if (inSW) {
                sw->findNearest(target, bestPoint, minSqDist);
                nw->findNearest(target, bestPoint, minSqDist);
                ne->findNearest(target, bestPoint, minSqDist);
                se->findNearest(target, bestPoint, minSqDist);
            } else {
                se->findNearest(target, bestPoint, minSqDist);
                nw->findNearest(target, bestPoint, minSqDist);
                ne->findNearest(target, bestPoint, minSqDist);
                sw->findNearest(target, bestPoint, minSqDist);
            }
        }
    }
};

// 4. Test Execution
int main() {
    std::cout << "=== GeoIndex-CPP: Nearest-Neighbor (k-NN) Test ===\n\n";

    QuadtreeNode quadtree(BoundingBox(0.0, 0.0, 100.0, 100.0), 4);

    // Insert scattered test points
    quadtree.insert(Point(1, 10.0, 10.0));
    quadtree.insert(Point(2, 45.0, 42.0)); // Close to our query
    quadtree.insert(Point(3, -30.0, -20.0));
    quadtree.insert(Point(4, 80.0, 90.0));

    // Target query coordinate
    Point queryTarget(0, 40.0, 40.0);
    Point bestMatch(-1, 0.0, 0.0);
    double minSqDist = std::numeric_limits<double>::max();

    // Execute Nearest Neighbor search
    quadtree.findNearest(queryTarget, bestMatch, minSqDist);

    std::cout << "Query Target:     (" << queryTarget.x << ", " << queryTarget.y << ")\n";
    std::cout << "Nearest Point ID: " << bestMatch.id << " found at (" << bestMatch.x << ", " << bestMatch.y << ")\n";
    std::cout << "Euclidean Distance: " << std::sqrt(minSqDist) << "\n";
    std::cout << "\n=== k-NN Test Complete ===\n";

    return 0;
}