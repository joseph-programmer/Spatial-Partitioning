#include "BVH.h"


// Implementation of AABB
AABB::AABB() : min(Vector3f(INFINITY, INFINITY, INFINITY)), max(Vector3f(-INFINITY, -INFINITY, -INFINITY)) {}

AABB::AABB(const Vector3f& min, const Vector3f& max) : min(min), max(max) {}

void AABB::expand(const Vector3f& point) {
    min.x = std::min(min.x, point.x);
    min.y = std::min(min.y, point.y);
    min.z = std::min(min.z, point.z);
    max.x = std::max(max.x, point.x);
    max.y = std::max(max.y, point.y);
    max.z = std::max(max.z, point.z);
}

bool AABB::intersects(const AABB& other) const {
    return (min.x <= other.max.x && max.x >= other.min.x) &&
        (min.y <= other.max.y && max.y >= other.min.y) &&
        (min.z <= other.max.z && max.z >= other.min.z);
}

// Implementation of Triangle
Triangle::Triangle(const Vector3f& v1, const Vector3f& v2, const Vector3f& v3) : v1(v1), v2(v2), v3(v3) {
    boundingBox.expand(v1);
    boundingBox.expand(v2);
    boundingBox.expand(v3);
}

Vector3f Triangle::getCentroid() const {
    return Vector3f((v1.x + v2.x + v3.x) / 3,
        (v1.y + v2.y + v3.y) / 3,
        (v1.z + v2.z + v3.z) / 3);
}

// Implementation of BVH
BVH::BVH(const std::vector<Triangle>& triangles) {
    if (triangles.empty()) {
        throw std::invalid_argument("Cannot construct BVH with empty triangle list");
    }
    std::vector<Triangle> mutableTriangles = triangles;
    root = buildRecursive(mutableTriangles);
}

int BVH::selectSplitAxis(const std::vector<Triangle>& triangles) {
    Vector3f min(INFINITY, INFINITY, INFINITY);
    Vector3f max(-INFINITY, -INFINITY, -INFINITY);
    for (const auto& tri : triangles) {
        Vector3f centroid = tri.getCentroid();
        min.x = std::min(min.x, centroid.x);
        min.y = std::min(min.y, centroid.y);
        min.z = std::min(min.z, centroid.z);
        max.x = std::max(max.x, centroid.x);
        max.y = std::max(max.y, centroid.y);
        max.z = std::max(max.z, centroid.z);
    }
    Vector3f diff = Vector3f(max.x - min.x, max.y - min.y, max.z - min.z);
    if (diff.x > diff.y && diff.x > diff.z) return 0;
    if (diff.y > diff.z) return 1;
    return 2;
}

std::unique_ptr<BVHNode> BVH::buildRecursive(std::vector<Triangle>& triangles, int depth) {
    auto node = std::make_unique<BVHNode>();

    for (const auto& triangle : triangles) {
        node->boundingBox.expand(triangle.v1);
        node->boundingBox.expand(triangle.v2);
        node->boundingBox.expand(triangle.v3);
    }

    if (triangles.size() <= 4 || depth > 20) {
        node->triangles = std::move(triangles);
        return node;
    }

    int axis = selectSplitAxis(triangles);
    auto compareFunc = [axis](const Triangle& a, const Triangle& b) {
        Vector3f centroidA = a.getCentroid();
        Vector3f centroidB = b.getCentroid();
        return axis == 0 ? centroidA.x < centroidB.x :
            axis == 1 ? centroidA.y < centroidB.y :
            centroidA.z < centroidB.z;
        };

    std::sort(triangles.begin(), triangles.end(), compareFunc);

    auto mid = triangles.begin() + triangles.size() / 2;
    std::vector<Triangle> leftTriangles(triangles.begin(), mid);
    std::vector<Triangle> rightTriangles(mid, triangles.end());

    node->left = buildRecursive(leftTriangles, depth + 1);
    node->right = buildRecursive(rightTriangles, depth + 1);

    return node;
}

void BVH::intersectRecursive(const BVHNode* node, const AABB& queryBox, std::vector<Triangle>& result) const {
    if (!node->boundingBox.intersects(queryBox)) {
        return;
    }

    if (!node->triangles.empty()) {
        for (const auto& triangle : node->triangles) {
            if (triangle.boundingBox.intersects(queryBox)) {
                result.push_back(triangle);
            }
        }
    }
    else {
        intersectRecursive(node->left.get(), queryBox, result);
        intersectRecursive(node->right.get(), queryBox, result);
    }
}

std::vector<Triangle> BVH::intersect(const AABB& queryBox) const {
    std::vector<Triangle> result;
    intersectRecursive(root.get(), queryBox, result);
    return result;
}

bool checkCollision(const BVH& bvh1, const BVH& bvh2) {
    // Get all triangles from the first BVH
    std::vector<Triangle> bvh1Triangles = bvh1.intersect(AABB(Vector3f(-INFINITY, -INFINITY, -INFINITY), Vector3f(INFINITY, INFINITY, INFINITY)));

    for (const auto& triangle : bvh1Triangles) {
        // Check if any triangles from the second BVH intersect with the current triangle's bounding box
        std::vector<Triangle> intersectingTriangles = bvh2.intersect(triangle.boundingBox);
        if (!intersectingTriangles.empty()) {
            return true;  // Collision detected
        }
    }

    return false;  // No collision detected
}
