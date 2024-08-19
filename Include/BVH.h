#ifndef BVH_H
#define BVH_H

#include <vector>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <memory>
#include "Vector3.h"



/**
 * @brief Axis-Aligned Bounding Box (AABB) struct.
 */
struct AABB {
    Vector3f min, max;
    AABB();
    AABB(const Vector3f& min, const Vector3f& max);

    /**
     * @brief Expands the bounding box to include the given point.
     * @param point The point to include.
     */
    void expand(const Vector3f& point);

    /**
     * @brief Checks if this AABB intersects with another AABB.
     * @param other The other AABB.
     * @return True if the boxes intersect, false otherwise.
     */
    bool intersects(const AABB& other) const;
};

/**
 * @brief A triangle in 3D space, defined by three vertices.
 */
struct Triangle {
    Vector3f v1, v2, v3;
    AABB boundingBox;

    Triangle(const Vector3f& v1, const Vector3f& v2, const Vector3f& v3);

    /**
     * @brief Computes the centroid of the triangle.
     * @return The centroid as a Vec3.
     */
    Vector3f getCentroid() const;
};

/**
 * @brief A node in the Bounding Volume Hierarchy (BVH) tree.
 */
struct BVHNode {
    AABB boundingBox;
    std::vector<Triangle> triangles;
    std::unique_ptr<BVHNode> left;
    std::unique_ptr<BVHNode> right;
};

/**
 * @brief A Bounding Volume Hierarchy (BVH) class for efficient spatial queries.
 */
class BVH {
private:
    std::unique_ptr<BVHNode> root;

    /**
     * @brief Selects the axis on which to split the triangles.
     * @param triangles The list of triangles.
     * @return The index of the axis to split on.
     */
    static int selectSplitAxis(const std::vector<Triangle>& triangles);

    /**
     * @brief Recursively builds the BVH tree.
     * @param triangles The list of triangles to build from.
     * @param depth The current depth of the tree.
     * @return A pointer to the root node of the tree.
     */
    static std::unique_ptr<BVHNode> buildRecursive(std::vector<Triangle>& triangles, int depth = 0);

    /**
     * @brief Recursively checks for intersection with the BVH.
     * @param node The current BVH node.
     * @param queryBox The query bounding box.
     * @param result The list of intersected triangles.
     */
    void intersectRecursive(const BVHNode* node, const AABB& queryBox, std::vector<Triangle>& result) const;

public:
    BVH(const std::vector<Triangle>& triangles);

    // Explicitly delete copy constructor and assignment operator
    BVH(const BVH&) = delete;
    BVH& operator=(const BVH&) = delete;

    // Move constructor and assignment operator
    BVH(BVH&&) = default;
    BVH& operator=(BVH&&) = default;

    /**
     * @brief Finds all triangles that intersect with the given bounding box.
     * @param queryBox The query bounding box.
     * @return A vector of intersected triangles.
     */
    std::vector<Triangle> intersect(const AABB& queryBox) const;
};

/**
 * @brief Checks for collisions between two sets of triangles.
 *
 * @param bvh1 The first BVH.
 * @param bvh2 The second BVH.
 * @return True if there is a collision, false otherwise.
 */
bool checkCollision(const BVH& bvh1, const BVH& bvh2);

#endif // BVH_H
