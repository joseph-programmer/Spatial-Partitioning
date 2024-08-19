#ifndef SPARSEVOXELOCTREE_H
#define SPARSEVOXELOCTREE_H

#include <vector>
#include <memory>
#include <cmath>
#include <fstream>
#include <mutex>
#include "Vector3.h"

/**
 * @brief A class representing a node in the octree.
 */
class OctreeNode {
public:
    std::unique_ptr<OctreeNode> children[8]; /**< Children nodes in the octree */
    bool isLeaf; /**< Indicates if the node is a leaf */
    bool isOccupied; /**< Indicates if the node is occupied */

    /**
     * @brief Constructor to initialize the node.
     */
    OctreeNode() : isLeaf(true), isOccupied(false) {}
};

/**
 * @brief A class representing a Sparse Voxel Octree.
 */
class SparseVoxelOctree {
private:
    std::unique_ptr<OctreeNode> root; /**< Root node of the octree */
    int maxDepth; /**< Maximum depth of the octree */
    float size; /**< Size of the octree */
    Vector3f origin; /**< Origin of the octree */
    std::mutex mtx; /**< Mutex for thread safety */

public:
    /**
     * @brief Constructor to initialize the Sparse Voxel Octree.
     *
     * @param size Size of the octree
     * @param origin Origin of the octree
     * @param maxDepth Maximum depth of the octree
     */
    SparseVoxelOctree(float size, Vector3f origin, int maxDepth);

    /**
     * @brief Inserts a position into the octree.
     *
     * @param position The position to insert
     */
    void insert(const Vector3f& position);

    /**
     * @brief Queries whether a position is occupied in the octree.
     *
     * @param position The position to query
     * @return True if occupied, false otherwise
     */
    bool query(const Vector3f& position);

    /**
     * @brief Performs a raycast to check for intersections with occupied voxels.
     *
     * @param origin Origin of the ray
     * @param direction Direction of the ray
     * @param maxDistance Maximum distance the ray can travel
     * @param hitPoint The point where the ray hits a voxel
     * @return True if a voxel is hit, false otherwise
     */
    bool raycast(const Vector3f& origin, const Vector3f& direction, float maxDistance, Vector3f& hitPoint);

    /**
     * @brief Serializes the octree to a file.
     *
     * @param filename The name of the file to save to
     */
    void serialize(const std::string& filename) const;

    /**
     * @brief Deserializes the octree from a file.
     *
     * @param filename The name of the file to load from
     */
    void deserialize(const std::string& filename);

private:
    void insertRecursive(OctreeNode* node, const Vector3f& position, float nodeSize, const Vector3f& nodeOrigin, int depth);
    bool queryRecursive(const OctreeNode* node, const Vector3f& position, float nodeSize, const Vector3f& nodeOrigin, int depth) const;
    bool raycastRecursive(const OctreeNode* node, const Vector3f& origin, const Vector3f& direction, float maxDistance,
        const Vector3f& nodeOrigin, float nodeSize, Vector3f& hitPoint) const;
    void serializeRecursive(const OctreeNode* node, std::ofstream& file) const;
    void deserializeRecursive(OctreeNode* node, std::ifstream& file);
    int getOctant(const Vector3f& position, const Vector3f& nodeOrigin) const;
    bool intersectAABB(const Vector3f& origin, const Vector3f& direction, const Vector3f& boxMin, const Vector3f& boxMax,
        float& tMin, float& tMax) const;
};

#endif // SPARSEVOXELOCTREE_H
