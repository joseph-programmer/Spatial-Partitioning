#include "SparseVoxelOctree.h"

/**
 * @brief Constructor to initialize the Sparse Voxel Octree.
 */
SparseVoxelOctree::SparseVoxelOctree(float size, Vector3f origin, int maxDepth)
    : size(size), origin(origin), maxDepth(maxDepth) {
    root = std::make_unique<OctreeNode>();
}

void SparseVoxelOctree::insert(const Vector3f& position) {
    std::lock_guard<std::mutex> lock(mtx);
    insertRecursive(root.get(), position, size, origin, 0);
}

bool SparseVoxelOctree::query(const Vector3f& position) {
    std::lock_guard<std::mutex> lock(mtx);
    return queryRecursive(root.get(), position, size, origin, 0);
}

bool SparseVoxelOctree::raycast(const Vector3f& origin, const Vector3f& direction, float maxDistance, Vector3f& hitPoint) {
    std::lock_guard<std::mutex> lock(mtx);
    return raycastRecursive(root.get(), origin, direction, maxDistance, this->origin, size, hitPoint);
}

void SparseVoxelOctree::serialize(const std::string& filename) const {
    std::ofstream file(filename, std::ios::binary);
    serializeRecursive(root.get(), file);
}

void SparseVoxelOctree::deserialize(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    root = std::make_unique<OctreeNode>();
    deserializeRecursive(root.get(), file);
}

void SparseVoxelOctree::insertRecursive(OctreeNode* node, const Vector3f& position, float nodeSize, const Vector3f& nodeOrigin, int depth) {
    if (depth == maxDepth) {
        node->isOccupied = true;
        return;
    }

    if (node->isLeaf) {
        node->isLeaf = false;
    }

    int octant = getOctant(position, nodeOrigin);
    float halfSize = nodeSize / 2;
    Vector3f newOrigin(
        nodeOrigin.x + (octant & 1 ? halfSize : 0),
        nodeOrigin.y + (octant & 2 ? halfSize : 0),
        nodeOrigin.z + (octant & 4 ? halfSize : 0)
    );

    if (!node->children[octant]) {
        node->children[octant] = std::make_unique<OctreeNode>();
    }

    insertRecursive(node->children[octant].get(), position, halfSize, newOrigin, depth + 1);
}

bool SparseVoxelOctree::queryRecursive(const OctreeNode* node, const Vector3f& position, float nodeSize, const Vector3f& nodeOrigin, int depth) const {
    if (!node) return false;
    if (node->isLeaf) return node->isOccupied;

    int octant = getOctant(position, nodeOrigin);
    float halfSize = nodeSize / 2;
    Vector3f newOrigin(
        nodeOrigin.x + (octant & 1 ? halfSize : 0),
        nodeOrigin.y + (octant & 2 ? halfSize : 0),
        nodeOrigin.z + (octant & 4 ? halfSize : 0)
    );

    return queryRecursive(node->children[octant].get(), position, halfSize, newOrigin, depth + 1);
}

bool SparseVoxelOctree::raycastRecursive(const OctreeNode* node, const Vector3f& origin, const Vector3f& direction, float maxDistance,
    const Vector3f& nodeOrigin, float nodeSize, Vector3f& hitPoint) const {
    if (!node) return false;
    if (node->isLeaf && node->isOccupied) {
        hitPoint = origin;
        return true;
    }

    float tMin, tMax;
    if (!intersectAABB(origin, direction, nodeOrigin, nodeOrigin + Vector3f(nodeSize, nodeSize, nodeSize), tMin, tMax)) {
        return false;
    }

    if (tMin > maxDistance || tMax < 0) return false;

    if (!node->isLeaf) {
        float halfSize = nodeSize / 2;
        for (int i = 0; i < 8; ++i) {
            Vector3f childOrigin(
                nodeOrigin.x + (i & 1 ? halfSize : 0),
                nodeOrigin.y + (i & 2 ? halfSize : 0),
                nodeOrigin.z + (i & 4 ? halfSize : 0)
            );
            if (raycastRecursive(node->children[i].get(), origin, direction, maxDistance, childOrigin, halfSize, hitPoint)) {
                return true;
            }
        }
    }

    return false;
}

void SparseVoxelOctree::serializeRecursive(const OctreeNode* node, std::ofstream& file) const {
    file.write(reinterpret_cast<const char*>(&node->isLeaf), sizeof(bool));
    file.write(reinterpret_cast<const char*>(&node->isOccupied), sizeof(bool));

    if (!node->isLeaf) {
        for (int i = 0; i < 8; ++i) {
            bool hasChild = static_cast<bool>(node->children[i]);
            file.write(reinterpret_cast<const char*>(&hasChild), sizeof(bool));
            if (hasChild) {
                serializeRecursive(node->children[i].get(), file);
            }
        }
    }
}

void SparseVoxelOctree::deserializeRecursive(OctreeNode* node, std::ifstream& file) {
    file.read(reinterpret_cast<char*>(&node->isLeaf), sizeof(bool));
    file.read(reinterpret_cast<char*>(&node->isOccupied), sizeof(bool));

    if (!node->isLeaf) {
        for (int i = 0; i < 8; ++i) {
            bool hasChild;
            file.read(reinterpret_cast<char*>(&hasChild), sizeof(bool));
            if (hasChild) {
                node->children[i] = std::make_unique<OctreeNode>();
                deserializeRecursive(node->children[i].get(), file);
            }
        }
    }
}

int SparseVoxelOctree::getOctant(const Vector3f& position, const Vector3f& nodeOrigin) const {
    int octant = 0;
    if (position.x >= nodeOrigin.x) octant |= 1;
    if (position.y >= nodeOrigin.y) octant |= 2;
    if (position.z >= nodeOrigin.z) octant |= 4;
    return octant;
}

bool SparseVoxelOctree::intersectAABB(const Vector3f& origin, const Vector3f& direction, const Vector3f& boxMin, const Vector3f& boxMax,
    float& tMin, float& tMax) const {
    Vector3f invDir(1.0f / direction.x, 1.0f / direction.y, 1.0f / direction.z);
    Vector3f t1 = (boxMin - origin) * invDir;
    Vector3f t2 = (boxMax - origin) * invDir;

    tMin = std::max(std::max(std::min(t1.x, t2.x), std::min(t1.y, t2.y)), std::min(t1.z, t2.z));
    tMax = std::min(std::min(std::max(t1.x, t2.x), std::max(t1.y, t2.y)), std::max(t1.z, t2.z));

    return tMax >= tMin;
}
