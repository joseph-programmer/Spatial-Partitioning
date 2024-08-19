#include "SpatialGrid.h"
#include <cmath>
#include <algorithm>
#include "GameObject.h"

/**
 * @brief Constructs a new SpatialGrid.
 * @param cellSize Size of each grid cell
 */
SpatialGrid::SpatialGrid(float cellSize) : cellSize(cellSize) {}

/**
 * @brief Hashes the cell coordinates into a unique key.
 * @param x X coordinate of the cell
 * @param y Y coordinate of the cell
 * @param z Z coordinate of the cell
 * @return A unique hash key for the cell
 */
size_t SpatialGrid::hashCell(int x, int y, int z) const {
    return static_cast<size_t>(x * 73856093 ^ y * 19349663 ^ z * 83492791);
}

/**
 * @brief Retrieves all grid cells an object occupies.
 * @param obj The game object
 * @return A vector of hashed cell keys
 */
std::vector<size_t> SpatialGrid::getCellsForObject(const GameObject& obj) const {
    std::vector<size_t> cells;
    int minX = static_cast<int>((obj.position.x - obj.radius) / cellSize);
    int maxX = static_cast<int>((obj.position.x + obj.radius) / cellSize);
    int minY = static_cast<int>((obj.position.y - obj.radius) / cellSize);
    int maxY = static_cast<int>((obj.position.y + obj.radius) / cellSize);
    int minZ = static_cast<int>((obj.position.z - obj.radius) / cellSize);
    int maxZ = static_cast<int>((obj.position.z + obj.radius) / cellSize);

    for (int x = minX; x <= maxX; ++x) {
        for (int y = minY; y <= maxY; ++y) {
            for (int z = minZ; z <= maxZ; ++z) {
                cells.push_back(hashCell(x, y, z));
            }
        }
    }
    return cells;
}

/**
 * @brief Inserts a game object into the grid.
 * @param obj Pointer to the game object
 */
void SpatialGrid::insertObject(GameObject* obj) {
    for (size_t cellHash : getCellsForObject(*obj)) {
        grid[cellHash].push_back(obj);
    }
}

/**
 * @brief Removes a game object from the grid.
 * @param obj Pointer to the game object
 */
void SpatialGrid::removeObject(GameObject* obj) {
    for (size_t cellHash : getCellsForObject(*obj)) {
        auto& cell = grid[cellHash];
        cell.erase(std::remove(cell.begin(), cell.end(), obj), cell.end());
    }
}

/**
 * @brief Updates the position of a game object in the grid.
 * @param obj Pointer to the game object
 */
void SpatialGrid::updateObject(GameObject* obj) {
    removeObject(obj);
    insertObject(obj);
}

/**
 * @brief Queries the grid for objects within a certain range.
 * @param center The center of the query range
 * @param radius The radius of the query range
 * @return A vector of pointers to game objects within the range
 */
std::vector<GameObject*> SpatialGrid::queryRange(const Vector3f& center, float radius) const {
    std::vector<GameObject*> result;
    std::unordered_map<int, bool> checked;

    int minX = static_cast<int>((center.x - radius) / cellSize);
    int maxX = static_cast<int>((center.x + radius) / cellSize);
    int minY = static_cast<int>((center.y - radius) / cellSize);
    int maxY = static_cast<int>((center.y + radius) / cellSize);
    int minZ = static_cast<int>((center.z - radius) / cellSize);
    int maxZ = static_cast<int>((center.z + radius) / cellSize);

    for (int x = minX; x <= maxX; ++x) {
        for (int y = minY; y <= maxY; ++y) {
            for (int z = minZ; z <= maxZ; ++z) {
                size_t cellHash = hashCell(x, y, z);
                auto it = grid.find(cellHash);
                if (it != grid.end()) {
                    for (GameObject* obj : it->second) {
                        if (!checked[obj->id]) {
                            checked[obj->id] = true;
                            float dx = center.x - obj->position.x;
                            float dy = center.y - obj->position.y;
                            float dz = center.z - obj->position.z;
                            if (dx * dx + dy * dy + dz * dz <= (radius + obj->radius) * (radius + obj->radius)) {
                                result.push_back(obj);
                            }
                        }
                    }
                }
            }
        }
    }

    return result;
}
