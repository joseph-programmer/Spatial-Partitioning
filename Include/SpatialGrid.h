#ifndef SPATIALGRID_H
#define SPATIALGRID_H

#include <vector>
#include <unordered_map>
#include "Vector3.h"

class GameObject;

/**
 * @brief A class implementing a 3D spatial grid for efficient object management.
 */
class SpatialGrid {
private:
    float cellSize; ///< The size of each grid cell
    std::unordered_map<size_t, std::vector<GameObject*>> grid; ///< The grid storing objects

    /**
     * @brief Hashes the cell coordinates into a unique key.
     * @param x X coordinate of the cell
     * @param y Y coordinate of the cell
     * @param z Z coordinate of the cell
     * @return A unique hash key for the cell
     */
    size_t hashCell(int x, int y, int z) const;

    /**
     * @brief Retrieves all grid cells an object occupies.
     * @param obj The game object
     * @return A vector of hashed cell keys
     */
    std::vector<size_t> getCellsForObject(const GameObject& obj) const;

public:
    /**
     * @brief Constructs a new SpatialGrid.
     * @param cellSize Size of each grid cell
     */
    SpatialGrid(float cellSize);

    /**
     * @brief Inserts a game object into the grid.
     * @param obj Pointer to the game object
     */
    void insertObject(GameObject* obj);

    /**
     * @brief Removes a game object from the grid.
     * @param obj Pointer to the game object
     */
    void removeObject(GameObject* obj);

    /**
     * @brief Updates the position of a game object in the grid.
     * @param obj Pointer to the game object
     */
    void updateObject(GameObject* obj);

    /**
     * @brief Queries the grid for objects within a certain range.
     * @param center The center of the query range
     * @param radius The radius of the query range
     * @return A vector of pointers to game objects within the range
     */
    std::vector<GameObject*> queryRange(const Vector3f& center, float radius) const;
};

#endif // SPATIALGRID_H
