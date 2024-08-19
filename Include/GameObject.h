#ifndef GAMEOBJECT_H
#define GAMEOBJECT_H

#include "Vector3.h"

/**
 * @brief A class representing a game object with a position and radius.
 */
class GameObject {
public:
    int id;           ///< Unique identifier for the game object
    Vector3f position; ///< Position of the game object
    float radius;     ///< Radius of the game object, assuming it's spherical

    /**
     * @brief Constructs a new GameObject.
     * @param id Unique identifier for the game object
     * @param pos Initial position of the game object
     * @param r Radius of the game object
     */
    GameObject(int id, Vector3f pos, float r);
};


#endif // GAMEOBJECT_H