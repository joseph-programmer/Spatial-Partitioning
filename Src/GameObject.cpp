#include "GameObject.h"

/**
 * @brief Constructs a new GameObject.
 * @param id Unique identifier for the game object
 * @param pos Initial position of the game object
 * @param r Radius of the game object
 */
GameObject::GameObject(int id, Vector3f pos, float r) : id(id), position(pos), radius(r) {}
