#ifndef TANK_H
#define TANK_H
#include "Globals.h"
#include <vector>
#include <memory>
#include <ngl/VAOPrimitives.h>
#include <ngl/Camera.h>

/// @file Tank.h
/// @brief this class is used to visualize the simulation's boundary
/// @author Antonia Strantzi
/// @date 22/08/2016
/// @class Tank

class Tank
{
public:
  Tank();
  Tank(std::vector<float> _p, std::vector<float> _d);
  ~Tank();
  void draw(ngl::Camera _c, ngl::Mat4 _mouseTX);
  void buildVAO(); // build vertex array object for the bounding box/tank
private:
  ngl::VertexArrayObject *m_t;
  std::vector<float> m_pos; // initial drawing position
  std::vector<float> m_dim; // dimensions
};

#endif // TANK_H
