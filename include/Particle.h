#ifndef PARTICLE_H
#define PARTICLE_H
#include "Globals.h"
#include <vector>
#include <ngl/Vec3.h>
#include <ngl/Colour.h>
#include <Tank.h>

/// @file Particle.h
/// @brief this class creates the particles and controls their properties
/// @author Antonia Strantzi
/// @date 22/08/2016
/// @class Particle
class System;

class Particle
{
public:
  Particle();
  Particle(int _i, std::vector<float> _p, System *_s);
  ~Particle();
  void draw(ngl::Mat4 _mouseTX);
  void update(int _switchQuantity);

  void setId(int _i);
  int getId();
  void setPosition(std::vector<float> _p);
  std::vector<float> getPosition();
  void setMass(float _m);
  float getMass();

  void setVelocity();
  void setVelocity(std::vector<float> _v);
  std::vector<float> getVelocity();
  void setXSPHVelocity();
  void setAcceleration();
  std::vector<float> getAcceleration();

  void setDensity();
  float getDensity();
  void setPressure();
  float getPressure();
  void setPressureForce();
  std::vector<float> getPressureForce();
  void setViscosity();
  std::vector<float> getViscosity();
  void setGravityForce();
  std::vector<float> getGravityForce();
  void setBuoyancyForce();
  std::vector<float> getBuoyancyForce();
  void setSurfaceTension();
  std::vector<float> getSurfaceTension();
  void setSumForces();
  std::vector<float> getSumForces();

  int hashFunction(std::vector<float> _x);
  void computeHashKey();
  int getHashKey();
  void setNeighboursList();
  std::vector<int> getNeighboursList();

  void checkBoundaryCollision();

  std::vector<float> m_pos;
  float m_mass, m_dens, m_pres;

private:
  int m_id;
  std::vector<float> m_origin, m_vel, m_acc;
  std::vector<float> m_fp, m_fv, m_fg, m_fb, m_fst, m_sumForces; // forces
  int m_hashKey, m_prevHashKey;
  System *m_system;
  std::vector<int> m_neighbours;

  float kernel(std::vector<float> _r, int _type);
  std::vector<float> kernelGradient(std::vector<float> _r, int _type);
  float kernelLaplacian(std::vector<float> _r, int _type);
};

#endif // PARTICLE_H
