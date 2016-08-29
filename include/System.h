#ifndef SYSTEM_H
#define SYSTEM_H
#include "Globals.h"
#include <vector>
#include <array>
#include <map>
#include <ngl/Vec3.h>
#include <ngl/Camera.h>

#include "Particle.h"
#include "Tank.h"
#include <Alembic/AbcGeom/All.h>
#include <Alembic/AbcCoreOgawa/All.h>

/// @file System.h
/// @brief this class represents the particle system making up the simulation
/// @author Antonia Strantzi
/// @author Jonathan Macey
/// @date 22/08/2016
/// @class System
/// Methods toggleExport, exportFrame and part of System written by Jonathan Macey
/// All other methods written by Antonia Strantzi
class System
{
public:
  System();
  System(std::vector<float> _p);
  ~System();

  void draw(ngl::Mat4 _mouseTX);
  void update();
  void setCam(ngl::Camera *_c);
  ngl::Camera *getCam()const;

  void makeHashTable(int _x, int _p);
  void updateHashTable(int _p, int _h, int _i);
  void exportFrame();

  void toggleExport() {m_export ^= true;}
  Particle getParticle(int _i);
  std::map<int, std::vector<int>> getHashTable();

  std::map<int, std::vector<int>> m_hashMap; // the hash table
  std::vector<Particle> m_particles; // the system's particles

private:
  std::vector<float> m_pos;
  ngl::Camera *m_cam;
  bool m_export = true;

  std::vector<float> calcPosition(int _i);
  std::unique_ptr <Alembic::AbcGeom::OArchive> m_archive;
  std::unique_ptr <Alembic::AbcGeom::OPoints> m_partsOut;
  std::unique_ptr <Alembic::AbcGeom::OC4fArrayProperty> m_rgbOut;
};

#endif // SYSTEM_H
