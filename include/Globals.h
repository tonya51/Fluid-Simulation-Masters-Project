#ifndef GLOBALS_H
#define GLOBALS_H
#include <vector>
#include <string>

/// @file Globals.h
/// @brief this class sets global variables used from other classes
/// @author Antonia Strantzi
/// @date 22/08/2016
/// @class Globals

extern std::string fileName;
extern int updateCycles; // max update cycles for the simulation

extern float particleSize;
extern float dt; // time step
extern std::vector<float> tankDims; // bounding box/tank dimensions
extern std::vector<float> tankPos; // bounding box/tank starting drawing position
extern std::vector<float> g; // gravity
extern float mass;
extern int maxNeighbours;

extern std::string shapeType; // initial shape of the fluid volume
extern float volume;
extern std::vector<float> volumeDims; // dimensions for the initial fluid volume
extern float rad; // radius if the fluid volume is a sphere
extern std::vector<float> wind;
extern int totalParticles;

extern float kh; // smoothing kernel radius
extern float hl; // cell size for spatial hashing
extern int nh; // size of hash table
extern int p1; // spatial hashing large prime number constants
extern int p2;
extern int p3;

extern float rd; // fluid rest density
extern float kconst; // k constant for pressure
extern float mvisc; // viscosity constant
extern float sigmasurf; // surface tension constant
extern float lsurf; // surface tension threshold
extern float buoyancyConst; // buoyancy constant

extern int nbRad; // radius used in a previous version of the neighbour search
extern int updates; // simulation updates count

#endif // GLOBALS_H
