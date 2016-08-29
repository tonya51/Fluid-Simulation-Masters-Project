#include "Globals.h"
#include <math.h>

std::string fileName = "particlesOut.abc";
int updateCycles = 300;

float particleSize = 0.05;
float dt = 0.02;
std::vector<float> tankDims = {3,4,3};
std::vector<float> tankPos = {-tankDims[0]/2,tankDims[1],-tankDims[2]/2};std::vector<float> g = {0, -9.8, 0}; // gravity acceleration
//float mass = (volume*rd)/totalParticles;
float mass = 0.02;
int maxNeighbours = 50;

// Uncomment to choose a different way of initializing the fluid
/*CORNER BOX*/
std::string shapeType = "boxRandom";
float volume = 1.5;
std::vector<float> volumeDims = {1.5, volume/(1.5*1.5), 1.5};
std::vector<float> wind = {0,0,0};
float rad = 1;
///*DAM BREAK*/
//std::string shapeType = "boxRandom";
//float volume = 6;
//std::vector<float> volumeDims = {1, volume/(1*tankDims[2]), tankDims[2]};
//std::vector<float> volumeDims = {};
//float volume = volumeDims[0]*volumeDims[1]*volumeDims[2];
//std::vector<float> wind = {0,0,0};
//float rad = 1;
///*FULL BOX*/
//std::string shapeType = "boxRandom";
//float volume = 6;
//std::vector<float> volumeDims = {tankDims[0], volume/(tankDims[0]*tankDims[2]), tankDims[2]};
//std::vector<float> volumeDims = {};
//float volume = volumeDims[0]*volumeDims[1]*volumeDims[2];
//std::vector<float> wind = {2,0,0};
//float rad = 1;
///*SPHERE*/
//std::string shapeType = "sphere";
//float volume = 6;
//float rad = cbrt(volume/(4*M_PI));
//float rad = ;
//float volume = 4*M_PI*rad*rad*rad;
//std::vector<float> wind = {0,0,0};

int totalParticles = volume/(4*M_PI*pow(particleSize,3));
//int totalParticles = 3000;

float kh = 0.35;
//float kh = cbrt(maxNeighbours*volume*3/(totalParticles*4*M_PI));
float hl = kh;
int nh;
int p1 = 73856093;
int p2 = 19349663;
int p3 = 83492791;

float rd = 998.2; // water rest density
float kconst = 5;
float mvisc = 3.5;
float sigmasurf = 0.0728;
float lsurf = 6;
float buoyancyConst = 0;

int nbRad = 10;
int updates = 0;
