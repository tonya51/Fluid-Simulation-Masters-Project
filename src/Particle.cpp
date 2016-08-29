#include "Particle.h"
#include "Globals.h"
#include <ngl/Random.h>
#include <ngl/ShaderLib.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/Transformation.h>
#include <ngl/Light.h>
#include <ngl/Material.h>
#include "System.h"
#include <random>
#include <math.h>
#include <algorithm>
#include <numeric>

float vectorLength(std::vector<float> _x, bool _p)
{
  if(_p == true) // length squared
    return _x[0]*_x[0]+_x[1]*_x[1]+_x[2]*_x[2];
  else
    return sqrt(_x[0]*_x[0]+_x[1]*_x[1]+_x[2]*_x[2]);
}

// smoothing kernel function
float Particle::kernel(std::vector<float> _r, int _type)
{
  float w, a;
  a = vectorLength(_r, true);
  if(a <= (kh*kh))
  {
    if(_type == 0) // 6th polynomial - density
      w = (315/(64*M_PI*pow(kh,9)))*pow(kh*kh-a,3);
    else if(_type == 1) // spike - pressure force
    {
      a = sqrt(a);
      w = (15/(M_PI*pow(kh,6)))*pow(kh-a,3);
    }
    else if(_type == 2) // viscosity
    {
      a = sqrt(a);
      w = (15/(2*M_PI*pow(kh,3)))*(-(pow(a,3)/(2*pow(kh,3)))+(pow(a,2)/pow(kh,2))+(kh/(2*a))-1);
    }
  }
  else
    w = 0;
  return w;
}

// smoothing kernel function gradient
std::vector<float> Particle::kernelGradient(std::vector<float> _r, int _type)
{
  std::vector<float> w = {0,0,0};
  float a = vectorLength(_r, true);
  if(a <= (kh*kh))
  {
    if(_type == 0) // 6th polynomial - density
    {
      for(int i=0; i<3; i++)
      {
        w[i] = -(945/(32*M_PI*pow(kh,9)))*pow(kh*kh-a,2)*_r[i];
      }
    }
    else if(_type == 1) // spike - pressure force
    {
      a = sqrt(a);
      for(int i=0; i<3; i++)
      {
        w[i] = -((45*_r[i])/(M_PI*pow(kh,6)*a))*pow(kh-a,2);
      }
    }
    else if(_type == 2) // viscosity
    {
      a = sqrt(a);
      for(int i=0; i<3; i++)
      {
        w[i] = ((15*_r[i])/(2*M_PI*pow(kh,3)))*(-((3*a)/(2*pow(kh,3)))+(2/pow(kh,2))-(kh/(2*pow(a,3))));
      }
    }
  }
  else
    w = {0,0,0};
  return w;
}

// smoothing kernel function Laplacian
float Particle::kernelLaplacian(std::vector<float> _r, int _type)
{
  float w;
  float a = vectorLength(_r, true);
  if(_type == 0) // 6th polynomial - density
    w = -(945/(32*M_PI*pow(kh,9)))*(kh*kh-a)*((3*kh*kh)-(7*a));
  else if(_type == 1) // spike - pressure force
  {
    a = sqrt(a);
    w = -(90/(M_PI*pow(kh,6)*a))*(kh-a)*(kh-(2*a));
  }
  else if(_type == 2) // viscosity
  {
    a = sqrt(a);
    w = (45/(M_PI*pow(kh,6)))*(kh-a);
  }
  return w;
}

Particle::Particle()
{

}

Particle::Particle(int _i, std::vector<float> _p, System *_s)
{
  m_id = _i;
  m_pos = _p;
  m_origin = _p;
  m_system = _s;
  m_vel = {0,0,0};
  m_fp = {0,0,0}; // pressure force
  m_fv = {0,0,0}; // viscosity
  m_fg = {0,0,0}; // gravity force
  m_fb = {0,0,0}; // buoyancy
  m_fst = {0,0,0}; // surface tension
  m_sumForces = {0,0,0};
  m_acc = {0,0,0};
}

Particle::~Particle()
{
  //delete(this);
}

void Particle::update(int _switchQuantity)
{
  if(_switchQuantity == 0) // update position and neighbours
  {
    for(int i=0; i<3; i++)
    {
      m_pos[i] += dt*m_vel[i];
    }
    m_prevHashKey = m_hashKey;
    computeHashKey();
    if(m_prevHashKey != m_hashKey)
    {
      m_system->updateHashTable(m_prevHashKey, m_hashKey, m_id);
    }
    setNeighboursList();
  }
  else if(_switchQuantity == 1) // update density and pressure
  {
    setDensity();
    setPressure();
  }
  else if(_switchQuantity == 2) // update forces
  {
    setPressureForce();
    setViscosity();
    setGravityForce();
    //setBuoyancyForce();
    setSurfaceTension();
    setSumForces();
  }
  else if(_switchQuantity == 3) // update acceleration and velocity
  {
    setAcceleration();
    setVelocity();
    setXSPHVelocity();
    checkBoundaryCollision();
  }
}

void Particle::draw(ngl::Mat4 _mouseTX)
{
  ngl::VAOPrimitives *prim=ngl::VAOPrimitives::instance();
  ngl::Transformation trans;
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();
  shader->use("Phong");
  ngl::Material m(ngl::STDMAT::PEWTER);
  m.loadToShader("material");

  ngl::Vec3 posVec;
  posVec.set(m_pos[0], m_pos[1], m_pos[2]);
  trans.setPosition(posVec);

  ngl::Mat4 MV;
  ngl::Mat4 MVP;
  ngl::Mat3 normalMatrix;
  ngl::Mat4 M;

  M=trans.getMatrix()*_mouseTX;
  MV=M*m_system->getCam()->getViewMatrix();
  MVP=MV*m_system->getCam()->getProjectionMatrix();
  normalMatrix=MV;
  normalMatrix.inverse();

  shader->setShaderParamFromMat4("MV",MV);
  shader->setShaderParamFromMat4("MVP",MVP);
  shader->setShaderParamFromMat3("normalMatrix",normalMatrix);
  shader->setShaderParamFromMat4("M",M);

  prim->draw("sphere");
}

void Particle::setId(int _i) {m_id = _i;}

int Particle::getId() {return m_id;}

void Particle::setPosition(std::vector<float> _p) {m_pos = _p;}

std::vector<float> Particle::getPosition() {return m_pos;}

void Particle::setMass(float _m) {m_mass = _m;}

float Particle::getMass() {return m_mass;}

void Particle::setVelocity()
{
  for(int i=0; i<3; i++)
  {
    m_vel[i] += dt*m_acc[i];
  }
}

void Particle::setVelocity(std::vector<float> _v)
{
  m_vel = {_v[0], _v[1], _v[2]};
}

std::vector<float> Particle::getVelocity() {return m_vel;}

void Particle::setXSPHVelocity()
{
  float e = 0.1;
  float sum = 0;
  float w;
  Particle p;
  for(int i=0; i<m_neighbours.size(); i++)
  {
    p = m_system->m_particles[m_neighbours[i]];
    w = kernel({m_pos[0]-p.m_pos[0], m_pos[1]-p.m_pos[1], m_pos[2]-p.m_pos[2]}, 0);
    for(int j=0; j<3; j++)
    {
      sum += (2*p.m_mass/(m_dens+p.m_dens))*(p.getVelocity()[j]-m_vel[j])*w;
    }
  }
  for(int i=0; i<3; i++)
  {
    m_vel[i] += e*sum;
  }
}

void Particle::setAcceleration()
{
  for(int i=0; i<3; i++)
  {
    m_acc[i] = m_sumForces[i]/m_dens;
  }
}

std::vector<float> Particle::getAcceleration() {return m_acc;}

void Particle::setDensity()
{
  float sum = rd; // starting with the rest density
  int n = m_neighbours.size();
  Particle p;
  for(int i=0; i<n; i++)
  {
    p = m_system->m_particles[m_neighbours[i]];
    sum += p.m_mass*kernel({m_pos[0]-p.m_pos[0], m_pos[1]-p.m_pos[1], m_pos[2]-p.m_pos[2]}, 0);
  }
  m_dens = sum;
}

float Particle::getDensity() {return m_dens;}

void Particle::setPressure()
{
  m_pres = kconst*(m_dens-rd);
}

float Particle::getPressure() {return m_pres;}

void Particle::setPressureForce()
{
  int n = m_neighbours.size();
  Particle p;
  std::vector<float> kgrad;
  kgrad = {0,0,0};
  for(int i=0; i<n; i++)
  {
    if(m_neighbours[i] != m_id)
    {
      p = m_system->m_particles[m_neighbours[i]];
      kgrad = kernelGradient({m_pos[0]-p.m_pos[0], m_pos[1]-p.m_pos[1], m_pos[2]-p.m_pos[2]}, 1);
      for(int j=0; j<3; j++)
      {
        m_fp[j] -= p.m_mass*((m_pres+p.m_pres)/(2*p.m_dens))*kgrad[j];
      }
    }
  }
}

std::vector<float> Particle::getPressureForce() {return m_fp;}

void Particle::setViscosity()
{
  int n = m_neighbours.size();
  Particle p;
  std::vector<float> e;
  float w;
  for(int i=0; i<n; i++)
  {
    if(m_neighbours[i] != m_id)
    {
      p = m_system->m_particles[m_neighbours[i]];
      e = p.getVelocity();
      w = kernelLaplacian({m_pos[0]-p.m_pos[0], m_pos[1]-p.m_pos[1], m_pos[2]-p.m_pos[2]}, 2);
      for(int j=0; j<3; j++)
      {
        m_fv[j] += mvisc*p.m_mass*((e[j]-m_vel[j])/p.m_dens)*w;
      }
    }
  }
}

std::vector<float> Particle::getViscosity() {return m_fv;}

void Particle::setGravityForce()
{
  for(int i=0; i<3; i++)
  {
    m_fg[i] = m_dens*g[i];
  }
}

std::vector<float> Particle::getGravityForce() {return m_fg;}

void Particle::setBuoyancyForce()
{
  for(int i=0; i<3; i++)
  {
    m_fb[i] = buoyancyConst*(m_dens-rd)*g[i];
  }
}

std::vector<float> Particle::getBuoyancyForce() {return m_fb;}

void Particle::setSurfaceTension()
{
  int ns = m_neighbours.size();
  Particle p;
  std::vector<float> n = {0,0,0}; // surface normal
  float c = 0; // colour field
  float k = 0; // curvature of surface

  for(int i=0; i<ns; i++)
  {
    p = m_system->m_particles[m_neighbours[i]];
    c += (p.m_mass/p.m_dens)*kernel({m_pos[0]-p.m_pos[0], m_pos[1]-p.m_pos[1], m_pos[2]-p.m_pos[2]}, 0);
    for(int j=0; j<3; j++)
    {
      n[j] += (p.m_mass/p.m_dens)*kernelGradient({m_pos[0]-p.m_pos[0], m_pos[1]-p.m_pos[1], m_pos[2]-p.m_pos[2]}, 0)[j];
    }
    k -= (p.m_mass/p.m_dens)*kernelLaplacian({m_pos[0]-p.m_pos[0], m_pos[1]-p.m_pos[1], m_pos[2]-p.m_pos[2]}, 0)/vectorLength(n, false);
  }
  if(vectorLength(n, false) >= lsurf)
  {
    for(int i=0; i<3; i++)
    {
      m_fst[i] = sigmasurf*k*n[i];
    }
  }
}

std::vector<float> Particle::getSurfaceTension() {return m_fst;}

void Particle::setSumForces()
{
  for(int i=0; i<3; i++)
  {
    m_sumForces[i] = m_fp[i]+m_fv[i]+m_fg[i]+m_fst[i]+m_dens*wind[i];
  }
}

std::vector<float> Particle::getSumForces() {return m_sumForces;}

int Particle::hashFunction(std::vector<float> _x)
{
  std::vector<int> d = {(int)floor(_x[0]/hl), (int)floor(_x[1]/hl), (int)floor(_x[2]/hl)};
  return ((d[0]*p1)^(d[1]*p2)^(d[2]*p3)) % nh;
}

void Particle::computeHashKey()
{
  m_hashKey = hashFunction(m_pos);
}

int Particle::getHashKey() {return m_hashKey;}

void Particle::setNeighboursList()
{
  float step = kh;
  std::vector<float> tempPos;
  m_neighbours.clear();
  m_neighbours = m_system->m_hashMap[m_hashKey];
  // bounding box corners
  std::vector<float> bbmin = {m_pos[0]-kh, m_pos[1]-kh, m_pos[2]-kh};
  std::vector<float> bbmax = {m_pos[0]+kh, m_pos[1]+kh, m_pos[2]+kh};
  int key;
  std::vector<int> tempList;
  std::vector<int> checkedKeys; // for positions whose hash keys have already been checked during the bounding box iterations

  for(float i=bbmin[0]; i<bbmax[0]; i+=step)
  {
    for(float j=bbmin[1]; j<bbmax[1]; j+=step)
    {
      for(float k=bbmin[2]; k<bbmax[2]; k+=step)
      {
        key = hashFunction({i,j,k}); // get a discretized position within the bounding box and its hash key
        // check if hash key has already been checked before, else go in the loop
        if((key != m_hashKey) && (std::find(checkedKeys.begin(), checkedKeys.end(), key)==checkedKeys.end()))
        {
          tempList = m_system->m_hashMap[key];
          for(int l=0; l<tempList.size(); l++)
          {
            if(std::find(m_neighbours.begin(), m_neighbours.end(), tempList[l]) == m_neighbours.end())
            {
              tempPos = m_system->m_particles[tempList[l]].m_pos;
              // check if candidate neighbour lies within the smoothing kernel radius
              if(vectorLength({m_pos[0]-tempPos[0],m_pos[1]-tempPos[1],m_pos[2]-tempPos[2]}, true) <= kh*kh)
                m_neighbours.push_back(tempList[l]);
            }
          }
          checkedKeys.push_back(key);
          tempList.clear();
        }
      }
    }
  }
}

std::vector<int> Particle::getNeighboursList() {return m_neighbours;}

void Particle::checkBoundaryCollision()
{
  float loseVel = 0.75;
  if(m_pos[0]+dt*m_vel[0]<=tankPos[0] || m_pos[0]+dt*m_vel[0]>=tankPos[0]+tankDims[0])
    m_vel[0] = -m_vel[0];
  if(m_pos[1]+dt*m_vel[1]<=tankPos[1]-tankDims[1]) // if the particle hits the floor
    m_vel[1] = -loseVel*m_vel[1];
  if(m_pos[1]+dt*m_vel[1]>=tankPos[1])
    m_vel[1] = -m_vel[1];
  if(m_pos[2]+dt*m_vel[2]<=tankPos[2] || m_pos[2]+dt*m_vel[2]>=tankPos[2]+tankDims[2])
    m_vel[2] = -m_vel[2];
}
