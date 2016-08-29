#include "System.h"
#include "Tank.h"
#include "Globals.h"
#include <iostream>
#include <ngl/Random.h>
#include <ngl/Transformation.h>
#include <ngl/ShaderLib.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/Logger.h>
#include <ngl/NGLStream.h>
#include <QElapsedTimer>
#include <ngl/RibExport.h>
#include <boost/format.hpp>
#include <array>
#include <QGuiApplication>
namespace AbcG = Alembic::AbcGeom;

bool isPrime(int _n)
{
  for(int i=2;i<_n/2;i++)
  {
    if(_n%i==0)
      return false;
  }
  return true;
}

int nextPrime(int _x)
{
  bool q = false;
  if(_x%2==0)
    _x--;
  while(q==false)
  {
    _x+=2;
    if(isPrime(_x)==true)
      q = true;
  }
  return _x;
}

std::vector<float> System::calcPosition(int _i)
{
  // method to stack particles in rows and columns
  float ps = 2*particleSize;
  int ty = (int)_i/((volumeDims[0]/ps)*(volumeDims[2]/ps));
  int rest = _i-(ty*(volumeDims[0]/ps)*(volumeDims[2]/ps));
  int tz = (int)rest/(volumeDims[0]/ps);
  int tx = rest-(tz*(volumeDims[0]/ps));
  return {(tx*ps)+particleSize-tankDims[0]/2, ty*ps+particleSize, -(tz*ps)-particleSize-(tankDims[2]/2-volumeDims[2])};
}

System::System()
{

}

System::System(std::vector<float> _p)
{
  m_pos = _p;
  std::cout<<"Number of particles: "<<totalParticles<<" - export filename: "<<fileName<<std::endl;
  std::cout<<"kernel size = "<<kh<<std::endl;
  std::vector<float> startPos;
  Particle *prt;
  std::vector<float> sp;

  nh = nextPrime(2*totalParticles); // calculate hash table size

  for(int i=0; i<totalParticles; i++)
  {
    if(shapeType=="box") // a cube of stacked particles
      startPos = calcPosition(i);
    else if(shapeType=="sphere") // a sphere of particles
    {
      float px, py, pz;
      px = ((float)rand()/RAND_MAX)*(2*rad)-rad;
      py = ((float)rand()/RAND_MAX)*(2*rad)-rad;
      pz = ((float)rand()/RAND_MAX)*(2*rad)-rad;
      while(px*px+py*py+pz*pz > rad*rad)
      {
        px = ((float)rand()/RAND_MAX)*(2*rad)-rad;
        py = ((float)rand()/RAND_MAX)*(2*rad)-rad;
        pz = ((float)rand()/RAND_MAX)*(2*rad)-rad;
      }
      startPos = {px,py+2,pz};
    }
    else if(shapeType=="boxRandom") // randomly placed particles within a cube
    {
      float px, py, pz;
      px = ((float)rand()/RAND_MAX)*volumeDims[0];
      py = ((float)rand()/RAND_MAX)*volumeDims[1];
      pz = ((float)rand()/RAND_MAX)*volumeDims[2];
      startPos = {px-tankDims[0]/2,py,pz-tankDims[2]/2};
    }

    prt = new Particle(i, startPos, this); // create particle
    prt->setMass(mass);
    prt->computeHashKey();
    makeHashTable(prt->getHashKey(), prt->getId());
    m_particles.push_back(*prt); // push back into the system's vector of particles
  }

  for(int i=0; i<totalParticles; i++)
  {
    m_particles[i].setNeighboursList();
  }
  for(int i=0; i<totalParticles; i++)
  {
    m_particles[i].setDensity();
    m_particles[i].setPressure();
  }
  for(int i=0; i<totalParticles; i++)
  {
    m_particles[i].setPressureForce();
    m_particles[i].setViscosity();
    m_particles[i].setGravityForce();
    //m_particles[i].setBuoyancy();
    m_particles[i].setSurfaceTension();
    m_particles[i].setSumForces();
  }
  for(int i=0; i<totalParticles; i++)
  {
    m_particles[i].setAcceleration();
    m_particles[i].setVelocity();
    m_particles[i].checkBoundaryCollision();
  }

  m_archive.reset(new AbcG::OArchive(Alembic::AbcCoreOgawa::WriteArchive(),fileName) );

  AbcG::TimeSampling ts(1.0f/24.0f, 0.0f);
  AbcG::OObject topObj( *m_archive.get(), AbcG::kTop );
  Alembic::Util::uint32_t tsidx = topObj.getArchive().addTimeSampling(ts);
  m_partsOut.reset( new AbcG::OPoints(topObj, "simpleParticles", tsidx) );
  m_rgbOut.reset(new AbcG::OC4fArrayProperty( m_partsOut->getSchema(), ".colour",false,AbcG::kVertexScope, tsidx ));
}

System::~System()
{
  delete(this);
}

void System::draw(ngl::Mat4 _mouseTX)
{
  for(int i=0; i<totalParticles; i++)
  {
    m_particles[i].draw(_mouseTX);
  }
  if(m_export==true)
  {
    exportFrame();
  }
}

void System::update()
{
  if(updates == 0)
  {
    time_t starting = time(0);
    char *timestr = ctime(&starting);
    std::cout<<"Starting at: "<<timestr<<std::endl;
  }
  for(int i=0; i<totalParticles; i++)
  {
    m_particles[i].update(0);
  }
  for(int i=0; i<totalParticles; i++)
  {
    m_particles[i].update(1);
  }
  for(int i=0; i<totalParticles; i++)
  {
    m_particles[i].update(2);
  }
  for(int i=0; i<totalParticles; i++)
  {
    m_particles[i].update(3);
  }

  std::cout<<"Loop: "<<updates++<<std::endl<<"......................"<<std::endl;
  if(updates==updateCycles+1) // kill frame export before exiting the programme
  {
    m_export=false;
    time_t ending = time(0);
    char *timestr2 = ctime(&ending);
    std::cout<<"Ending at: "<<timestr2<<std::endl;
    std::cout<<"export false"<<std::endl;
  }
  if(updates==updateCycles+2)
  {
    QGuiApplication::exit(EXIT_SUCCESS);
  }
}

void System::setCam(ngl::Camera *_c) {m_cam = _c;}

ngl::Camera * System::getCam()const{return m_cam;}

void System::makeHashTable(int _x, int _p)
{
  m_hashMap[_x].push_back(_p);
}

void System::updateHashTable(int _p, int _h, int _i)
{
  // _p previous hash key
  // _h new hash key
  // _i particle id
  int j;
  for(j=0; j<m_hashMap[_p].size(); j++) // find particle's old position in the hash table using its previous hash key
  {
    if(m_hashMap[_p][j] == _i)
      break;
  }
  m_hashMap[_p].erase(m_hashMap[_p].begin()+j); // delete particle from previous position
  makeHashTable(_h, _i); // add particle to new position according to new hash key
}

void System::exportFrame()
{
  static int frame=0;
  ++frame;
  // this is the data we are going to store, alembic uses Imath
  // internally so we convert from ngl
  // this is the array of particle positions for the frame
  std::vector<Imath::V3f> positions;
  // these are the particle id's which are required so use use index no
  std::vector<Alembic::Util::uint64_t> id;
  // set this to push back into the array
  Imath::V3f data;
  // colour values
  Imath::C4f c;
  std::vector<Imath::C4f> colours;
  //std::vector<GLfloat> getPos;

  for(unsigned int  i=0; i<totalParticles; i++)
  {
    //getPos = {(GLfloat)
    positions.push_back(Imath::V3f(m_particles[i].getPosition()[0],(GLfloat)m_particles[i].getPosition()[1],m_particles[i].getPosition()[2]));
    id.push_back(i);
    colours.push_back(Imath::C4f(1.0f,0.0f,0.0f,1.0f));
  }
  // create as samples we need to do this else we get a most vexing parse
  // https://en.wikipedia.org/wiki/Most_vexing_parse using below
  // psamp(V3fArraySample( positions),UInt64ArraySample(id))
  AbcG::V3fArraySample pos(positions);
  AbcG::UInt64ArraySample ids(id);
  AbcG::OPointsSchema::Sample psamp( pos,ids );

  m_partsOut->getSchema().set( psamp );
  AbcG::C4fArraySample colourArray(colours);
  m_rgbOut->set(colourArray);

}

Particle System::getParticle(int _i) {return m_particles[_i];}

std::map<int, std::vector<int>> System::getHashTable() {return m_hashMap;}
