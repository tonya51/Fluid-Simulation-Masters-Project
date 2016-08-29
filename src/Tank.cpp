#include "Tank.h"
#include "Globals.h"
#include <iostream>
#include <memory>
#include <ngl/Camera.h>
#include <ngl/Light.h>
#include <ngl/Material.h>
#include <ngl/NGLInit.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/ShaderLib.h>
#include <ngl/Transformation.h>

Tank::Tank()
{
  m_pos = tankPos;
  m_dim = tankDims;
}

Tank::Tank(std::vector<float> _p, std::vector<float> _d)
{
  m_pos.resize(3);
  m_pos[0] = _p[0];
  m_pos[1] = _p[1];
  m_pos[2] = _p[2];
  m_dim = _d;
}

Tank::~Tank()
{

}

void Tank::draw(ngl::Camera _c, ngl::Mat4 _mouseTX)
{
  ngl::Transformation trans;
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();
  (*shader)["Phong"]->use();
  ngl::Vec3 posVec;
  posVec.set(0,0,0);
  trans.setPosition(posVec);

  ngl::Mat4 MV;
  ngl::Mat4 MVP;
  ngl::Mat3 normalMatrix;
  ngl::Mat4 M;

  M=trans.getMatrix()*_mouseTX;
  MV=M*_c.getViewMatrix();
  MVP=MV*_c.getProjectionMatrix();
  normalMatrix=MV;
  normalMatrix.inverse();

  shader->setShaderParamFromMat4("MV",MV);
  shader->setShaderParamFromMat4("MVP",MVP);
  shader->setShaderParamFromMat3("normalMatrix",normalMatrix);
  shader->setShaderParamFromMat4("M",M);

  m_t->bind();
  m_t->draw();
  m_t->unbind();
}

void Tank::buildVAO()
{
  m_t = ngl::VertexArrayObject::createVOA(GL_LINE_STRIP);
  m_t->bind();
  const static GLubyte indices[] = {0,1,2,3,0,4,5,1,5,6,2,6,7,3,7,4};
  std::vector<float> p = m_pos;
  std::vector<float> d = m_dim;
  float s = 0;

  GLfloat vertices[] = {p[0],     p[1]+2*s,     p[2],
                        p[0]+d[0]+2*s,p[1]+2*s,     p[2],
                        p[0]+d[0]+2*s,p[1]+2*s,     p[2]+d[2]+2*s,
                        p[0],     p[1]+2*s,     p[2]+d[2]+2*s,
                        p[0],     p[1]-d[1],p[2],
                        p[0]+d[0]+2*s,p[1]-d[1],p[2],
                        p[0]+d[0]+2*s,p[1]-d[1],p[2]+d[2]+2*s,
                        p[0],     p[1]-d[1],p[2]+d[2]+2*s};

  GLfloat colours[] = {0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0,
                       0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0};

  m_t->setIndexedData(24*sizeof(GLfloat),vertices[0],sizeof(indices),&indices[0],GL_UNSIGNED_BYTE,GL_STATIC_DRAW);
  m_t->setVertexAttributePointer(0,3,GL_FLOAT,0,0);
  m_t->setIndexedData(24*sizeof(GLfloat),colours[0],sizeof(indices),&indices[0],GL_UNSIGNED_BYTE,GL_STATIC_DRAW);
  m_t->setVertexAttributePointer(1,3,GL_FLOAT,0,0);
  m_t->setNumIndices(sizeof(indices));
  m_t->unbind();
}
