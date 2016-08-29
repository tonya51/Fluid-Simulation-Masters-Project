#include <QGuiApplication>
#include <QMouseEvent>
#include "Globals.h"
#include "NGLScene.h"
#include <ngl/Camera.h>
#include <ngl/Light.h>
#include <ngl/Material.h>
#include <ngl/NGLInit.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/ShaderLib.h>
#include <ngl/Transformation.h>

const static float INCREMENT=0.01;
const static float ZOOM=0.1;

NGLScene::NGLScene()
{
  m_rotate=false;
  m_spinXFace=0;
  m_spinYFace=0;
  setTitle("Fluid Simulation");
  m_fps=0;
  m_frames=0;
}

NGLScene::~NGLScene()
{

}

void NGLScene::resizeGL(QResizeEvent *_event)
{
  m_width = _event->size().width()*devicePixelRatio();
  m_height = _event->size().height()*devicePixelRatio();
  m_cam.setShape(45.0f,(float)width()/height(),0.05f,350.0f);
}

void NGLScene::resizeGL(int _w , int _h)
{
  m_cam.setShape(45.0f,(float)_w/_h,0.05f,350.0f);
  m_width = _w*devicePixelRatio();
  m_height = _h*devicePixelRatio();
}

void NGLScene::initializeGL()
{
  ngl::NGLInit::instance();
  glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_MULTISAMPLE);

  ngl::Vec3 from(0,2.5,8);
  ngl::Vec3 to(0,2.5,0);
  ngl::Vec3 up(0,1,0);
  m_cam.set(from,to,up);
  m_cam.setShape(60,(float)720.0/576.0,0.5,150);

  ngl::ShaderLib *shader=ngl::ShaderLib::instance();
  shader->createShaderProgram("Phong");
  shader->attachShader("PhongVertex",ngl::ShaderType::VERTEX);
  shader->attachShader("PhongFragment",ngl::ShaderType::FRAGMENT);
  shader->loadShaderSource("PhongVertex","shaders/Phong.vs");
  shader->loadShaderSource("PhongFragment","shaders/Phong.fs");
  shader->compileShader("PhongVertex");
  shader->compileShader("PhongFragment");
  shader->attachShaderToProgram("Phong","PhongVertex");
  shader->attachShaderToProgram("Phong","PhongFragment");
  shader->bindAttribute("Phong",0,"inVert");
  shader->bindAttribute("Phong",1,"inUV");
  shader->bindAttribute("Phong",2,"inNormal");
  shader->linkProgramObject("Phong");
  (*shader)["Phong"]->use();
  shader->setShaderParam1i("Normalize",1);

  ngl::Material m(ngl::STDMAT::PEWTER);
  m.loadToShader("material");
  ngl::Light light(ngl::Vec3(2,2,20),ngl::Colour(1,1,1,1),ngl::Colour(1,1,1,1),ngl::LightModes::POINTLIGHT);
  ngl::Mat4 iv = m_cam.getViewMatrix();
  iv.transpose();
  light.setTransform(iv);
  light.setAttenuation(1,0,0);
  light.enable();
  light.loadToShader("light");

  ngl::VAOPrimitives *prim = ngl::VAOPrimitives::instance();

  prim->createSphere("sphere",particleSize,10);
  m_system.reset(new System({0,0,0})); // initialize the particle system
  m_system->setCam(&m_cam); // set the camera
  m_tank = new Tank(tankPos, tankDims); // initialize the bounding box/tank
  m_tank->buildVAO(); // build the vertex array object for the bounding box/tank

  glViewport(0,0,width(),height());
  m_fpsTimer = startTimer(0);
  m_timer.start();
}

void NGLScene::paintGL()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  ngl::Mat4 rotX;
  ngl::Mat4 rotY;
  rotX.rotateX(m_spinXFace);
  rotY.rotateY(m_spinYFace);
  m_mouseGlobalTX=rotY*rotX;
  m_mouseGlobalTX.m_m[3][0] = m_modelPos.m_x;
  m_mouseGlobalTX.m_m[3][1] = m_modelPos.m_y;
  m_mouseGlobalTX.m_m[3][2] = m_modelPos.m_z;

  ngl::Mat4 MVP;
  MVP=m_mouseGlobalTX*m_cam.getVPMatrix();

  glViewport(0,0,m_width,m_height);
  m_system->draw(m_mouseGlobalTX); // draw the particle system
  m_tank->draw(m_cam, m_mouseGlobalTX); // draw the bounding box/tank
}

void NGLScene::mouseMoveEvent (QMouseEvent * _event)
{
  // note the method buttons() is the button state when event was called
  // this is different from button() which is used to check which button was
  // pressed when the mousePress/Release event is generated
  if(m_rotate && _event->buttons() == Qt::LeftButton)
  {
    int diffx=_event->x()-m_origX;
    int diffy=_event->y()-m_origY;
    m_spinXFace += (float) 0.5f * diffy;
    m_spinYFace += (float) 0.5f * diffx;
    m_origX = _event->x();
    m_origY = _event->y();
    update();

  }
        // right mouse translate code
  else if(m_translate && _event->buttons() == Qt::RightButton)
  {
    int diffX = (int)(_event->x() - m_origXPos);
    int diffY = (int)(_event->y() - m_origYPos);
    m_origXPos=_event->x();
    m_origYPos=_event->y();
    m_modelPos.m_x += INCREMENT * diffX;
    m_modelPos.m_y -= INCREMENT * diffY;
    update();

   }
}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::mousePressEvent ( QMouseEvent * _event)
{
  // this method is called when the mouse button is pressed in this case we
  // store the value where the maouse was clicked (x,y) and set the Rotate flag to true
  if(_event->button() == Qt::LeftButton)
  {
    m_origX = _event->x();
    m_origY = _event->y();
    m_rotate =true;
  }
  // right mouse translate mode
  else if(_event->button() == Qt::RightButton)
  {
    m_origXPos = _event->x();
    m_origYPos = _event->y();
    m_translate=true;
  }

}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::mouseReleaseEvent ( QMouseEvent * _event )
{
  // this event is called when the mouse button is released
  // we then set Rotate to false
  if (_event->button() == Qt::LeftButton)
  {
    m_rotate=false;
  }
        // right mouse translate mode
  if (_event->button() == Qt::RightButton)
  {
    m_translate=false;
  }
}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::wheelEvent(QWheelEvent *_event)
{

  // check the diff of the wheel position (0 means no change)
  if(_event->delta() > 0)
  {
    m_modelPos.m_z+=ZOOM;
  }
  else if(_event->delta() <0 )
  {
    m_modelPos.m_z-=ZOOM;
  }
  update();
}

void NGLScene::keyPressEvent(QKeyEvent *_event)
{
  // this method is called every time the main window recives a key event.
  // we then switch on the key value and set the camera in the GLWindow
  switch (_event->key())
  {
  case Qt::Key_Escape : QGuiApplication::exit(EXIT_SUCCESS); break;
  case Qt::Key_E : m_system->toggleExport(); break;
  default : break;
  }
  update();
}

void NGLScene::timerEvent(QTimerEvent *_event)
{
  m_system->update();
  if(_event->timerId()==m_fpsTimer)
  {
    if(m_timer.elapsed()>(1000.0*dt))
    {
      m_frames++;
      m_timer.restart();
    }
  }
  update();
}
