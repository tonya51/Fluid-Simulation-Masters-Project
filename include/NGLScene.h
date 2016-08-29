#ifndef NGLSCENE_H__
#define NGLSCENE_H__
#include "Globals.h"
#include <ngl/Camera.h>
#include <ngl/Colour.h>
#include <ngl/Light.h>
#include <ngl/Text.h>
#include "System.h"
#include <QTime>
#include <QOpenGLWindow>
#include <memory>

/// @file NGLScene.h
/// @brief this class inherits from the Qt OpenGLWindow and allows us to use NGL to draw OpenGL
/// @author Jonathan Macey
/// @version 1.0
/// @date 10/9/13
/// Revision History :
/// This is an initial version used for the new NGL6 / Qt 5 demos
/// @class NGLScene
/// @brief our main glwindow widget for NGL applications all drawing elements are
/// put in this file
/// Methods NGLScene, ~NGLScene, initializeGL, paintGL modified by Antonia Strantzi

class NGLScene : public QOpenGLWindow
{
public:
  NGLScene();
  ~NGLScene();
  void initializeGL();
  void paintGL();
  void resizeGL(QResizeEvent *_event);
  void resizeGL(int _w, int _h);

private:
  ngl::Camera m_cam;
  int m_fpsTimer, m_fps, m_frames;
  QTime m_timer;
  int m_width, m_height;
  std::unique_ptr<System> m_system;
  Tank *m_tank;

  int m_spinXFace;
  int m_spinYFace;
  bool m_rotate;
  bool m_translate;
  int m_origX;
  int m_origY;
  int m_origXPos;
  int m_origYPos;
  ngl::Mat4 m_mouseGlobalTX;
  ngl::Vec3 m_modelPos;
  void keyPressEvent(QKeyEvent *_event);
  void mouseMoveEvent(QMouseEvent *_event);
  void mousePressEvent(QMouseEvent *_event);
  void mouseReleaseEvent(QMouseEvent *_event);
  void wheelEvent(QWheelEvent *_event);

  void loadMatricesToShader();
  void timerEvent(QTimerEvent *);
};

#endif // NGLSCENE_H
