#include "ui/lidar_point_painter.h"

#include "util/opengl_utils.h"

namespace colmap{
LidarPointPainter::LidarPointPainter() : num_geoms_(0) {}

LidarPointPainter::~LidarPointPainter() {
  vao_.destroy();
  vbo_.destroy();
}

void LidarPointPainter::Setup(){
  vao_.destroy();
  vbo_.destroy();
  if (shader_program_.isLinked()) {
    shader_program_.release();
    shader_program_.removeAllShaders();
  }

  shader_program_.addShaderFromSourceFile(QOpenGLShader::Vertex,
                                          ":/shaders/points.v.glsl");
  shader_program_.addShaderFromSourceFile(QOpenGLShader::Fragment,
                                          ":/shaders/points.f.glsl");
  shader_program_.link();
  shader_program_.bind();

  vao_.create();
  vbo_.create();

#if DEBUG
  glDebugLog();
#endif  
}

void LidarPointPainter::Upload(const std::vector<LidarPointPainter::Data>& data) {
  // size_t num_geoms_
  num_geoms_ = data.size();
  if (num_geoms_ == 0) {
    return;
  }

  vao_.bind();
  vbo_.bind();

  // Upload data array to GPU
  vbo_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
  vbo_.allocate(data.data(),
                static_cast<int>(data.size() * sizeof(LidarPointPainter::Data)));

  // in_position
  shader_program_.enableAttributeArray("a_position");
  shader_program_.setAttributeBuffer("a_position", GL_FLOAT, 0, 3,
                                     sizeof(LidarPointPainter::Data));

  // in_color
  shader_program_.enableAttributeArray("a_color");
  shader_program_.setAttributeBuffer("a_color", GL_FLOAT, 3 * sizeof(GLfloat),
                                     4, sizeof(LidarPointPainter::Data));

  // Make sure they are not changed from the outside
  vbo_.release();
  vao_.release();

#if DEBUG
  glDebugLog();
#endif
}

void LidarPointPainter::Render(const QMatrix4x4& pmv_matrix,
                          const float point_size) {
  if (num_geoms_ == 0) {
    return;
  }

  shader_program_.bind();
  vao_.bind();

  shader_program_.setUniformValue("u_pmv_matrix", pmv_matrix);
  shader_program_.setUniformValue("u_point_size", point_size);

  QOpenGLFunctions* gl_funcs = QOpenGLContext::currentContext()->functions();
  gl_funcs->glDrawArrays(GL_POINTS, 0, (GLsizei)num_geoms_);

  // Make sure the VAO is not changed from the outside
  vao_.release();

#if DEBUG
  glDebugLog();
#endif
}

} // colmap