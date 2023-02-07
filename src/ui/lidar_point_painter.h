#ifndef COLMAP_SRC_UI_LIDAR_POINT_PAINTER_H_
#define COLMAP_SRC_UI_LIDAR_POINT_PAINTER_H_

#include <QtCore>
#include <QtOpenGL>

namespace colmap {
class LidarPointPainter {
 public:
  LidarPointPainter();
  ~LidarPointPainter();

  struct Data {
    Data() : x(0), y(0), z(0), r(0), g(0), b(0), a(0) {}
    Data(const float x_, const float y_, const float z_, const float r_,
         const float g_, const float b_, const float a_)
        : x(x_), y(y_), z(z_), r(r_), g(g_), b(b_), a(a_) {}

    float x, y, z;
    float r, g, b, a;
  };  

  void Setup();
  void Upload(const std::vector<LidarPointPainter::Data>& data);
  void Render(const QMatrix4x4& pmv_matrix, const float point_size);

 private:
  QOpenGLShaderProgram shader_program_;
  QOpenGLVertexArrayObject vao_;
  QOpenGLBuffer vbo_;

  size_t num_geoms_;
};

}// namespace colmap

#endif 