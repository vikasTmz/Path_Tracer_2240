#ifndef PATHTRACER_H
#define PATHTRACER_H

#include <QImage>
#include <QtMath>

#include "scene/scene.h"
#include "erand48.h"


class PathTracer
{
public:
    PathTracer(int width, int height, quint16 num_samples);

    void traceScene(QRgb *imageData, const Scene &scene);

private:
    int m_width, m_height;
    quint16 m_num_samples;

    void toneMap(QRgb *imageData, std::vector<Eigen::Vector3f> &intensityValues);

    Eigen::Vector3f tracePixel(int x, int y, const Scene &scene, const Eigen::Matrix4f &invViewMatrix, int sx, int sy);
    Eigen::Vector3f traceRay(const Ray& r, const Scene &scene, uint depth, unsigned short *Xi);
};

#endif // PATHTRACER_H
