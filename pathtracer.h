#ifndef PATHTRACER_H
#define PATHTRACER_H

#include <QImage>
#include <QtMath>
#include <QTime>

#include "scene/scene.h"
#include "erand48.h"


class PathTracer
{
public:
    PathTracer(int width, int height, quint16 num_samples);

    void traceScene(QRgb *imageData, const Scene &scene);
    static float random();

private:
    int m_width, m_height, m_mindepth, m_maxdepth;
    quint16 m_num_samples;

    float brdfLimit(const tinyobj::real_t *material, float thresh);
    const float russianRouletteBRDF(uint depth, float low);
    void depthOfField(Eigen::Vector3f &orientation, Eigen::Vector3f &direction, Eigen::Vector3f &p,
                      Eigen::Vector3f &d, float focal, float aperture, unsigned short *ER48SEED);

    void toneMap(QRgb *imageData, std::vector<Eigen::Vector3f> &intensityValues);

    Eigen::Vector3f tracePixel(int x, int y, const Scene &scene, const Eigen::Matrix4f &invViewMatrix);
    Eigen::Vector3f traceRay(const Ray& r, const Scene &scene, uint depth, unsigned short *ER48SEED, bool itsLit = true);
    Eigen::Vector3f directLighting(const Scene &scene, const Eigen::Vector3f& normal, const Eigen::Vector3f& hit);
    void sampleHemisphereImportance(Eigen::Vector3f &wi, float &pdf, const Eigen::Vector3f &normal,
                                    const tinyobj::material_t& mat, unsigned short *ER48SEED);

};

#endif // PATHTRACER_H
