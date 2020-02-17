#include "pathtracer.h"

#include <iostream>
#include <algorithm>

#include <Eigen/Dense>

#include <util/CS123Common.h>

double GAMMA_CORR = 0.454545455;

using namespace Eigen;

PathTracer::PathTracer(int width, int height, quint16 num_samples)
    : m_width(width), m_height(height), m_num_samples(num_samples)
{
}

template <typename T>
T clamp(const T& v, const T& lower, const T& upper) {
    assert( !(upper < lower) );
    return (v < lower) ? lower : (upper < v) ? upper : v;
}

void PathTracer::traceScene(QRgb *imageData, const Scene& scene)
{
    std::vector<Vector3f> intensityValues(m_width * m_height);
    Matrix4f invViewMat = (scene.getCamera().getScaleMatrix() * scene.getCamera().getViewMatrix()).inverse();
    for(int y = 0; y < m_height; ++y)
    {
        //#pragma omp parallel for
        for(int x = 0; x < m_width; ++x)
        {
            int offset = x + (y * m_width); // i=(h-y-1)*w+x
            for(int sy = 0; sy < 2; sy++)
            {
                for(int sx = 0; sx < 2; sx++)
                {

                        intensityValues[offset] += tracePixel(x, y, scene, invViewMat); // c[i] = c[i] + Vec(clamp(r.x),clamp(r.y),clamp(r.z))*.25;

                }
            }
        }
    }

    toneMap(imageData, intensityValues);
}

Vector3f PathTracer::tracePixel(int x, int y, const Scene& scene, const Matrix4f &invViewMatrix)
{
    for(int s = 0; s < m_num_samples; s++)
    {

    }
    Vector3f p(0, 0, 0); //cam.o (camera position)
    //             cx  =     w      ,       cy =             , cam.d.z = -1
    Vector3f d((2.f * x / m_width) - 1, 1 - (2.f * y / m_height), -1); // Vec d
    d.normalize();

    Ray r(p, d);
    r = r.transform(invViewMatrix);
    return traceRay(r, scene);
}

Vector3f PathTracer::traceRay(const Ray& r, const Scene& scene)
{
    IntersectionInfo i;
    Ray ray(r);
    if(scene.getIntersection(ray, &i)) {
          //** Example code for accessing materials provided by a .mtl file **
//        const Triangle *t = static_cast<const Triangle *>(i.data);//Get the triangle in the mesh that was intersected
//        const tinyobj::material_t& mat = t->getMaterial();//Get the material of the triangle from the mesh
//        const tinyobj::real_t *d = mat.diffuse;//Diffuse color as array of floats
//        const std::string diffuseTex = mat.diffuse_texname;//Diffuse texture name
        return Vector3f(1, 1, 1);
    } else {
        return Vector3f(0, 0, 0);
    }
}

int clampIntensity(float v){
    v = clamp(v,0.0f,1.0f);
    return (int)(255 * qPow( v, GAMMA_CORR ) + 0.5);
}

void PathTracer::toneMap(QRgb *imageData, std::vector<Vector3f> &intensityValues) {
    for(int y = 0; y < m_height; ++y) {
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);
            std::cout << clampIntensity(intensityValues[offset](0)) << std::endl;
            imageData[offset] = qRgb(clampIntensity(intensityValues[offset](0)),
                                     clampIntensity(intensityValues[offset](1)),
                                     clampIntensity(intensityValues[offset](2)));
//            imageData[offset] = intensityValues[offset].norm() > 0 ? qRgb(255, 255, 255) : qRgb(40, 40, 40);
        }
    }

}
