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

//double erand48(unsigned short xsubi[3]) {
//    return (double)rand() / (double)RAND_MAX;
//}

void PathTracer::traceScene(QRgb *imageData, const Scene& scene)
{
    std::vector<Vector3f> intensityValues(m_width * m_height);
    Matrix4f invViewMat = (scene.getCamera().getScaleMatrix() * scene.getCamera().getViewMatrix()).inverse();

    #pragma omp parallel for collapse(2)
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
                    intensityValues[offset] = tracePixel(x, y, scene, invViewMat, sx, sy); // c[i] = c[i] + Vec(clamp(r.x),clamp(r.y),clamp(r.z))*.25;

                }
            }
        }
    }

    toneMap(imageData, intensityValues);
}

Vector3f PathTracer::tracePixel(int x, int y, const Scene& scene, const Matrix4f &invViewMatrix, int sx, int sy)
{
    Vector3f out(0, 0, 0);
    Vector3f p(0, 0, 0); //cam.o (camera position)
    uint depth = 0;
    unsigned short Xi[3]={0,0,y*y*y};
    float xcom, ycom;
    double r1, r2, dx, dy;

    for(int s = 0; s < m_num_samples; s++)
    {

        r1 = 2 * erand48(Xi);
        dx=r1<1 ? qSqrt(r1)-1: 1-qSqrt(2-r1);
        r2=2*erand48(Xi);
        dy=r2<1 ? qSqrt(r2)-1: 1-qSqrt(2-r2);

        // Vec d = cx * ( ( (sx+.5 + dx)/2 + x)/w - .5) + cy * ( ( (sy+.5 + dy)/2 + y)/h - .5) + cam.d;
        // (w/h) * (  ( x + (sx+.5 + dx)/2 ) / m_width - .5  )
        xcom = x + 0.5 * sx + 0.25 + 0.5 * dx;
        ycom = y + 0.5 * sy + 0.25 + 0.5 * dy;

        //             cx  =     w      ,       cy =             , cam.d.z = -1
        Vector3f d((2.f * xcom / m_width) - 1, 1 - (2.f * ycom / m_height), -1); // Vec d
        d.normalize();

        Ray r(p, d); // Ray(cam.o+d*140, d.norm())
        r = r.transform(invViewMatrix);

        out += traceRay(r, scene, depth, Xi); // r = r + radiance( Ray(cam.o+d*140, d.norm()) ,0,Xi);
    }

    out = out * (1./m_num_samples);

    return out;
}

Vector3f PathTracer::traceRay(const Ray& r, const Scene& scene, uint depth, unsigned short *Xi)
{
    IntersectionInfo i;
    Ray ray(r);
    if(scene.getIntersection(ray, &i) && depth <= 10) {

          //** Example code for accessing materials provided by a .mtl file **
        const Mesh *m = static_cast<const Mesh *>(i.object);
        const Triangle *t = static_cast<const Triangle *>(i.data);//Get the triangle in the mesh that was intersected
        const tinyobj::material_t& mat = t->getMaterial();//Get the material of the triangle from the mesh
//        const std::string diffuseTex = mat.diffuse_texname;//Diffuse texture name
        const tinyobj::real_t *diffuse = mat.diffuse;//Diffuse color as array of floats
        const tinyobj::real_t *emission = mat.emission;

        Vector3f normal = t->getNormal(i).normalize(); // surface normal   ,  n
        Vector3f surf_normal = normal.dot(r.d) < 0 ? normal : -normal ; // surface normal fixed for orientation , nl

        Vector3f L = Vector3f(diffuse[0], diffuse[1], diffuse[2]); // Vec f = obj.c;
        double p = L[0] > L[1] && L[0] > L[2] ? L[0] : L[1] > L[2] ? L[1] : L[2];

        // Russian roulette termination.
        // If random number between 0 and 1 is > p, terminate and return hit object's emmission

        if (++depth > 5)
        {
            if (erand48(Xi) < p * 0.9)
            { // Multiply by 0.9 to avoid infinite loop with colours of 1.0
                L = L * (0.9 / p);
            }
            else
            {
                return Vector3f(emission[0], emission[1], emission[2]);
            }
        }

        // Diffuse
        if (mat.illum == 2)
        {
            double r1 = 2 * EIGEN_PI * erand48(Xi);
            double r2 = erand48(Xi);
            double r2s = sqrt(r2);
            Vector3f u = ( ( qFabs(surf_normal[0]) > 0.1 ? Vector2f(0,1) : VectorXf(1) ) % surf_normal ).normalize();
            Vector3f v = surf_normal % u;
            Vector3f d = (u * qCos(r1) * r2s + v * qSin(r1) * r2s + surf_normal * qSqrt(1 - r2)).normalize();

        }


//        std::cout << "i.hit = " << i.hit << std::endl;

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
//            std::cout << clampIntensity(intensityValues[offset](0)) << std::endl;
            imageData[offset] = qRgb(clampIntensity(intensityValues[offset](0)),
                                     clampIntensity(intensityValues[offset](1)),
                                     clampIntensity(intensityValues[offset](2)));
//            imageData[offset] = intensityValues[offset].norm() > 0 ? qRgb(255, 255, 255) : qRgb(40, 40, 40);
        }
    }

}
