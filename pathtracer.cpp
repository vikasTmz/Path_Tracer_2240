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

std::default_random_engine rnd(1);
std::uniform_real_distribution<float> dis(0, 1);

float PathTracer::random() {
    return dis(rnd);
}

//double erand48(unsigned short xsubi[3]) {
//    return (double)rand() / (double)RAND_MAX;
//}

void PathTracer::traceScene(QRgb *imageData, const Scene& scene)
{
    std::vector<Vector3f> intensityValues(m_width * m_height);
    Matrix4f invViewMat = (scene.getCamera().getScaleMatrix() * scene.getCamera().getViewMatrix()).inverse();

//    #pragma omp parallel for collapse(2)
    #pragma omp parallel for schedule(dynamic, 1)
    for(int y = 0; y < m_height; ++y)
    {
        fprintf(stderr,"\rRendering (%d spp) %5.2f%%",m_num_samples*4,100.*y/(m_height-1));
        for(int x = 0; x < m_width; ++x)
        {
            int offset = x + (y * m_width); // i=(h-y-1)*w+x
//            for(int sy = 0; sy < 2; sy++)
//            {
//                for(int sx = 0; sx < 2; sx++)
//                {
                    intensityValues[offset] = tracePixel(x, y, scene, invViewMat, 0, 0); // c[i] = c[i] + Vec(clamp(r.x),clamp(r.y),clamp(r.z))*.25;
//                }
//            }
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
//    float xcom, ycom;
//    double r1, r2, dx, dy;

    for(int s = 0; s < m_num_samples; s++)
    {

//        r1 = 2 * erand48(Xi);
//        dx=r1<1 ? qSqrt(r1)-1: 1-qSqrt(2-r1);
//        r2=2*erand48(Xi);
//        dy=r2<1 ? qSqrt(r2)-1: 1-qSqrt(2-r2);

//        // Vec d = cx * ( ( (sx+.5 + dx)/2 + x)/w - .5) + cy * ( ( (sy+.5 + dy)/2 + y)/h - .5) + cam.d;
//        // (w/h) * (  ( x + (sx+.5 + dx)/2 ) / m_width - .5  )
//        xcom = x + 0.5 * sx + 0.25 + 0.5 * dx;
//        ycom = y + 0.5 * sy + 0.25 + 0.5 * dy;

//        //             cx  =     w      ,       cy =             , cam.d.z = -1
//        Vector3f d((2.f * xcom / m_width) - 1, 1 - (2.f * ycom / m_height), -1); // Vec d
//        d.normalize();
        Vector3f d((2.f * x / m_width) - 1, 1 - (2.f * y / m_height), -1); // Vec d
        d.normalize();

        Ray r(p, d); // Ray(cam.o+d*140, d.norm())
        r = r.transform(invViewMatrix);

        out += traceRay(r, scene, depth, Xi); // r = r + radiance( Ray(cam.o+d*140, d.norm()) ,0,Xi);
    }

    out = out * (1./m_num_samples);

    return out;
}

Vector3f tangentConvert(const Vector3f& pointdir, const Vector3f& normal) {
    Vector3f tangent = qFabs(normal[0]) > qFabs(normal[1]) ? Vector3f(normal[2], 0, -normal[0]) : Vector3f(0, -normal[2], normal[1]);
    tangent.normalize();
    Vector3f bitangent = normal.cross(tangent);

    return Vector3f(pointdir[0] * bitangent[0] + pointdir[1] * normal[0] + pointdir[2] * tangent[0],
                    pointdir[0] * bitangent[1] + pointdir[1] * normal[1] + pointdir[2] * tangent[1],
                    pointdir[0] * bitangent[2] + pointdir[1] * normal[2] + pointdir[2] * tangent[2]).normalized();
}

void cosineSampleHemisphere(const Vector3f &normal, Vector3f &wi, float &pdf, unsigned short *Xi)
{
    double r1 = erand48(Xi);
    double r2 = erand48(Xi);

    float phi = 2.f * EIGEN_PI * r1;
    float sr2 = qSqrt(r2);

    wi = tangentConvert(Vector3f(sr2 * qCos(phi), qSqrt(1.0f - r2), sr2 * qSin(phi)).normalized(), normal);
    pdf = normal.dot(wi) / EIGEN_PI;
}

// Possible bugs
Vector3f directLighting(const Vector3f& hit, const Vector3f& normal, const Scene& scene) {
    Vector3f intensity(0, 0, 0);

    for (Object* light : *scene.lights) {
        Vector3f lightSample = light->sample();
        Vector4f lightSample4d = Vector4f(lightSample[0], lightSample[1], lightSample[2], 1);
        Vector4f lightTransformVec = light->transform.matrix() * lightSample4d;

//        Vector3f lightPoint = Vector3f(light->transform() * Vector4f(light->sample(), 1));
        Vector3f lightPoint = Vector3f(lightTransformVec[0],lightTransformVec[1],lightTransformVec[2]);

        Vector3f toLight = (lightPoint - hit);
        float distSquare = qPow(toLight[0],2) + qPow(toLight[1],2) + qPow(toLight[2],2);
        toLight.normalize();

        IntersectionInfo i;
        Ray newray(hit + toLight*FLOAT_EPSILON, toLight);

        if (scene.getIntersection(newray, &i) && i.object == light) {
            const Mesh * m = static_cast<const Mesh *>(i.object);
            const Triangle *t = static_cast<const Triangle *>(i.data);
            const tinyobj::material_t& mat = m->getMaterial(t->getIndex());

            if (toLight.dot(t->getNormal(i)) > 0.0f ) continue;
            float ndotl = clamp(toLight.dot(normal), 0.f, 1.f);

            intensity += Vector3f(mat.emission[0], mat.emission[1], mat.emission[2]) * (light->getSurfaceArea() * ndotl * qAbs(toLight.dot((t->getNormal(i)).normalized() )) / distSquare);
        }
    }

    return intensity;
}

Vector3f vectmod(Vector3f a, Vector3f b) {
    return Vector3f(a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]);
}

Vector3f PathTracer::traceRay(const Ray& r, const Scene& scene, uint depth, unsigned short *Xi)
{
    Vector3f L = Vector3f(0,0,0); // radiance

    IntersectionInfo i;
    Ray ray(r);                             // maxdepth
    if(scene.getIntersection(ray, &i) && depth < 10) {

          //** Example code for accessing materials provided by a .mtl file **
        const Mesh *m = static_cast<const Mesh *>(i.object);
        const Triangle *t = static_cast<const Triangle *>(i.data);//Get the triangle in the mesh that was intersected
        const tinyobj::material_t& mat = t->getMaterial();//Get the material of the triangle from the mesh
//        const std::string diffuseTex = mat.diffuse_texname;//Diffuse texture name
        const tinyobj::real_t *diffuse = mat.diffuse;//Diffuse color as array of floats
        const tinyobj::real_t *emission = mat.emission;
        const tinyobj::real_t *specular = mat.specular;

/////////////////////////////////////////////////////////////////////////////////////
//                  NEW  --- START
/////////////////////////////////////////////////////////////////////////////////////

        if (!depth)
            L +=Vector3f(emission);

        Vector3f normal = t->getNormal(i);
        normal.normalize(); // surface normal   ,  n
        Vector3f surf_normal = normal.dot(ray.d) < 0 ? normal : -normal ; // surface normal fixed for orientation , nl

        // Diffuse
        if (mat.illum == 2)
        {                           // mindepth
            const float pdf_rr = depth < 1 ? 1.0f : qMin(qMax(diffuse[0], qMax(diffuse[1], diffuse[2])), 0.99f);
            Vector3f albedo = Vector3f(diffuse[0],diffuse[1],diffuse[3]) / EIGEN_PI;
            if (erand48(Xi) < pdf_rr)
            {
                Vector3f wi;
                float pdf;
                cosineSampleHemisphere(normal, wi, pdf, Xi);
                const float illum_scale = wi.dot(normal) / (pdf * pdf_rr);

                Vector3f directlight =  directLighting(i.hit, normal, scene);//* illum_scale;
                Vector3f indirectlight = traceRay(Ray(i.hit + FLOAT_EPSILON * wi, wi), scene, depth + 1, Xi) * illum_scale;
                L += (albedo.array() * directlight.array()).matrix();
                L += (albedo.array() * indirectlight.array()).matrix();

            }
        }

        // Perfect specular
        else if (mat.illum == 5)
        {
            Vector3f albedo = Vector3f(1.0f, 1.0f, 1.0f);
            const float pdf_rr = depth < 1 ? 1.0f : qMin(qMax(specular[0], qMax(specular[1], specular[2])), 0.99f);
            if (erand48(Xi) < pdf_rr)
            {
              Vector3f refl = (ray.d - 2.f * normal * normal.dot(ray.d)).normalized();
              if ( normal.dot(ray.d) >= 0.0f ) refl = ray.d;
              Vector3f indirectlight = traceRay(Ray(i.hit + FLOAT_EPSILON * refl, refl), scene, depth + 1, Xi) / pdf_rr;
              L += (albedo.array() * indirectlight.array()).matrix();
            }
        }

        // Refraction
        else if (mat.illum == 7)
        {
            Vector3f albedo = Vector3f(1.0f, 1.0f, 1.0f);
            const float pdf_rr = depth < 1 ? 1.f : 0.95f;
            if (erand48(Xi) < pdf_rr) {
                const Vector3f refl = (ray.d - 2.f * normal * normal.dot(ray.d)).normalized();
                const float ni = 1.f;
                const float nt = mat.ior;
                const float ratio = normal.dot(ray.d) < 0 ? ni / nt : nt / ni;

                const float costheta = ray.d.dot(surf_normal);
                const float radicand = 1.f - ratio * ratio * (1.f - costheta*costheta);

                // TODO m_full
                Vector3f indirectlight(0,0,0);
                if (radicand < FLOAT_EPSILON) {
                  indirectlight = traceRay(Ray(i.hit + FLOAT_EPSILON * refl, refl), scene, depth + 1, Xi) / pdf_rr;
                } else {
                    Vector3f refr;
                    if (normal.dot(ray.d) < 0) {
                        refr = ray.d * ratio - normal * (costheta * ratio + qSqrt(radicand));
                    } else {
                        refr = ray.d * ratio + normal * (costheta * ratio + qSqrt(radicand));
                    }
                    const float R0 = (nt - ni) * (nt - ni) / ((nt + ni) * (nt + ni));
                    const float Rtheta = R0 + (1.f - R0) * qPow(1.f - (normal.dot(ray.d) < 0 ? -costheta : refr.dot(normal)), 5);
                    if (erand48(Xi) < Rtheta) {
                      indirectlight = traceRay(Ray(i.hit + FLOAT_EPSILON * refl, refl), scene, depth + 1, Xi) / pdf_rr;
                    } else {
                      indirectlight = traceRay(Ray(i.hit + FLOAT_EPSILON * refr, refr), scene, depth + 1, Xi) / pdf_rr;
                    }
                }
                 L += (albedo.array() * indirectlight.array()).matrix();
            }

        }
/////////////////////////////////////////////////////////////////////////////////////
//                  NEW  --- END
/////////////////////////////////////////////////////////////////////////////////////

        return L;
    }
    else
    {
        return Vector3f(0, 0, 0);
    }
}

int clampIntensity(float v){
//    v = clamp(v,0.0f,1.0f);
//    return (int)(255.0f * qPow( v, GAMMA_CORR ) + 0.5);
    return (int)(255.0f * v / (1.0f + v));
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
