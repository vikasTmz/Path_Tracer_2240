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
//Vector3f directLighting(const Vector3f& hit, const Vector3f& normal, const Scene& scene) {
//    Vector3f intensity(0, 0, 0);

//    for (Object* light : *scene.lights) {
////        Vector3f lightPoint = Vector3f(light->transform);// * Vector4f(light->sample(), 1));

//        Vector3f toLight = (hit);
//        float distSquare = qPow(toLight[0],2) + qPow(toLight[1],2) + qPow(toLight[2],2);
//        toLight.normalize();

//        IntersectionInfo i;
//        Ray newray(hit + toLight*FLOAT_EPSILON, toLight);

//        if (scene.getIntersection(newray, &i) && i.object == light) {
//            const Mesh * m = static_cast<const Mesh *>(i.object);
//            const Triangle *t = static_cast<const Triangle *>(i.data);
//            const tinyobj::material_t& mat = m->getMaterial(t->getIndex());

//            if (toLight.dot(t->getNormal(i)) > 0.0f ) continue;
//            float ndotl = clamp(toLight.dot(normal), 0.f, 1.f);

//            intensity += Vector3f(mat.emission[0], mat.emission[1], mat.emission[2]) * (light->getSurfaceArea() * ndotl * qAbs(toLight.dot((t->getNormal(i)).normalized() )) / distSquare);
//        }
//    }

//    return intensity;
//}
//Vector3f directLighting(Vector3f p, const Vector3f pn, const tinyobj::material_t& pmat, int pmat_type, Ray p_rin, const Scene &scene){
//    /* Sample direct light rays from emissive triangles loaded in the scene.
//       Args:
//       - p: intersection point where we sample direct lights from
//       - pn: the normal at the intersection point
//       - scene: the scene data
//       Return:
//       - dL: the direct light contribution divided by p(a) = 1/area
//    */

//    Vector3f dL = Vector3f(0,0,0);
//    const tinyobj::real_t *d = pmat.diffuse; //Diffuse color as array of floats
//    Vector3f pd = Vector3f(d[0], d[1], d[2]);
//    std::vector<Triangle *> lights = scene.getEmissiveTriangles();


//    for(int i =0 ;i < lights.size(); ++i){

//        Vector3f pl = lights[i]->getRandomPointWithin();
//        Vector3f rd = (pl - p).normalized();
//        Ray ray2light(p, rd); //the ray from hit point to a sampled point in the light source

//        IntersectionInfo li;
//        if(scene.getIntersection(ray2light, &li)) {
//            //check if shadowed
//            const Triangle *t = static_cast<const Triangle *>(li.data);
//            if(t->getIndex() == lights[i]->getIndex()){

//                //get the parameters needed from this emissive triangle
//                Vector3f e = Vector3f(lights[i]->getMaterial().emission[0], lights[i]->getMaterial().emission[1], lights[i]->getMaterial().emission[2]);

//                const Vector3f ln = lights[i]->getNormal(li); //emissive triangle normal
//                const Affine3f l_invNT = lights[i]->getInverseNormalTransform(); //emissive triangle invNormalTransform matrix
//                const Vector3f world_ln = l_invNT * ln;

//                float cos_p = qMin(1.f, qMax(0.f, pn.dot(ray2light.d))); //cosine term at p
//                float cos_l = qMin(1.f, qMax(0.f, world_ln.dot(-ray2light.d))); //cosine term at the light
//                float dist2 = qPow((pl - p).norm(), 2); //Question: Is this what is meant in the equation;
//                float t_area = lights[i]->getArea();
//                float c = ( cos_p * cos_l )/ dist2;

//                //dL += ( e.array() * c * brdf(pmat, pmat_type, p_rin, ray2light, pn).array() ).matrix() / pa;
//                dL += (e.array() * pd.array()).matrix() * c * t_area; //essentially divided by pdf = 1/area
//                //dL += e * c * t_area; //essentially divided by pdf = 1/area

//            }
//        }
//    }
//    dL = dL / lights.size();

//    return dL / lights.size();

//}

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

        if (!depth)
            L +=Vector3f(emission);

        Vector3f normal = t->getNormal(i);
        normal.normalize(); // surface normal   ,  n
        Vector3f surf_normal = normal.dot(r.d) < 0 ? normal : -normal ; // surface normal fixed for orientation , nl

        // Diffuse
        if (mat.illum == 2)
        {                           // mindepth
            const float pdf_rr = depth < 1 ? 1.0f : qMin(qMax(diffuse[0], qMax(diffuse[1], diffuse[2])), 0.99f);
            Vector3f albedo = Vector3f(diffuse) / EIGEN_PI;
            if (erand48(Xi) < pdf_rr)
            {
                Vector3f wi;
                float pdf;
                cosineSampleHemisphere(normal, wi, pdf, Xi);
                const float illum_scale = wi.dot(normal) / (pdf * pdf_rr);
//                L += directLighting(i.hit, normal, mat, mat.illum, ray, scene) * illum_scale * albedo;
//                L += directLighting(i.hit, normal, scene) * illum_scale * albedo;
                L += traceRay(Ray(i.hit + FLOAT_EPSILON * wi, wi), scene, depth + 1, Xi) * illum_scale;

            }
        }
//        Vector3f L = Vector3f(diffuse[0], diffuse[1], diffuse[2]); // Vec f = obj.c;
//        double p = L[0] > L[1] && L[0] > L[2] ? L[0] : L[1] > L[2] ? L[1] : L[2];

//        // Russian roulette termination.
//        // If random number between 0 and 1 is > p, terminate and return hit object's emmission

//        if (++depth > 5)
//        {
//            if (erand48(Xi) < p * 0.9)
//            { // Multiply by 0.9 to avoid infinite loop with colours of 1.0
//                L = L * (0.9 / p);
//            }
//            else
//            {
//                return Vector3f(emission[0], emission[1], emission[2]);
//            }
//        }

//        // Diffuse
//        if (mat.illum == 2)
//        {
////            double r1 = 2 * EIGEN_PI * erand48(Xi);
////            double r2 = erand48(Xi);
////            double r2s = sqrt(r2);
////            Vector3f u = ( ( qFabs(surf_normal[0]) > 0.1 ? Vector2f(0,1) : VectorXf(1) ) % surf_normal ).normalize();
////            Vector3f v = surf_normal % u;
////            Vector3f d = (u * qCos(r1) * r2s + v * qSin(r1) * r2s + surf_normal * qSqrt(1 - r2)).normalize();

//        }


//        std::cout << "i.hit = " << i.hit << std::endl;

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
