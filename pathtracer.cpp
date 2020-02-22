#include "pathtracer.h"

#include <iostream>
#include <algorithm>

#include <Eigen/Dense>

#include <util/CS123Common.h>

using namespace Eigen;

PathTracer::PathTracer(int width, int height, quint16 num_samples)
    : m_width(width), m_height(height), m_num_samples(num_samples)
{
    m_mindepth = 5;
    m_maxdepth = 10;
}

//////////////////////////////////////////////////
//                 Helpers
//////////////////////////////////////////////////

std::default_random_engine rseed(1);
std::uniform_real_distribution<float> distribution(0, 1);

template <typename T>
T clamp(const T& v, const T& lower, const T& upper) {
    assert( !(upper < lower) );
    return (v < lower) ? lower : (upper < v) ? upper : v;
}

Vector3f vectorTransform(const Affine3f &a, const Vector3f &b, float c)
{
    Vector4f b4d = Vector4f(b[0],b[1],b[2],c);
    Vector4f ab = a.matrix() * b4d;
    return Vector3f(ab[0],ab[1],ab[2]);
}

float PathTracer::brdfLimit(const tinyobj::real_t *material, float thresh)
{
    return qMin(qMax(material[0], qMax(material[1], material[2])), thresh);
}

const float PathTracer::russianRouletteBRDF(uint depth, float low)
{
    return depth < m_mindepth ? 1.0f : low;
}

float PathTracer::random()
{
    return distribution(rseed);
}

void PathTracer::depthOfField(Vector3f &orientation, Vector3f &direction, Vector3f &p, Vector3f &d, float focal, float aperture, unsigned short *ER48SEED)
{
    Vector3f erand3f( 0.5 - erand48(ER48SEED), 0.5 - erand48(ER48SEED), 0.5 - erand48(ER48SEED));
    orientation = p + aperture * erand3f;
    direction = (p + focal * d - orientation);
    direction.normalize();
}

//////////////////////////////////////////////////



void PathTracer::traceScene(QRgb *imageData, const Scene& scene)
{
    std::vector<Vector3f> intensityValues(m_width * m_height);
    Matrix4f invViewMat = (scene.getCamera().getScaleMatrix() * scene.getCamera().getViewMatrix()).inverse();

    #pragma omp parallel for schedule(dynamic, 1)
    for(int y = 0; y < m_height; ++y)
    {
        fprintf(stderr,"\rRendering (%d Samples Per Pixel) %5.2f%%",m_num_samples,100.*y/(m_height-1)); // Taken from smallpt
        for(int x = 0; x < m_width; ++x)
        {
            int offset = x + (y * m_width);
            intensityValues[offset] = tracePixel(x, y, scene, invViewMat);

        }
    }

    toneMap(imageData, intensityValues);
}

Vector3f PathTracer::tracePixel(int x, int y, const Scene& scene, const Matrix4f &invViewMatrix)
{
    Vector3f out(0, 0, 0);
    Vector3f p(0, 0, 0);
    uint depth = 0;
    unsigned short ER48SEED[3]={0,0,y*y*y};
    float antialiasX, antialiasY;

    for(int s = 0; s < m_num_samples; s++)
    {

        antialiasX = x + 0.5 - erand48(ER48SEED);
        antialiasY = y + 0.5 - erand48(ER48SEED);
        // Solve Aliasing problems
        Vector3f d((2.0f * antialiasX / m_width) - 1, 1 - (2.0f * antialiasY / m_height), -1);
        d.normalize();

        Ray r(p, d);
        r = r.transform(invViewMatrix);

        // DEPTH OF FIELD
//        Vector3f orientation;
//        Vector3f direction;
//        depthOfField(orientation, direction, p, d, 4.0,0.5,ER48SEED);
//        r.o = orientation;
//        r.d = direction;
//        r = r.transform(invViewMatrix);



        out += traceRay(r, scene, depth, ER48SEED);
    }

    out = out * (1./m_num_samples);

    return out;
}

void PathTracer::sampleHemisphereImportance(Vector3f &wi, float &pdf, const Vector3f &normal, const tinyobj::material_t& mat, unsigned short *ER48SEED)
{
    double r1 = erand48(ER48SEED);
    double r2 = erand48(ER48SEED);

    float phi = 2.f * EIGEN_PI * r1;
    float theta = qSqrt(r2);

    Vector3f dir = Vector3f(theta * qCos(phi), qSqrt(1.0f - r2), theta * qSin(phi)).normalized();
    Vector3f tangent = qFabs(normal[0]) > qFabs(normal[1]) ? Vector3f(normal[2], 0, -normal[0]) : Vector3f(0, -normal[2], normal[1]).normalized();
    Vector3f per_tangent = normal.cross(tangent);
    wi = Vector3f(dir[0] * per_tangent[0] + dir[1] * normal[0] + dir[2] * tangent[0],
                        dir[0] * per_tangent[1] + dir[1] * normal[1] + dir[2] * tangent[1],
                        dir[0] * per_tangent[2] + dir[1] * normal[2] + dir[2] * tangent[2]).normalized();

    pdf = normal.dot(wi) / EIGEN_PI;
}

Vector3f PathTracer::directLighting(const Scene &scene, const Vector3f& normal, const Vector3f& hit)
{

    Vector3f out(0, 0, 0);
    std::vector<Triangle *> emissiveSurf = scene.getLightSources();
//    std::cout << emissiveSurf.size() << std::endl;
    for(int i =0 ;i < emissiveSurf.size(); ++i)
    {
        Vector3f hit_to_light = (vectorTransform(emissiveSurf[i]->transform, emissiveSurf[i]->sample(), 1.0f) - hit);
        float d_h2l = qPow(hit_to_light[0],2) + qPow(hit_to_light[1],2) + qPow(hit_to_light[2],2); // squared distance between light and i.hit
        hit_to_light.normalize();

        IntersectionInfo li;
        Ray newray(hit + hit_to_light * FLOAT_EPSILON, hit_to_light);

        if(scene.getIntersection(newray, &li))
        {
            const Triangle *t = static_cast<const Triangle *>(li.data);
            if(t->getIndex() == emissiveSurf[i]->getIndex())
            {

                Vector3f emission = Vector3f(emissiveSurf[i]->getMaterial().emission);
                emission = emission * 2.5f; // artificial brighten
                float remove_peaks = hit_to_light.dot(t->getNormal(li));

                if (remove_peaks > 0.0f )
                    continue;

                float abs_normal = clamp(hit_to_light.dot(normal), 0.0f, 1.0f) * qAbs(hit_to_light.dot((t->getNormal(li)).normalized() ));
                const float area = emissiveSurf[i]->getArea();

                out += emission * (area * abs_normal  / d_h2l);

            }

        }
    }

    return out;
}

Vector3f PathTracer::traceRay(const Ray& r, const Scene& scene, uint depth, unsigned short *ER48SEED, bool itsLit)
{
    Vector3f L = Vector3f(0,0,0); // radiance

    IntersectionInfo i;
    Ray ray(r);
    if(scene.getIntersection(ray, &i) && depth < m_maxdepth) {

        const Mesh *m = static_cast<const Mesh *>(i.object);
        const Triangle *t = static_cast<const Triangle *>(i.data);
        const tinyobj::material_t& mat = t->getMaterial();
        //const std::string diffuseTex = mat.diffuse_texname;//Diffuse texture name
        const tinyobj::real_t *diffuse = mat.diffuse;
        const tinyobj::real_t *emission = mat.emission;
        const tinyobj::real_t *specular = mat.specular;



        Vector3f normal = vectorTransform(m->inverseNormalTransform,  t->getNormal(i), 0.0f);
        normal.normalize();
        Vector3f surf_normal = normal.dot(ray.d) < 0 ? normal : -normal ; // surface normal fixed for orientation

        // Enable Caustics
        if (itsLit) if(m->isLight) L +=Vector3f(emission);

        auto pdfbreak = erand48(ER48SEED);

        // DIFFUSE
        if (mat.illum == 2)
        {
            const float pdf_rr = russianRouletteBRDF(depth, brdfLimit(diffuse, 0.99f));

            if (pdfbreak < pdf_rr)
            {
                Vector3f wi;
                float pdf;
                sampleHemisphereImportance(wi, pdf, normal, mat, ER48SEED);

                const float scalle = wi.dot(normal) / (pdf * pdf_rr);
                Ray ray_o(i.hit + FLOAT_EPSILON * wi, wi);

                Vector3f directlight = directLighting(scene, normal, i.hit) * scalle;
                Vector3f indirectlight = traceRay(ray_o, scene, depth + 1, ER48SEED, false) * scalle;

                Vector3f albedo = Vector3f(diffuse) / EIGEN_PI;

                // GLOSSY
                if (Vector3f(specular) != Vector3f(0,0,0))
                {
                    Vector3f refl;
                    float ndotrd = normal.dot(ray.d);

                    if ( ndotrd >= 0.0f )
                        refl = ray.d;
                    else
                        refl = (ray.d - 2.f * normal * ndotrd).normalized();

                    float highlight = qPow(qMax(0.f, qMin(1.f,refl.dot(ray_o.d))), mat.shininess);
                    Vector3f brdf = Vector3f(specular) * (mat.shininess + 2) * highlight / (2 * EIGEN_PI);
                    L += (brdf.array() * directlight.array()).matrix();
                    L += (brdf.array() * indirectlight.array()).matrix();

                }
                else
                {

                    L += (albedo.array() * directlight.array()).matrix();
                    L += (albedo.array() * indirectlight.array()).matrix();
                }



            }
        }

        // REFRACTION
        else if (mat.illum == 7)
        {
            const float pdf_rr = russianRouletteBRDF(depth, 0.95f);

            if (pdfbreak < pdf_rr)
            {
                Vector3f indirectlight(0,0,0);
                float ndotrd = normal.dot(ray.d);

                const float costheta = ray.d.dot(surf_normal);
                const float ratio = ndotrd < 0 ? 1.0f / mat.ior : mat.ior;
                const float cand_radi = 1.0f - qPow(ratio,2) * (1.0f - qPow(costheta,2));
                const Vector3f reflection = (ray.d - 2.f * normal * ndotrd).normalized();

                if (cand_radi < FLOAT_EPSILON)
                {
                  indirectlight = traceRay(Ray(i.hit + FLOAT_EPSILON * reflection, reflection), scene, depth + 1, ER48SEED, true) / pdf_rr;
                }

                else
                {
                    // WITH ATTENUATION : https://www.scratchapixel.com/lessons/3d-basic-rendering/global-illumination-path-tracing
                    Vector3f refraction = ray.d * ratio - surf_normal * (costheta * ratio + qSqrt(cand_radi));

                    const float r_o = qPow((mat.ior - 1.0f),2) / qPow((mat.ior + 1.0f),2);
                    float mix_angle = ndotrd < 0 ? -costheta : refraction.dot(normal);
//                    const float theta_R = r_o + (1.f - r_o) * qPow(1.f - mix_angle, 5);

                    if (erand48(ER48SEED) < (r_o + (1.f - r_o) * qPow(1.f - mix_angle, 5)))
                    {
                       Ray ray_o(i.hit + FLOAT_EPSILON * reflection, reflection);
                      indirectlight = traceRay(ray_o, scene, depth + 1, ER48SEED, true) / pdf_rr;
                    }
                    else
                    {
                        Ray ray_o(i.hit + FLOAT_EPSILON * refraction, refraction);
                      indirectlight = traceRay(ray_o, scene, depth + 1, ER48SEED, true) / pdf_rr;
                    }
                }

                Vector3f albedo = Vector3f(1.0f, 1.0f, 1.0f);
                L += (albedo.array() * indirectlight.array()).matrix();
            }

        }

        // SPECULAR
        else if (mat.illum == 5)
        {
            const float pdf_rr = russianRouletteBRDF(depth, brdfLimit(specular, 0.99f));

            if (pdfbreak < pdf_rr)
            {
              float ndotrd = normal.dot(ray.d);

              Vector3f reflection = (ray.d - 2.f * normal * ndotrd);
              reflection.normalize();

              if ( ndotrd >= 0.0f )
                  reflection = ray.d;

              Ray ray_o(i.hit + FLOAT_EPSILON * reflection, reflection);
              Vector3f indirectlight = traceRay(ray_o, scene, depth + 1, ER48SEED, true) / pdf_rr;

              Vector3f albedo = Vector3f(1.0f, 1.0f, 1.0f);
              L += (albedo.array() * indirectlight.array()).matrix();
            }
        }

        return L;
    }
    else
    {
        return Vector3f(0, 0, 0);
    }
}

int clampIntensity(float v){
    return (int)(255.0f * v / (1.0f + v));
}

void PathTracer::toneMap(QRgb *imageData, std::vector<Vector3f> &intensityValues) {

    for(int y = 0; y < m_height; ++y) {
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);

            imageData[offset] = qRgb(clampIntensity(intensityValues[offset](0)),
                                     clampIntensity(intensityValues[offset](1)),
                                     clampIntensity(intensityValues[offset](2)));

        }
    }

}
