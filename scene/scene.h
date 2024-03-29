#ifndef SCENE_H
#define SCENE_H

#include <QString>

#include "BVH/BVH.h"

#include "basiccamera.h"

#include "util/CS123SceneData.h"

#include "shape/mesh.h"

#include <memory>

class Scene
{
public:
    Scene();
    virtual ~Scene();

    static bool load(QString filename, Scene **scenePointer);

    void setBVH(const BVH &bvh);
    const BVH& getBVH() const;
    const std::vector<Triangle *>& getLightSources() const;

    const BasicCamera& getCamera() const;

    void setCamera(const BasicCamera& camera);
    void setGlobalData(const CS123SceneGlobalData& data);
    void addLight(const CS123SceneLightData& data);
    const std::vector<CS123SceneLightData>& getLights();

    bool getIntersection(const Ray& ray, IntersectionInfo* I) const;

private:

    BVH *m_bvh;
    std::vector<Triangle *> m_lightsources;
    std::vector<Object *> *_objects;
    BasicCamera m_camera;
    CS123SceneGlobalData m_globalData;
    std::vector<CS123SceneLightData> m_lights;

    static bool parseTree(CS123SceneNode *root, Scene *scene, const std::string& baseDir);
    static void parseNode(CS123SceneNode *node, const Eigen::Affine3f &parentTransform, std::vector<Object *> *objects, const std::string& baseDir);
    static void addPrimitive(CS123ScenePrimitive *prim, const Eigen::Affine3f &transform, std::vector<Object *> *objects, const std::string& baseDir);
    static Mesh *loadMesh(std::string filePath, const Eigen::Affine3f &transform, const std::string& baseDir);
};

#endif // SCENE_H
