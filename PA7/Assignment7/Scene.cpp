//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    // Randomly sample an light source object
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    if (depth > maxDepth)
    {
        return Vector3f(0.0f, 0.0f, 0.0f);
    }
    Intersection inter_shading_point = intersect(ray);
    Vector3f hitColor(0.0f);
    // Shading point is on a light source
    if (inter_shading_point.emit.norm() > 0.0f)
    {
        hitColor = Vector3f(1.0f);
    }
    // Shading point is on a common object
    else if (inter_shading_point.happened)
    {
        Vector3f p = inter_shading_point.coords; // Position of shading point
        Vector3f N = inter_shading_point.normal; // Normal of shading point
        Vector3f wo = -ray.direction; // Direction of emergent light
        Material* m = inter_shading_point.m;

        Intersection inter_light;
        float pdf_light = 0.0f;
        sampleLight(inter_light, pdf_light);

        Vector3f x_prime = inter_light.coords; // Position of sampling point
        Vector3f N_prime = inter_light.normal;
        Vector3f ws = normalize(x_prime - p);
        Vector3f l_dir(0.0f), l_indir(0.0f);

        // Shoot a ray from p to x'
        Intersection inter_p_to_xprime = intersect(Ray(p, ws));
        // If the ray is not blocked in the middle
        // Direct illumination
        if ((x_prime - inter_p_to_xprime.coords).norm() < 0.01f)
        {
            float pdf_light_inv = 1 / pdf_light;
            float dist_inv = 1 / dotProduct(x_prime - p, x_prime - p);
            l_dir = inter_light.emit * m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, N_prime) * dist_inv * pdf_light_inv;
        }
        
        // Indirect illumination
        float ksi = get_random_float();
        if (ksi < RussianRoulette)
        {
            Vector3f wi = m->sample(wo, N); // Sample indirect incident light
            float pdf_inv = 1 / m->pdf(wi, wo, N);
            float RR_inv = 1 / RussianRoulette;
            l_indir = castRay(Ray(p, wi), depth) * m->eval(wi, wo, N) * dotProduct(wi, N) * pdf_inv * RR_inv;
        }
        hitColor = l_dir + l_indir;
    }

    return hitColor;
}