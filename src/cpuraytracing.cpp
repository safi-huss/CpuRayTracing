#ifdef _WIN32
#include "app.h"
#include "cpuraytracing.h"
#else
#define __STDC_LIB_EXT1__
#define _USE_MATH_DEFINES
#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <vector>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <iomanip>
#include <cmath>

#include <float.h>

#define GLM_FORCE_ALIGNED_GENTYPES
#define GLM_EXT_INCLUDED
#define GLM_ENABLE_EXPERIMENTAL
#include "glm]\glm.hpp"
#include "glm/ext.hpp"
#include "glm/gtx/component_wise.hpp"

#include "app.h"
#include "cpuraytracing.h"

#endif

const float LOCAL_EPSILON = 0.00004f;

void CpuRayBoundingBoxIntersect(
    /*[out]*/   std::vector<uint8_t>& arg_outpIntersect,
    /*[in]*/    const std::vector<glm::vec3>& arg_inpRayDirs,
    /*[in]*/    const std::vector<glm::vec3>& arg_inpRayPnts,
    /*[in]*/    const glm::vec3&    arg_inf3AabBox,
    /*[in]*/    const glm::vec3&    arg_inf3ObjectLoc,
    /*[in]*/    const glm::quat&    arg_inf4ObjectOr)
{
    glm::vec3 negAabb = arg_inf3AabBox;

    for (uint32_t idx_ray = 0; idx_ray < arg_inpRayPnts.size(); idx_ray++) {
        glm::vec3 ray_inv_dir = 1.f / arg_inpRayDirs[idx_ray];
        glm::vec3 t0 = (arg_inf3AabBox - arg_inpRayPnts[idx_ray]) * ray_inv_dir;
        glm::vec3 t1 = (-negAabb - arg_inpRayPnts[idx_ray]) * ray_inv_dir;
        glm::vec3 temp = t0;

        t0.x = (t1.x < t0.x) ? t1.x : t0.x;
        t0.y = (t1.y < t0.y) ? t1.y : t0.y;
        t0.z = (t1.z < t0.z) ? t1.z : t0.z;

        t1.x = (temp.x > t1.x) ? temp.x : t1.x;
        t1.y = (temp.y > t1.y) ? temp.y : t1.y;
        t1.z = (temp.z > t1.z) ? temp.z : t1.z;

        float tmin = (isinf(t0.x)) ? t0.y : ((isinf(t0.y)) ? t0.x : fmax(t0.x, t0.y));
        tmin = (isinf(tmin)) ? t0.z : ((isinf(t0.z)) ? tmin : fmax(tmin, t0.z));
        float tmax = (isinf(t1.x)) ? t1.y : ((isinf(t1.y)) ? t1.x : fmin(t1.x, t1.y));
        tmax = (isinf(tmax)) ? t1.z : ((isinf(t1.z)) ? tmax : fmin(tmax, t1.z));

        arg_outpIntersect[idx_ray] = tmax > tmin;
    }
}

void CpuRayTriangleIntersect(
    /*[out]*/   std::vector<uint8_t>& arg_outpIntersect,
    /*[out]*/   std::vector<float>& arg_outpIntersectParam,
    /*[out]*/   std::vector<uint32_t>& arg_outTriangleIntersetIndex,
    /*[in]*/    const std::vector<glm::vec3>& arg_inpRayDirs,
    /*[in]*/    const std::vector<glm::vec3>& arg_inpRayPnts,
    /*[in]*/    const std::vector<glm::vec3>& arg_inpVertexBuffer,
    /*[in]*/    const glm::vec3&    arg_inf3ObjectLoc,
    /*[in]*/    const glm::quat&    arg_inf4ObjectOr)
{
    uint32_t num_tri = static_cast<uint32_t>(arg_inpVertexBuffer.size() / 3);

    for (uint32_t idx_ray = 0; idx_ray < arg_inpRayPnts.size(); idx_ray++) {
        for (uint32_t idx_vert = 0; idx_vert < arg_inpVertexBuffer.size(); idx_vert += 3) {
            uint32_t tri_idx = idx_vert / 3;

            glm::vec3 vertex1 = (arg_inf4ObjectOr * arg_inpVertexBuffer[idx_vert    ]) * glm::inverse(arg_inf4ObjectOr) + arg_inf3ObjectLoc;
            glm::vec3 vertex2 = (arg_inf4ObjectOr * arg_inpVertexBuffer[idx_vert + 1]) * glm::inverse(arg_inf4ObjectOr) + arg_inf3ObjectLoc;
            glm::vec3 vertex3 = (arg_inf4ObjectOr * arg_inpVertexBuffer[idx_vert + 2]) * glm::inverse(arg_inf4ObjectOr) + arg_inf3ObjectLoc;

            glm::vec3 tri_side1 = vertex2 - vertex1;
            glm::vec3 tri_side2 = vertex3 - vertex2;
            glm::vec3 tri_side3 = vertex1 - vertex3;

            glm::vec3 tri_normal = normalize(cross(tri_side1, -tri_side3));
            float plane_d = dot(tri_normal, vertex1);

            float denom = dot(arg_inpRayDirs[idx_ray], tri_normal);
            float numer = plane_d - dot(arg_inpRayPnts[idx_ray], tri_normal);

            float ray_t = numer / denom;
            glm::vec3 ray_plane_int = arg_inpRayPnts[idx_ray] + (arg_inpRayDirs[idx_ray] * ray_t);

            glm::vec3 cross1 = cross(tri_side1, ray_plane_int - vertex1);
            glm::vec3 cross2 = cross(tri_side2, ray_plane_int - vertex2);
            glm::vec3 cross3 = cross(tri_side3, ray_plane_int - vertex3);
            float dot1 = dot(tri_normal, cross1);
            float dot2 = dot(tri_normal, cross2);
            float dot3 = dot(tri_normal, cross3);

            if (idx_ray * num_tri + tri_idx < (arg_inpRayPnts.size() * num_tri)) {
                arg_outpIntersect[idx_ray * num_tri + tri_idx] = dot1 > 0.f && dot2 > 0.f && dot3 > 0.f;
                arg_outpIntersectParam[idx_ray * num_tri + tri_idx] = ray_t;
            }
        }
    }
}

void CpuCastToTextures(
    /*[out]*/   std::vector<uint32_t>& arg_outTexture,
    /*[out]*/   std::vector<uint8_t>& arg_outRaysToReflect,
    /*[in]*/    std::vector<uint8_t>& arg_inpTriIntersect,
    /*[in]*/    std::vector<float>& arg_inpTriIntersectParam,
    /*[in]      const float3* arg_inpRayDirs,*/
    /*[in]*/    const std::vector<glm::vec3>& arg_inpRayPnts,
    /*[in]*/    const std::vector<glm::vec3>& arg_inTex3dPtLookUp,
    /*[in]*/    const std::vector<glm::vec2>& arg_inUvCoordLookUp,
    /*[in]*/    const uint32_t  arg_indTexWidth,
    /*[in]*/    const uint32_t  arg_indTexHeight,
    /*[in]*/    const uint32_t  arg_indNumVert)
{
    uint32_t num_tri = arg_indNumVert / 3;

    for (uint64_t idx_ray = 0; idx_ray < arg_inpRayPnts.size(); idx_ray++) {
        //Calculate index in the Ray-Triangle Matrix
        uint32_t idx_ray_int = idx_ray * num_tri;

        glm::vec2 uv_coord = arg_inUvCoordLookUp[idx_ray];
        glm::vec2 tex_coord(uv_coord.x * (float)arg_indTexWidth, uv_coord.y * (float)arg_indTexHeight);
        uint8_t bCastShadow = 0;

        //index gaurd
        if (idx_ray_int < arg_inpTriIntersect.size()) {

            //Traverse over triangles
            for (uint32_t idx_tri_per_ray = idx_ray_int; idx_tri_per_ray < (idx_ray_int + num_tri); idx_tri_per_ray++) {
                //index gaurd
                if (idx_tri_per_ray < arg_inpTriIntersect.size()) {
                    if (arg_inpTriIntersect[idx_tri_per_ray] != 0) {
                        glm::vec3 cast_point_3d = arg_inTex3dPtLookUp[idx_ray];
                        float cast_pt_param = length(cast_point_3d - arg_inpRayPnts[idx_ray]);

                        if (cast_pt_param > arg_inpTriIntersectParam[idx_tri_per_ray] + LOCAL_EPSILON) {
                            bCastShadow = 1;
                        }
                    }
                }
            }

            if (bCastShadow == 1) {
                //Cast Shadow
                arg_outTexture[(uint32_t)tex_coord.x + ((uint32_t)tex_coord.y * arg_indTexWidth)] = 255;
                arg_outRaysToReflect[idx_ray] = 0;
            }
            else if (bCastShadow == 0) {
                //Light it up
                //Later
                arg_outRaysToReflect[idx_ray] = 1;
            }
        }
    }
}

void Generate3dPointsForUvPoints(
    std::vector<glm::vec3>& arg_vecGenerated,
    std::vector<glm::vec3>& arg_vecNormalsLookUp,
    std::vector<glm::vec2>& arg_vecUvCoordsLookUp,
    const std::vector<glm::vec3>& arg_vecVertices,
    const std::vector<glm::vec3>& arg_vecNormals,
    const std::vector<glm::vec2>& arg_vecUvCoords,
    const glm::vec3& arg_v3ObjectLocation,
    const glm::quat& arg_v4ObjectOrientation,
    const uint32_t& arg_dLvl1TextureWidth,
    const uint32_t& arg_dLvl1TextureHeight)
{
    float fTexWidthStep = 1.f / static_cast<float>(arg_dLvl1TextureWidth);
    float fTexHeightStep = 1.f / static_cast<float>(arg_dLvl1TextureHeight);
    size_t dPointCounter = 0;

    for (auto idx_tri = 0ul; idx_tri != arg_vecVertices.size(); idx_tri += 3) {

        const glm::vec2& v2TriangleUv1 = arg_vecUvCoords[idx_tri + 0];
        const glm::vec2& v2TriangleUv2 = arg_vecUvCoords[idx_tri + 1];
        const glm::vec2& v2TriangleUv3 = arg_vecUvCoords[idx_tri + 2];

        glm::vec2 v2UvMin, v2UvMax;

        v2UvMin.x = std::min(v2TriangleUv1.x, std::min(v2TriangleUv2.x, v2TriangleUv3.x));
        v2UvMin.y = std::min(v2TriangleUv1.y, std::min(v2TriangleUv2.y, v2TriangleUv3.y));

        v2UvMax.x = std::max(v2TriangleUv1.x, std::max(v2TriangleUv2.x, v2TriangleUv3.x));
        v2UvMax.y = std::max(v2TriangleUv1.y, std::max(v2TriangleUv2.y, v2TriangleUv3.y));

        size_t dPointCount = (((v2UvMax.y - v2UvMin.y) / fTexHeightStep) + 1) * (((v2UvMax.x - v2UvMin.x) / fTexWidthStep) + 1);
        size_t dVectorCapacity = arg_vecGenerated.size() + dPointCount;
        //        arg_vecGenerated.reserve(dVectorCapacity);
                //v2UvMin.x = (v2TriangleUv1.x < v2TriangleUv2.x && v2TriangleUv2.x < v2TriangleUv3.x) ? v2TriangleUv1.x 
                //         : ((v2TriangleUv2.x < v2TriangleUv1.x && v2TriangleUv2.x < v2TriangleUv3.x) ? v2TriangleUv2.x : v2TriangleUv3.x);
                //v2UvMin.y = (v2TriangleUv1.y < v2TriangleUv2.y && v2TriangleUv2.y < v2TriangleUv3.y) ? v2TriangleUv1.y
                //         : ((v2TriangleUv2.y < v2TriangleUv1.y && v2TriangleUv2.y < v2TriangleUv3.y) ? v2TriangleUv2.y : v2TriangleUv3.y);

                //v2UvMax.x = (v2TriangleUv1.x > v2TriangleUv2.x && v2TriangleUv2.x > v2TriangleUv3.x) ? v2TriangleUv1.x
                //         : ((v2TriangleUv2.x > v2TriangleUv1.x && v2TriangleUv2.x > v2TriangleUv3.x) ? v2TriangleUv2.x : v2TriangleUv3.x);
                //v2UvMax.y = (v2TriangleUv1.y > v2TriangleUv2.y && v2TriangleUv2.y > v2TriangleUv3.y) ? v2TriangleUv1.y
                //         : ((v2TriangleUv2.y > v2TriangleUv1.y && v2TriangleUv2.y > v2TriangleUv3.y) ? v2TriangleUv2.y : v2TriangleUv3.y);

        for (float iter_uv_y = v2UvMin.y; iter_uv_y <= v2UvMax.y; iter_uv_y += fTexHeightStep) {
            for (float iter_uv_x = v2UvMin.x; iter_uv_x <= v2UvMax.x; iter_uv_x += fTexWidthStep) {
                const glm::vec3 v3TestPoint = { iter_uv_x, iter_uv_y, 0 };

                const glm::vec2 v2TriUvEdge1 = v2TriangleUv2 - v2TriangleUv1;
                const glm::vec2 v2TriUvEdge2 = v2TriangleUv3 - v2TriangleUv2;
                const glm::vec2 v2TriUvEdge3 = v2TriangleUv1 - v2TriangleUv3;

                const glm::vec3 v3TriUvEdge1 = { v2TriUvEdge1.x, v2TriUvEdge1.y, 0 };
                const glm::vec3 v3TriUvEdge2 = { v2TriUvEdge2.x, v2TriUvEdge2.y, 0 };
                const glm::vec3 v3TriUvEdge3 = { v2TriUvEdge3.x, v2TriUvEdge3.y, 0 };

                //Check point is inside the triangle
                const glm::vec3 v3Cross1 = glm::cross(v3TriUvEdge1, v3TestPoint - glm::vec3(v2TriangleUv1, 0));
                const glm::vec3 v3Cross2 = glm::cross(v3TriUvEdge2, v3TestPoint - glm::vec3(v2TriangleUv2, 0));
                const glm::vec3 v3Cross3 = glm::cross(v3TriUvEdge3, v3TestPoint - glm::vec3(v2TriangleUv3, 0));

                const float fDot1 = glm::dot(glm::vec3(0.f, 0.f, -1.f), v3Cross1);
                const float fDot2 = glm::dot(glm::vec3(0.f, 0.f, -1.f), v3Cross2);
                const float fDot3 = glm::dot(glm::vec3(0.f, 0.f, -1.f), v3Cross3);

                if (fDot1 > 0.f && fDot2 > 0.f && fDot3 > 0.f) {
                    //Point is in the triangle, Calculate the 3D point of the UV coordinate
                    const glm::vec3& v3TriVertex1 = ((arg_v4ObjectOrientation * arg_vecVertices[idx_tri + 0]) * glm::inverse(arg_v4ObjectOrientation)) + arg_v3ObjectLocation; //arg_vecVertices[idx_tri + 0];
                    const glm::vec3& v3TriVertex2 = ((arg_v4ObjectOrientation * arg_vecVertices[idx_tri + 1]) * glm::inverse(arg_v4ObjectOrientation)) + arg_v3ObjectLocation; //arg_vecVertices[idx_tri + 1];
                    const glm::vec3& v3TriVertex3 = ((arg_v4ObjectOrientation * arg_vecVertices[idx_tri + 2]) * glm::inverse(arg_v4ObjectOrientation)) + arg_v3ObjectLocation; //arg_vecVertices[idx_tri + 2];

                    const glm::vec3& v3VertNormal1 = arg_vecNormals[idx_tri + 0];
                    const glm::vec3& v3VertNormal2 = arg_vecNormals[idx_tri + 1];
                    const glm::vec3& v3VertNormal3 = arg_vecNormals[idx_tri + 2];

                    const glm::vec3 v3TriUvQVert1 = v3TestPoint - glm::vec3(v2TriangleUv1, 0);
                    const glm::vec3 v3TriUvQVert2 = v3TestPoint - glm::vec3(v2TriangleUv2, 0);
                    const glm::vec3 v3TriUvQVert3 = v3TestPoint - glm::vec3(v2TriangleUv3, 0);

                    float fTriangleArea = glm::length(glm::cross(glm::vec3(v2TriUvEdge1, 0.f), -glm::vec3(v2TriUvEdge3, 0.f)));
                    float fRatio1 = glm::length(glm::cross(v3TriUvQVert2, v3TriUvQVert3)) / fTriangleArea;
                    float fRatio2 = glm::length(glm::cross(v3TriUvQVert3, v3TriUvQVert1)) / fTriangleArea;
                    float fRatio3 = glm::length(glm::cross(v3TriUvQVert1, v3TriUvQVert2)) / fTriangleArea;

                    glm::vec3 v3PointIn3D = v3TriVertex1 * fRatio1 + v3TriVertex2 * fRatio2 + v3TriVertex3 * fRatio3;

                    //TODO: add variance in the normal with tracked uncertainty based on the material roughness coefficient
                    glm::vec3 v3NewNormal = v3VertNormal1 * fRatio1 + v3VertNormal1 * fRatio2 + v3VertNormal3 * fRatio3;

                    arg_vecGenerated.push_back(v3PointIn3D);
                    arg_vecUvCoordsLookUp.push_back(glm::vec2(v3TestPoint.x, v3TestPoint.y));
                    arg_vecNormalsLookUp.push_back(v3NewNormal);
                }
                else {
                    //Handle false case
                }
            }
        }
    }
}

void GenerateRaysFrom3dPoints(
    /*[out]*/ std::vector<glm::vec3>& arg_vecRayNormals,
    /*[out]*/ std::vector<glm::vec3>& arg_vecRayOrigins,
    /*[in]*/  const std::vector<glm::vec3>& arg_vecRayTerminals,
    /*[in]*/  const CpuRayTracing::SLight& arg_objLight)
{
    size_t dNewRayCount = arg_vecRayTerminals.size();

    for (auto idx_ray = 0; idx_ray < dNewRayCount; idx_ray++) {
        glm::vec3 v3RayNormal, v3RayOrigin;

        switch (arg_objLight.m_eLightType)
        {
        case 0://AMBIENT
            break;

        case 1://DIREECTIONAL
            v3RayNormal = arg_objLight.m_v3Direction;
            v3RayOrigin = glm::vec3((fabs(v3RayNormal.x) < std::numeric_limits<float>::epsilon()) ? 0 : ((v3RayNormal.x > std::numeric_limits<float>::epsilon()) ? std::numeric_limits<float>::infinity() : -std::numeric_limits<float>::infinity()),
                (fabs(v3RayNormal.y) < std::numeric_limits<float>::epsilon()) ? 0 : ((v3RayNormal.y > std::numeric_limits<float>::epsilon()) ? std::numeric_limits<float>::infinity() : -std::numeric_limits<float>::infinity()),
                (fabs(v3RayNormal.z) < std::numeric_limits<float>::epsilon()) ? 0 : ((v3RayNormal.z > std::numeric_limits<float>::epsilon()) ? std::numeric_limits<float>::infinity() : -std::numeric_limits<float>::infinity()));
            break;

        case 2://POINT
            v3RayNormal = glm::normalize(arg_vecRayTerminals[idx_ray] - arg_objLight.m_v3Location);
            v3RayOrigin = arg_objLight.m_v3Location;
            break;

        case 3://SPOT
            break;

        default:
            break;
        };

        arg_vecRayNormals.push_back(v3RayNormal);
        arg_vecRayOrigins.push_back(v3RayOrigin);
    }
}

void GenerateReflectedRays(
    /*[out]*/ std::vector<glm::vec3>& arg_vecOutRayDirections,
    /*[out]*/ std::vector<glm::vec3>& arg_vecOutRayStartPts,
    /*[out]*/ std::vector<float>& arg_vecOutRayWeight,
    /*[in]*/ const std::vector<uint8_t>& arg_vecInRaysToReflect,
    /*[in]*/ const std::vector<glm::vec3>& arg_vecInRayDirections,
    /*[in]*/ const std::vector<glm::vec3>& arg_vecInReflectionPts,
    /*[in]*/ const std::vector<glm::vec3>& arg_vecInReflectionNorms,
    /*[in]*/ const std::vector<float>& arg_vecInRayWeight,
    /*[in]*/ const float arg_fMaterialRoughness)
{
    //Treating material roughness as standard-deviation/variance in the surface
    // Material Roughness needs to be between 0 and 1 (inclusive)
    // Num_rays = (1 / (1 - MatRoughnes)) * ScalingFactor
    assert(arg_fMaterialRoughness >= 0.f && arg_fMaterialRoughness <= 1.f);

    uint32_t dNumRays = static_cast<uint32_t>(roundf((1.f / (1.f - arg_fMaterialRoughness)) * 2.f));

    for (auto idx_in_rays = 0; idx_in_rays < arg_vecInRayDirections.size(); idx_in_rays++) {
        if (arg_vecInRaysToReflect[idx_in_rays] != 0) {
            const glm::vec3& v3NewStartPt = arg_vecInReflectionPts[idx_in_rays];
            const glm::vec3& v3ReflectionNormal = arg_vecInReflectionNorms[idx_in_rays];
            const glm::vec3& v3RayIncidenceDir = arg_vecInRayDirections[idx_in_rays];

            glm::vec3 v3NegIncidence = -v3RayIncidenceDir;

            float fAngleFromNormal = acos(glm::dot(v3NegIncidence, v3ReflectionNormal) / (glm::length(v3NegIncidence) * glm::length(v3ReflectionNormal)));

            for (auto idx_gen_rays = 0; idx_gen_rays < dNumRays; idx_gen_rays++) {
                float fNewRayAngle = libUncertainFloatGaussDist(-fAngleFromNormal, arg_fMaterialRoughness * (M_PI_2 - fAngleFromNormal));
                glm::vec3 f3RotationNormal = glm::normalize(glm::cross(v3NegIncidence, v3ReflectionNormal));

                glm::vec3 v3NewDirection = glm::rotate(v3NewDirection, fNewRayAngle, f3RotationNormal);

                arg_vecOutRayDirections.push_back(v3NewDirection);
                arg_vecOutRayStartPts.push_back(v3NewStartPt);
            }

            
            //glm::vec3 v3NewDirection = v3NegIncidence - (2.f * glm::dot(v3NegIncidence, v3ReflectionNormal) * v3ReflectionNormal);
            //glm::reflect()
        }
    }
}

namespace CpuRayTracing
{
    std::shared_ptr<std::vector<glm::vec3>> m_pvecRayStartPts;
    std::shared_ptr<std::vector<glm::vec3>> m_pvecRayNormals;

    std::shared_ptr<std::vector<CpuRayTracing::SLight>> m_pvecLight;

    std::shared_ptr<std::vector<glm::vec3>> m_pvecObjectLocation;
    std::shared_ptr<std::vector<glm::quat>> m_pvecObjectOrientation;
    std::shared_ptr<std::vector<glm::vec3>> m_pvecObjectAabBox;

    std::shared_ptr<std::vector<glm::vec3>> m_pvecObjectVertices;
    std::shared_ptr<std::vector<glm::vec3>> m_pvecObjectVNormals;
    std::shared_ptr<std::vector<glm::vec2>> m_pvecObjectUvCoords;

    std::shared_ptr<std::vector<glm::vec3>> m_pvecObjectGenerated3dTexPts;
    std::shared_ptr<std::vector<glm::vec3>> m_pvecObjectGenerated3dNormals;
    std::shared_ptr<std::vector<glm::vec2>> m_pvecObjectGeneratedUvTexPts;

    std::shared_ptr<std::vector<uint32_t>> m_pvecVertexIndices;
    std::shared_ptr<std::vector<uint32_t>> m_pvecUvCoordIndices;

    std::shared_ptr<std::vector<uint8_t>> m_pvecAabbIntersectMask;
    std::shared_ptr<std::vector<uint8_t>> m_pvecTriIntersectMask;

    std::shared_ptr<std::vector<std::vector<uint32_t>>> m_pvecInTextures;
    std::shared_ptr<std::vector<std::vector<uint32_t>>> m_pvecOutTextures;

    std::shared_ptr<std::vector<std::pair<uint32_t, uint32_t>>> m_pvecInTextureDims;
    std::shared_ptr<std::vector<std::pair<uint32_t, uint32_t>>> m_pvecOutTextureDims;

    std::shared_ptr<std::vector<uint8_t>> m_pvecRaysToReflect;


    void PipelineCreate()
    {
        m_pvecObjectGenerated3dTexPts = std::make_shared<std::vector<glm::vec3>>();
        m_pvecObjectGenerated3dNormals = std::make_shared<std::vector<glm::vec3>>();
        m_pvecObjectGeneratedUvTexPts = std::make_shared<std::vector<glm::vec2>>();

        m_pvecRayNormals = std::make_shared<std::vector<glm::vec3>>();
        m_pvecRayStartPts = std::make_shared<std::vector<glm::vec3>>();

        m_pvecVertexIndices = std::make_shared<std::vector<uint32_t>>();
        m_pvecUvCoordIndices = std::make_shared<std::vector<uint32_t>>();

        m_pvecAabbIntersectMask = std::make_shared<std::vector<uint8_t>>();
        m_pvecTriIntersectMask = std::make_shared<std::vector<uint8_t>>();

        m_pvecRaysToReflect = std::make_shared<std::vector<uint8_t>>();
    }

    void PipelineInit()
    {

    }

    void PipelineRun(uint32_t arg_dState)
    {
        try {
            for (auto iter_light = m_pvecLight->begin(); iter_light != m_pvecLight->end(); iter_light++) {
                auto& objLight = *iter_light;

                auto dObjectCount = m_pvecObjectAabBox->size();
                for (auto idx_obj = 0u; idx_obj < dObjectCount; idx_obj++) {
                    size_t dTriangleCount = m_pvecObjectVertices->size() / 3;
                    auto& vecObjectOutTexture = (*(m_pvecOutTextures.get()))[idx_obj];
                    auto& dimObjectTextureDims = (*(m_pvecOutTextureDims.get()))[idx_obj];

                    if (arg_dState == CPURAYTRACING_LIGHT_CASTING) {
                        //Generate 3D Points for all Uv Coordinates of an Object
                        Generate3dPointsForUvPoints(
                            *(m_pvecObjectGenerated3dTexPts.get()),
                            *(m_pvecObjectGenerated3dNormals.get()),
                            *(m_pvecObjectGeneratedUvTexPts.get()),
                            *(m_pvecObjectVertices.get()),
                            *(m_pvecObjectVNormals.get()),
                            *(m_pvecObjectUvCoords.get()),
                            m_pvecObjectLocation->at(idx_obj),
                            m_pvecObjectOrientation->at(idx_obj),
                            dimObjectTextureDims.first,
                            dimObjectTextureDims.second);

                        //Generate Rays from above 3d points
                        GenerateRaysFrom3dPoints(
                            *(m_pvecRayNormals.get()),
                            *(m_pvecRayStartPts.get()),
                            *(m_pvecObjectGenerated3dTexPts.get()),
                            objLight);

                        //Reflection Data Init
                        m_pvecRaysToReflect->assign(m_pvecRayNormals->size(), 0);
                    }
                    else if (arg_dState == CPURAYTRACING_REFLECT_CASTING) {
                        return; //Just for now
                    }

                    //Calculate Possible Ray Triangle Intersections
                    size_t dRayTriangleInstersectCount = dTriangleCount * m_pvecRayNormals->size();

                    //Now cycle through all the other objects
                    for (auto idx_other_obj = 0u; idx_other_obj < dObjectCount; idx_other_obj++) {
                        //if (idx_other_obj == idx_obj) continue;

                        glm::vec3& f3hAabBox = m_pvecObjectAabBox->at(idx_other_obj);
                        glm::vec3& f3hObjectLoc = m_pvecObjectLocation->at(idx_other_obj);
                        glm::quat& f4hObjectOrt = m_pvecObjectOrientation->at(idx_other_obj);

                        std::vector<uint8_t> vecOutRayBoxIntersect(m_pvecRayNormals->size());
                        std::vector<uint32_t> vecOutRayTriIntersectIndex(m_pvecRayNormals->size());
                        std::vector<uint8_t> vecOutRayTriIntersect(dRayTriangleInstersectCount);
                        std::vector<float> vecOutRayTriIntersectParam(dRayTriangleInstersectCount);

                        CpuRayBoundingBoxIntersect(
                            vecOutRayBoxIntersect, 
                            *(m_pvecRayNormals.get()), 
                            *(m_pvecRayStartPts.get()), 
                            f3hAabBox, 
                            f3hObjectLoc, 
                            f4hObjectOrt);

                        CpuRayTriangleIntersect(
                            vecOutRayTriIntersect,
                            vecOutRayTriIntersectParam,
                            vecOutRayTriIntersectIndex,
                            *(m_pvecRayNormals.get()),
                            *(m_pvecRayStartPts.get()),
                            *(m_pvecObjectVertices.get()),
                            f3hObjectLoc,
                            f4hObjectOrt);

                        CpuCastToTextures(
                            (*(m_pvecOutTextures.get()))[idx_obj],
                            *(m_pvecRaysToReflect.get()),
                            vecOutRayTriIntersect,
                            vecOutRayTriIntersectParam,
                            *(m_pvecRayStartPts.get()),
                            *(m_pvecObjectGenerated3dTexPts.get()),
                            *(m_pvecObjectGeneratedUvTexPts.get()),
                            dimObjectTextureDims.first,
                            dimObjectTextureDims.second,
                            m_pvecObjectVertices->size());
                    }

                    m_pvecObjectGenerated3dTexPts->erase(m_pvecObjectGenerated3dTexPts->begin(), m_pvecObjectGenerated3dTexPts->end());
                    m_pvecObjectGeneratedUvTexPts->erase(m_pvecObjectGeneratedUvTexPts->begin(), m_pvecObjectGeneratedUvTexPts->end());
                    m_pvecRayNormals->erase(m_pvecRayNormals->begin(), m_pvecRayNormals->end());
                    m_pvecRayStartPts->erase(m_pvecRayStartPts->begin(), m_pvecRayStartPts->end());
                }
            }
        }
        catch (std::runtime_error& exError) {
            std::cout << exError.what() << std::endl;
            return;
        }
    }

    std::shared_ptr<std::vector<glm::vec3>> GetRayStartPts()
    {
        return  m_pvecRayStartPts;
    }

    std::shared_ptr<std::vector<glm::vec3>> GetRayNormals()
    {
        return  m_pvecRayNormals;
    }

    std::shared_ptr<std::vector<SLight>> GetLightLocations()
    {
        return m_pvecLight;
    }

    std::shared_ptr<std::vector<glm::vec3>> GetObjectLocations()
    {
        return m_pvecObjectLocation;
    }

    std::shared_ptr<std::vector<glm::quat>> GetObjectOrientations()
    {
        return m_pvecObjectOrientation;
    }

    std::shared_ptr<std::vector<glm::vec3>> GetObjectAabBoxes()
    {
        return m_pvecObjectAabBox;
    }

    std::shared_ptr<std::vector<glm::vec3>> GetObjectVertices()
    {
        return m_pvecObjectVertices;
    }

    std::shared_ptr<std::vector<glm::vec3>> GetObjectNormals()
    {
        return m_pvecObjectVNormals;
    }

    std::shared_ptr<std::vector<glm::vec2>> GetObjectUvCoords()
    {
        return  m_pvecObjectUvCoords;
    }

    std::shared_ptr<std::vector<glm::vec3>> GetObjectGenerated3dTexPts()
    {
        return m_pvecObjectGenerated3dTexPts;
    }

    std::shared_ptr<std::vector<glm::vec2>> GetObjectGeneratedUvTexPts()
    {
        return  m_pvecObjectGeneratedUvTexPts;
    }

    std::shared_ptr<std::vector<uint32_t>> GetVertexIndices()
    {
        return m_pvecVertexIndices;
    }

    std::shared_ptr<std::vector<uint32_t>> GetUvCoordIndices()
    {
        return  m_pvecUvCoordIndices;
    }

    std::shared_ptr<std::vector<std::vector<uint32_t>>> GetInTextures()
    {
        return m_pvecInTextures;
    }

    std::shared_ptr<std::vector<std::vector<uint32_t>>> GetOutTextures()
    {
        return  m_pvecOutTextures;
    }

    std::shared_ptr<std::vector<std::pair<uint32_t, uint32_t>>> GetInTextureDims()
    {
        return m_pvecInTextureDims;
    }

    std::shared_ptr<std::vector<std::pair<uint32_t, uint32_t>>> GetOutTextureDims()
    {
        return m_pvecOutTextureDims;
    }

    void SetRayStartPts(std::shared_ptr<std::vector<glm::vec3>> arg_pvecRayStartPts)
    {
        m_pvecRayStartPts = arg_pvecRayStartPts;
    }

    void SetRayNormals(std::shared_ptr<std::vector<glm::vec3>> arg_pvecRayNormals)
    {
        m_pvecRayNormals = arg_pvecRayNormals;
    }

    void SetLights(std::shared_ptr<std::vector<SLight>> arg_pvecLights)
    {
        m_pvecLight = arg_pvecLights;
    }

    void SetObjectLocations(std::shared_ptr<std::vector<glm::vec3>> arg_pvecObjectLocations)
    {
        m_pvecObjectLocation = arg_pvecObjectLocations;
    }

    void SetObjectOrientations(std::shared_ptr<std::vector<glm::quat>> arg_pvecObjectOrientations)
    {
        m_pvecObjectOrientation = arg_pvecObjectOrientations;
    }

    void SetObjectAabBoxes(std::shared_ptr<std::vector<glm::vec3>> arg_pvecObjectAabBoxes)
    {
        m_pvecObjectAabBox = arg_pvecObjectAabBoxes;
    }

    void SetObjectVertices(std::shared_ptr<std::vector<glm::vec3>> arg_pvecObjectVertices)
    {
        m_pvecObjectVertices = arg_pvecObjectVertices;
    }

    void SetObjectNormals(std::shared_ptr<std::vector<glm::vec3>> arg_pvecObjectNormals)
    {
        m_pvecObjectVNormals = arg_pvecObjectNormals;
    }

    void SetObjectUvCoords(std::shared_ptr<std::vector<glm::vec2>> arg_pvecObjectUvCoords)
    {
        m_pvecObjectUvCoords = arg_pvecObjectUvCoords;
    }

    void SetInTextures(std::shared_ptr<std::vector<std::vector<uint32_t>>> arg_pvecInTextures)
    {
        m_pvecInTextures = arg_pvecInTextures;
    }

    void SetOutTextures(std::shared_ptr<std::vector<std::vector<uint32_t>>> arg_pvecOutTextures)
    {
        m_pvecOutTextures = arg_pvecOutTextures;
    }

    void SetInTextureDims(std::shared_ptr<std::vector<std::pair<uint32_t, uint32_t>>> arg_pvecInTextureDims)
    {
        m_pvecInTextureDims = arg_pvecInTextureDims;
    }

    void SetOutTextureDims(std::shared_ptr<std::vector<std::pair<uint32_t, uint32_t>>> arg_pvecInTextureDims)
    {
        m_pvecOutTextureDims = arg_pvecInTextureDims;
    }
};