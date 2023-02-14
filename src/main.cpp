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

//Fix for M_PI_2
#define M_PI_2     1.57079632679489661923   // pi/2

#include <float.h>

#define GLM_FORCE_CXX98
#define GLM_FORCE_ALIGNED_GENTYPES
#define GLM_EXT_INCLUDED
#define GLM_ENABLE_EXPERIMENTAL
#include "../glm/glm.hpp"
#include "../glm/ext.hpp"
#include "../glm/gtx/component_wise.hpp"

#include "../include/cpuraytracing.h"

#endif

const uint32_t LEVEL_0_TEXTURE_SIDE_LENGTH = 128;

void WriteTexture(std::string arg_strFilename, std::vector<uint32_t> arg_vecTexture, uint32_t arg_dTextureWidth, uint32_t arg_dTextureHeight)
{
    assert(arg_vecTexture.size() == static_cast<size_t>(arg_dTextureWidth) * static_cast<size_t>(arg_dTextureHeight));

    std::ofstream fileOut(arg_strFilename);

    for (uint32_t idx_y = 0; idx_y < arg_dTextureHeight; idx_y++) {
        for (uint32_t idx_x = 0; idx_x < arg_dTextureWidth; idx_x++) {
            uint32_t pixel = arg_vecTexture[idx_x + idx_y * arg_dTextureWidth];

            if ((pixel & 0xFF) == 0) fileOut << 'X';
            else if ((pixel & 0xFF) && ((pixel & 0xFFFFFF00) == 0)) fileOut << '0';
            else if (pixel == 0xFFFFFFFF) fileOut << '1';
        }

        fileOut << std::endl;
    }

    fileOut.close();
}

int main()
{
    //Initializing Scene Data
    std::shared_ptr<std::vector<glm::vec3>> pvecVertices = std::make_shared<std::vector<glm::vec3>>();
    std::shared_ptr<std::vector<glm::vec3>> pvecNormVecs = std::make_shared<std::vector<glm::vec3>>();
    std::shared_ptr<std::vector<glm::vec2>> pvecUvCoords = std::make_shared<std::vector<glm::vec2>>();

    std::shared_ptr<std::vector<glm::vec3>> pvecObjAabbs = std::make_shared<std::vector<glm::vec3>>();
    std::shared_ptr<std::vector<glm::vec3>> pvecObjLocns = std::make_shared<std::vector<glm::vec3>>();
    std::shared_ptr<std::vector<glm::quat>> pvecObjOrnts = std::make_shared<std::vector<glm::quat>>();

    std::shared_ptr<std::vector<CpuRayTracing::SLight>> pvecLights = std::make_shared<std::vector<CpuRayTracing::SLight>>();

    std::shared_ptr<std::vector<std::vector<uint32_t>>> pvecOutTextures = std::make_shared<std::vector<std::vector<uint32_t>>>();
    std::shared_ptr<std::vector<std::pair<uint32_t, uint32_t>>> pvecOutTextureDims = std::make_shared<std::vector<std::pair<uint32_t, uint32_t>>>();

    glm::vec3 v3AabBox(1.f, 1.f, 1.f);
    CpuRayTracing::SLight Light = {
        .m_dLightId = 0,
        .m_eLightType = 2,
        .m_v3Color = glm::vec3(255.f, 255.f, 255.f),
        .m_v3Direction = glm::normalize(glm::vec3(0.f, 0.f, 1.f)),
        .m_v3Location = glm::vec3(0.f, 0.f, -1000.f),
        .m_fInnerRadialAngle = 0.f,
        .m_fOuterRadialAngle = 0.f,
        .m_fAttenuation = 0.f
    };

    glm::vec3 pVertices[] = {
        glm::vec3(1.f,  1.f,  1.f),
        glm::vec3(-1.f,  1.f,  1.f),
        glm::vec3(1.f, -1.f,  1.f),
        glm::vec3(-1.f, -1.f,  1.f),
        glm::vec3(1.f,  1.f, -1.f),
        glm::vec3(-1.f,  1.f, -1.f),
        glm::vec3(1.f, -1.f, -1.f),
        glm::vec3(-1.f, -1.f, -1.f)
    };

    //glm::vec2 pUvCoords[] = {
    //    glm::vec2(0.f, 0.f),
    //    glm::vec2(0.f, 1.f),
    //    glm::vec2(1.f, 0.f),
    //    glm::vec2(1.f, 1.f)
    //};

    glm::vec2 pUvCoords[] = {
        glm::vec2(0.333f,   0.f),
        glm::vec2(0.666f,   0.f),
        glm::vec2(0.333f, 0.25f),
        glm::vec2(0.666f, 0.25f),
        glm::vec2(0.333f,  0.5f),
        glm::vec2(0.666f,  0.5f),
        glm::vec2(0.333f, 0.75f),
        glm::vec2(0.666f, 0.75f),
        glm::vec2(0.333f,   1.f),
        glm::vec2(0.666f,   1.f),
        glm::vec2(0.f,    0.25f),
        glm::vec2(1.f,    0.25f),
        glm::vec2(0.f,     0.5f),
        glm::vec2(1.f,     0.5f)
    };

    glm::vec3 pNormals[] = {
        glm::vec3(-1.f,  0.f,  0.f),
        glm::vec3(1.f,  0.f,  0.f),
        glm::vec3(0.f, -1.f,  0.f),
        glm::vec3(0.f,  1.f,  0.f),
        glm::vec3(0.f,  0.f, -1.f),
        glm::vec3(0.f,  0.f,  1.f)
    };

    uint32_t pVertIndices[] = {
        0, 1, 2,
        2, 1, 3,
        0, 2, 4,
        4, 2, 6,
        0, 4, 1,
        1, 4, 5,
        1, 5, 3,
        3, 5, 7,
        2, 3, 6,
        6, 3, 7,
        4, 6, 5,
        5, 6, 7
    };

    uint32_t pUvCoordIndices[] = {
         1,  0,  3,
         3,  0,  2,
        11,  3, 13,
        13,  3,  5,
         9,  7,  8,
         8,  7,  6,
        10, 12,  2,
         2, 12,  4,
         3,  2,  5,
         5,  2,  4,
         6,  7,  4,
         4,  7,  5
    };

    uint32_t pNormalIndices[] = {
        5, 5, 5,
        5, 5, 5,
        1, 1, 1,
        1, 1, 1,
        3, 3, 3,
        3, 3, 3,
        0, 0, 0,
        0, 0, 0,
        2, 2, 2,
        2, 2, 2,
        4, 4, 4,
        4, 4, 4
    };

    std::vector<uint32_t> imageOutputTexture1(LEVEL_0_TEXTURE_SIDE_LENGTH * LEVEL_0_TEXTURE_SIDE_LENGTH);
    std::vector<uint32_t> imageOutputTexture2(LEVEL_0_TEXTURE_SIDE_LENGTH * LEVEL_0_TEXTURE_SIDE_LENGTH);
    std::pair<uint32_t, uint32_t> dimOutputTexture1(LEVEL_0_TEXTURE_SIDE_LENGTH, LEVEL_0_TEXTURE_SIDE_LENGTH);
    std::pair<uint32_t, uint32_t> dimOutputTexture2(LEVEL_0_TEXTURE_SIDE_LENGTH, LEVEL_0_TEXTURE_SIDE_LENGTH);

    for (auto idx_tri = 0; idx_tri < 12; idx_tri++) {
        glm::vec3 pTriangle[3] = {
            pVertices[pVertIndices[idx_tri * 3 + 0]],
            pVertices[pVertIndices[idx_tri * 3 + 1]],
            pVertices[pVertIndices[idx_tri * 3 + 2]]
        };

        glm::vec3 pTriNorms[3] = {
            pNormals[pNormalIndices[idx_tri * 3 + 0]],
            pNormals[pNormalIndices[idx_tri * 3 + 1]],
            pNormals[pNormalIndices[idx_tri * 3 + 2]]
        };

        glm::vec2 pUv[3] = {
            pUvCoords[pUvCoordIndices[idx_tri * 3 + 0]],
            pUvCoords[pUvCoordIndices[idx_tri * 3 + 1]],
            pUvCoords[pUvCoordIndices[idx_tri * 3 + 2]]
        };

        pvecVertices->push_back(pTriangle[0]);
        pvecVertices->push_back(pTriangle[1]);
        pvecVertices->push_back(pTriangle[2]);

        pvecNormVecs->push_back(pTriNorms[0]);
        pvecNormVecs->push_back(pTriNorms[1]);
        pvecNormVecs->push_back(pTriNorms[2]);

        pvecUvCoords->push_back(pUv[0]);
        pvecUvCoords->push_back(pUv[1]);
        pvecUvCoords->push_back(pUv[2]);
    }

    pvecLights->push_back(Light);

    //Object 0
    pvecObjAabbs->push_back(v3AabBox);
    pvecObjLocns->push_back(glm::vec3(0.f, 0.f, 0.f));
    pvecObjOrnts->push_back(glm::quat(1.f, 0.f, 0.f, 0.f));

    //Object 1
    pvecObjAabbs->push_back(v3AabBox);
    pvecObjLocns->push_back(glm::vec3(0.5f, 0.5f, -10.f));
    pvecObjOrnts->push_back(glm::quat(1.f, 0.f, 0.f, 0.f));

    pvecOutTextures->push_back(imageOutputTexture1);
    pvecOutTextures->push_back(imageOutputTexture2);
    pvecOutTextureDims->push_back(dimOutputTexture1);
    pvecOutTextureDims->push_back(dimOutputTexture2);

    CpuRayTracing::SetLights(pvecLights);
    CpuRayTracing::SetObjectAabBoxes(pvecObjAabbs);
    CpuRayTracing::SetObjectLocations(pvecObjLocns);
    CpuRayTracing::SetObjectOrientations(pvecObjOrnts);

    CpuRayTracing::SetObjectVertices(pvecVertices);
    CpuRayTracing::SetObjectUvCoords(pvecUvCoords);
    CpuRayTracing::SetObjectNormals(pvecNormVecs);

    CpuRayTracing::SetOutTextures(pvecOutTextures);
    CpuRayTracing::SetOutTextureDims(pvecOutTextureDims);

    CpuRayTracing::PipelineCreate();
    CpuRayTracing::PipelineInit();

    std::chrono::high_resolution_clock::time_point timeStartProcess = std::chrono::high_resolution_clock::now();

    CpuRayTracing::PipelineRun(0);

    std::chrono::high_resolution_clock::time_point timeEndProcess = std::chrono::high_resolution_clock::now();

    std::cout << "Time taken by Algorithm\t= " << std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(timeEndProcess - timeStartProcess).count() << "ms" << std::endl;

    //Write Textures
    WriteTexture("Texture0.txt", pvecOutTextures->at(0), LEVEL_0_TEXTURE_SIDE_LENGTH, LEVEL_0_TEXTURE_SIDE_LENGTH);
    WriteTexture("Texture1.txt", pvecOutTextures->at(1), LEVEL_0_TEXTURE_SIDE_LENGTH, LEVEL_0_TEXTURE_SIDE_LENGTH);
}
