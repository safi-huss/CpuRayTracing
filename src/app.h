#pragma once

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

#ifdef _WIN32
#define GLM_FORCE_ALIGNED_GENTYPES
#define GLM_EXT_INCLUDED
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <glm/gtx/component_wise.hpp>
#else
#define GLM_FORCE_CXX98
#define GLM_FORCE_ALIGNED_GENTYPES
#define GLM_EXT_INCLUDED
#define GLM_ENABLE_EXPERIMENTAL
#include "../glm/glm/glm.hpp"
#include "../glm/glm/ext.hpp"
#include "../glm/glm/gtx/component_wise.hpp"
#endif