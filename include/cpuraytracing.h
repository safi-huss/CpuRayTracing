#pragma once

namespace CpuRayTracing
{
	const uint32_t CPURAYTRACING_LIGHT_CASTING = 0;
	const uint32_t CPURAYTRACING_REFLECT_CASTING = 1;

	struct SLight
	{
		uint32_t m_dLightId;
		uint32_t m_eLightType;
		glm::vec3 m_v3Color;
		glm::vec3 m_v3Direction;
		glm::vec3 m_v3Location;
		float m_fInnerRadialAngle;
		float m_fOuterRadialAngle;
		float m_fAttenuation;
	};

	void PipelineCreate();
	void PipelineInit();
	void PipelineRun(uint32_t arg_dState);

	std::shared_ptr<std::vector<glm::vec3>> GetRayStartPts();
	std::shared_ptr<std::vector<glm::vec3>> GetRayNormals();
	std::shared_ptr<std::vector<SLight>> GetLightLocations();
	std::shared_ptr<std::vector<glm::vec3>> GetObjectLocations();
	std::shared_ptr<std::vector<glm::quat>> GetObjectOrientations();
	std::shared_ptr<std::vector<glm::vec3>> GetObjectAabBoxes();
	std::shared_ptr<std::vector<glm::vec3>> GetObjectVertices();
	std::shared_ptr<std::vector<glm::vec3>> GetObjectNormals();
	std::shared_ptr<std::vector<glm::vec2>> GetObjectUvCoords();
	std::shared_ptr<std::vector<glm::vec3>> GetObjectGenerated3dTexPts();
	std::shared_ptr<std::vector<glm::vec2>> GetObjectGeneratedUvTexPts();
	std::shared_ptr<std::vector<uint32_t>> GetVertexIndices();
	std::shared_ptr<std::vector<uint32_t>> GetUvCoordIndices();
	
	std::shared_ptr<std::vector<std::vector<uint32_t>>> GetInTextures();
	std::shared_ptr<std::vector<std::vector<uint32_t>>> GetOutTextures();
	std::shared_ptr<std::vector<std::pair<uint32_t, uint32_t>>> GetInTextureDims();
	std::shared_ptr<std::vector<std::pair<uint32_t, uint32_t>>> GetOutTextureDims();

	void SetRayStartPts(std::shared_ptr<std::vector<glm::vec3>> arg_pvecRayStartPts);
	void SetRayNormals(std::shared_ptr<std::vector<glm::vec3>> arg_pvecRayNormals);
	void SetLights(std::shared_ptr<std::vector<SLight>> arg_pvecLightLocations);
	void SetObjectLocations(std::shared_ptr<std::vector<glm::vec3>> arg_pvecObjectLocations);
	void SetObjectOrientations(std::shared_ptr<std::vector<glm::quat>> arg_pvecObjectOrientations);
	void SetObjectAabBoxes(std::shared_ptr<std::vector<glm::vec3>> arg_pvecObjectAabBoxes);
	void SetObjectVertices(std::shared_ptr<std::vector<glm::vec3>> arg_pvecObjectVertices);
	void SetObjectNormals(std::shared_ptr<std::vector<glm::vec3>> arg_pvecObjectNormals);
	void SetObjectUvCoords(std::shared_ptr<std::vector<glm::vec2>> arg_pvecObjectUvCoords);

	void SetInTextures(std::shared_ptr<std::vector<std::vector<uint32_t>>> arg_pvecInTextures);
	void SetOutTextures(std::shared_ptr<std::vector<std::vector<uint32_t>>> arg_pvecOutTextures);
	void SetInTextureDims(std::shared_ptr<std::vector<std::pair<uint32_t, uint32_t>>> arg_pvecInTextureDims);
	void SetOutTextureDims(std::shared_ptr<std::vector<std::pair<uint32_t, uint32_t>>> arg_pvecInTextureDims);
};