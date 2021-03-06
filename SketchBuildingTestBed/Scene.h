#pragma once

#include <vector>
#include "CGA.h"
#include "GLUtils.h"
#include "FaceSelector.h"
#include "BuildingSelector.h"

class RenderManager;

namespace sc {

class SceneObject {
public:
	Scene* scene;

	float offset_x;
	float offset_y;
	float offset_z;
	float object_width;
	float object_depth;
	float height;
	
	std::map<std::string, cga::Grammar> grammars;
	std::vector<boost::shared_ptr<glutils::Face> > faces;

public:
	SceneObject(Scene* scene);
	SceneObject(Scene* scene, float offset_x, float offset_y, float offset_width, float offset_depth, float height, const cga::Grammar& grammar);
	void setFootprint(float offset_x, float offset_y, float offset_z, float object_width, float object_depth);
	void setHeight(float height);
	void setGrammar(const std::string& name, const cga::Grammar& grammar);
	void setGrammar(const std::string& name, const cga::Grammar& grammar, const std::vector<float>& params, bool normalized);

	void generateGeometry(cga::CGA* system, RenderManager* renderManager, const std::string& stage);
	void updateGeometry(RenderManager* renderManager, const std::string& stage);
};

class Scene {
public:
	cga::CGA system;
	std::vector<SceneObject> _objects;
	int _currentObject;

	std::vector<std::vector<SceneObject> > _history;

	FaceSelector* faceSelector;
	BuildingSelector* buildingSelector;

	std::string default_grammar_file;
	std::map<std::string, cga::Grammar> default_grammars;

public:
	Scene();

	void clear();
	void clearCurrentObject();
	void newObject();
	void removeObject(int objectId);
	void updateHistory();
	void undo();
	void alignObjects(float threshold);
	void alignObjects(int currentObject, int controlPoint, float threshold);
	void alignObjects(const glutils::Face& baseFace, float threshold);
	SceneObject& currentObject() { return _objects[_currentObject]; }

	std::pair<int, boost::shared_ptr<glutils::Face> > findFace(const std::vector<glm::vec2>& lasso, const glm::mat4& mvpMatrix, const glm::vec3& camera_view, int screen_width, int screen_height);

	void generateGeometry(RenderManager* renderManager, const std::string& stage);
	void generateGeometry(RenderManager* renderManager, const std::string& stage, int currentObject);
	void updateGeometry(RenderManager* renderManager, const std::string& stage);

	void saveGeometry(const std::string& filename);

	void loadDefaultGrammar(const std::string& default_grammar_file);
	void setDefaultGrammar(const std::string& name, const cga::Grammar& grammar);
};

}
