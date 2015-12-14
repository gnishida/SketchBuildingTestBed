﻿#include <iostream>
#include "GLWidget3D.h"
#include "MainWindow.h"
#include <GL/GLU.h>
#include <QDir>
#include <QFileInfoList>
#include "OBJLoader.h"
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>
#include "GrammarParser.h"
#include "Rectangle.h"
#include "GLUtils.h"
//#include "Regression.h"
#include <time.h>
#include "LeftWindowItemWidget.h"
#include "Scene.h"
#include "LayoutExtractor.h"

#ifndef M_PI
#define M_PI	3.141592653
#endif

GLWidget3D::GLWidget3D(QWidget *parent) : QGLWidget(QGLFormat(QGL::SampleBuffers), parent) {
	mainWin = (MainWindow*)parent;
	dragging = false;
	ctrlPressed = false;
	demo_mode = 0;

	// This is necessary to prevent the screen overdrawn by OpenGL
	setAutoFillBackground(false);

	// 光源位置をセット
	// ShadowMappingは平行光源を使っている。この位置から原点方向を平行光源の方向とする。
	light_dir = glm::normalize(glm::vec3(-4, -5, -8));
	//light_dir = glm::normalize(glm::vec3(-1, -3, -2));

	// シャドウマップ用のmodel/view/projection行列を作成
	glm::mat4 light_pMatrix = glm::ortho<float>(-100, 100, -100, 100, 0.1, 200);
	glm::mat4 light_mvMatrix = glm::lookAt(-light_dir * 50.0f, glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
	light_mvpMatrix = light_pMatrix * light_mvMatrix;

	std::string stage_names[6] = { "building", "roof", "facade", "floor", "window", "ledge" };

	// load grammars and their thumbnail images
	for (int i = 0; i < 6; ++i) {
		{
			QStringList filters;
			filters << "*.xml";
			QFileInfoList fileInfoList = QDir(std::string("cga/" + stage_names[i] + "/").c_str()).entryInfoList(filters, QDir::Files | QDir::NoDotAndDotDot);
			for (auto fileInfo : fileInfoList) {
				cga::Grammar grammar;
				try {
					std::cout << "Load gramar: " << fileInfo.absoluteFilePath().toUtf8().constData() << std::endl;
					cga::parseGrammar(fileInfo.absoluteFilePath().toUtf8().constData(), grammar);
					grammars[stage_names[i]].push_back(grammar);
				}
				catch (const std::string& ex) {
					std::cout << "ERROR:" << std::endl << ex << std::endl;
				}
				catch (const char* ex) {
					std::cout << "ERROR:" << std::endl << ex << std::endl;
				}
			}
		}

		// load thumbnail for each grammar
		{
			QStringList filters;
			filters << "*.png" << "*.jpg" << "*.bmp";
			QFileInfoList fileInfoList = QDir(std::string("cga/" + stage_names[i] + "/").c_str()).entryInfoList(filters, QDir::Files | QDir::NoDotAndDotDot);
			for (auto fileInfo : fileInfoList) {
				grammarImages[stage_names[i]].push_back(QImage(fileInfo.absoluteFilePath()).scaled(LeftWindowItemWidget::IMAGE_WIDTH, LeftWindowItemWidget::IMAGE_HEIGHT));
			}
		}
	}

	// initialize deep learning network
	/*
	regressions.resize(2);
	regressions[0] = new Regression("../models/cuboid_43/deploy.prototxt", "../models/cuboid_43/train_iter_64000.caffemodel");
	regressions[1] = new Regression("../models/lshape_44/deploy.prototxt", "../models/lshape_44/train_iter_64000.caffemodel");
	*/





	// initialize stylized polylines
	style_polylines.resize(10);
	style_polylines[0].push_back(glm::vec2(-0.025, -0.025));
	style_polylines[0].push_back(glm::vec2(0.3, 0.035));
	style_polylines[0].push_back(glm::vec2(0.6, 0.05));
	style_polylines[0].push_back(glm::vec2(0.85, 0.04));
	style_polylines[0].push_back(glm::vec2(1.01, 0.02));

	style_polylines[1].push_back(glm::vec2(-0.01, 0.01));
	style_polylines[1].push_back(glm::vec2(0.13, -0.01));
	style_polylines[1].push_back(glm::vec2(0.27, -0.02));
	style_polylines[1].push_back(glm::vec2(0.7, -0.02));
	style_polylines[1].push_back(glm::vec2(0.81, 0));
	style_polylines[1].push_back(glm::vec2(1.02, 0));

	style_polylines[2].push_back(glm::vec2(-0.02, 0.0));
	style_polylines[2].push_back(glm::vec2(0.12, 0.01));
	style_polylines[2].push_back(glm::vec2(0.37, 0.02));
	style_polylines[2].push_back(glm::vec2(0.6, 0.02));
	style_polylines[2].push_back(glm::vec2(0.77, 0.01));
	style_polylines[2].push_back(glm::vec2(0.91, 0.005));
	style_polylines[2].push_back(glm::vec2(0.99, -0.01));

	style_polylines[3].push_back(glm::vec2(-0.02, 0.0));
	style_polylines[3].push_back(glm::vec2(0.57, -0.01));
	style_polylines[3].push_back(glm::vec2(0.8, -0.01));
	style_polylines[3].push_back(glm::vec2(1.01, 0.01));

	style_polylines[4].push_back(glm::vec2(-0.01, 0.0));
	style_polylines[4].push_back(glm::vec2(0.13, -0.01));
	style_polylines[4].push_back(glm::vec2(0.23, -0.02));
	style_polylines[4].push_back(glm::vec2(0.31, -0.02));
	style_polylines[4].push_back(glm::vec2(0.38, -0.01));
	style_polylines[4].push_back(glm::vec2(0.46, 0.0));
	style_polylines[4].push_back(glm::vec2(0.61, 0.02));
	style_polylines[4].push_back(glm::vec2(0.68, 0.03));
	style_polylines[4].push_back(glm::vec2(0.8, 0.03));
	style_polylines[4].push_back(glm::vec2(0.88, 0.02));
	style_polylines[4].push_back(glm::vec2(0.97, 0.01));

	style_polylines[5].push_back(glm::vec2(0.05, -0.04));
	style_polylines[5].push_back(glm::vec2(0.29, -0.03));
	style_polylines[5].push_back(glm::vec2(0.47, -0.01));
	style_polylines[5].push_back(glm::vec2(0.59, 0.02));
	style_polylines[5].push_back(glm::vec2(0.75, 0.03));
	style_polylines[5].push_back(glm::vec2(1.03, 0.04));

	style_polylines[6].push_back(glm::vec2(-0.02, 0.04));
	style_polylines[6].push_back(glm::vec2(0.16, -0.01));
	style_polylines[6].push_back(glm::vec2(0.42, -0.06));
	style_polylines[6].push_back(glm::vec2(0.65, -0.07));
	style_polylines[6].push_back(glm::vec2(0.83, -0.04));
	style_polylines[6].push_back(glm::vec2(0.98, -0.02));

	style_polylines[7].push_back(glm::vec2(0.0, 0.0));
	style_polylines[7].push_back(glm::vec2(0.24, 0.02));
	style_polylines[7].push_back(glm::vec2(0.59, 0.03));
	style_polylines[7].push_back(glm::vec2(0.79, 0.01));
	style_polylines[7].push_back(glm::vec2(0.91, -0.01));
	style_polylines[7].push_back(glm::vec2(1.02, -0.04));

	style_polylines[8].push_back(glm::vec2(-0.01, -0.02));
	style_polylines[8].push_back(glm::vec2(0.15, 0.0));
	style_polylines[8].push_back(glm::vec2(0.28, 0.02));
	style_polylines[8].push_back(glm::vec2(0.44, 0.01));
	style_polylines[8].push_back(glm::vec2(0.59, 0.0));
	style_polylines[8].push_back(glm::vec2(0.74, -0.03));
	style_polylines[8].push_back(glm::vec2(0.81, -0.04));
	style_polylines[8].push_back(glm::vec2(0.89, -0.04));
	style_polylines[8].push_back(glm::vec2(0.98, -0.03));

	style_polylines[9].push_back(glm::vec2(0.02, -0.02));
	style_polylines[9].push_back(glm::vec2(0.41, -0.03));
	style_polylines[9].push_back(glm::vec2(0.56, -0.04));
	style_polylines[9].push_back(glm::vec2(0.68, -0.03));
	style_polylines[9].push_back(glm::vec2(0.78, -0.02));
	style_polylines[9].push_back(glm::vec2(0.85, -0.01));
	style_polylines[9].push_back(glm::vec2(0.94, 0.0));
	style_polylines[9].push_back(glm::vec2(0.96, 0.02));
}

void GLWidget3D::drawLineTo(const QPoint &endPoint) {
	QPoint pt1(lastPos.x(), lastPos.y());
	QPoint pt2(endPoint.x(), endPoint.y());

	QPainter painter(&sketch);
	painter.setRenderHint(QPainter::Antialiasing);
	painter.setRenderHint(QPainter::HighQualityAntialiasing);

	painter.setBrush(QBrush(Qt::black));
	painter.drawLine(pt1, pt2);

	strokes.back().push_back(glm::vec2(endPoint.x(), height() - endPoint.y()));

	lastPos = endPoint;
}

/**
 * Clear the canvas.
 */
void GLWidget3D::clearSketch() {
	sketch.fill(qRgba(255, 255, 255, 255));
	strokes.clear();
}

void GLWidget3D::clearGeometry() {
	scene.clear();
	scene.generateGeometry(&renderManager, stage);
	update();
}

/**
* Draw the scene.
*/
void GLWidget3D::drawScene() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glDepthMask(true);

	renderManager.renderAll();
}

/**
 * Load a grammar from a file and generate a 3d geometry.
 * This is only for test usage.
 */
void GLWidget3D::loadCGA(char* filename) {
	renderManager.removeObjects();
	
	try {
		cga::Grammar grammar;
		cga::parseGrammar(filename, grammar);
		cga::CGA::randomParamValues(grammar);

		sc::SceneObject so;
		so.setFootprint(0, 0, 0, 16, 12);

		std::vector<float> params(10);
		for (int i = 0; i < params.size(); ++i) params[i] = 0.5f;
		so.setGrammar("Start", grammar, params, true);
		//so.setGrammar(grammar, params);

		cga::CGA system;
		so.generateGeometry(&system, &renderManager, "");
	}
	catch (const std::string& ex) {
		std::cout << "ERROR:" << std::endl << ex << std::endl;
	}
	catch (const char* ex) {
		std::cout << "ERROR:" << std::endl << ex << std::endl;
	}

	renderManager.updateShadowMap(this, light_dir, light_mvpMatrix);

	update();
}

void GLWidget3D::generateGeometry() {
	scene.generateGeometry(&renderManager, stage);

	//if (stage == "final" || stage == "peek_final") {
		renderManager.updateShadowMap(this, light_dir, light_mvpMatrix);
	//}
}

void GLWidget3D::updateGeometry() {
	scene.updateGeometry(&renderManager, stage);
}

void GLWidget3D::selectOption(int option_index) {
	if (stage == "building") {
		predictBuilding(option_index);
	}
	else if (stage == "roof") {
		predictRoof(option_index);
	}
	else if (stage == "facade") {
		if (scene.faceSelector->selected()) {
			scene.currentObject().setGrammar(scene.faceSelector->selectedFaceName(), grammars["facade"][option_index]);
			generateGeometry();
		}
	}
	else if (stage == "floor") {
		if (scene.faceSelector->selected()) {
			scene.currentObject().setGrammar(scene.faceSelector->selectedFaceName(), grammars["floor"][option_index]);
			generateGeometry();
		}
	}
	else if (stage == "window") {
		predictWindow(option_index);
	}
	else if (stage == "ledge") {
		predictLedge(option_index);
	}

	update();
}

void GLWidget3D::updateBuildingOptions() {
	mainWin->thumbsList->clear();

	std::vector<int> indexes;
	for (int i = 0; i < grammarImages["building"].size(); ++i) {
		indexes.push_back(i);
	}
	std::random_shuffle(indexes.begin(), indexes.end());

	QPainter painter(&sketch);
	for (int i = 0; i < grammarImages["building"].size(); ++i) {
		mainWin->addListItem("???", grammarImages["building"][indexes[i]], indexes[i]);
	}

	predictBuilding(0);

	update();
}

void GLWidget3D::updateRoofOptions() {
	mainWin->thumbsList->clear();

	std::vector<int> indexes;
	for (int i = 0; i < grammarImages["roof"].size(); ++i) {
		indexes.push_back(i);
	}
	std::random_shuffle(indexes.begin(), indexes.end());

	QPainter painter(&sketch);
	for (int i = 0; i < grammarImages["roof"].size(); ++i) {
		mainWin->addListItem("???", grammarImages["roof"][indexes[i]], indexes[i]);
	}

	predictRoof(0);

	update();
}

void GLWidget3D::updateFacadeOptions() {
	mainWin->thumbsList->clear();

	std::pair<int, std::vector<float> > result = LayoutExtractor::extractFacadePattern(width(), height(), strokes, scene.faceSelector->selectedFaceCopy(), camera.mvpMatrix);

	QPainter painter(&sketch);
	mainWin->addListItem("1.00", grammarImages["facade"][result.first], result.first);
	for (int i = 0; i < grammarImages["facade"].size(); ++i) {
		if (i == result.first) continue;

		mainWin->addListItem("0.00", grammarImages["facade"][i], i);
	}

	predictFacade(result.first, result.second);

	update();
}

void GLWidget3D::updateFloorOptions() {
	mainWin->thumbsList->clear();

	std::pair<int, std::vector<float> > result = LayoutExtractor::extractFloorPattern(width(), height(), strokes, scene.faceSelector->selectedFaceCopy(), camera.mvpMatrix);

	QPainter painter(&sketch);
	mainWin->addListItem("1.00", grammarImages["floor"][result.first], result.first);
	for (int i = 0; i < grammarImages["floor"].size(); ++i) {
		if (i == result.first) continue;

		mainWin->addListItem("0.00", grammarImages["floor"][i], i);
	}

	predictFloor(result.first, result.second);

	update();
}

void GLWidget3D::updateWindowOptions() {
	mainWin->thumbsList->clear();

	std::vector<int> indexes;
	for (int i = 0; i < grammarImages["window"].size(); ++i) {
		indexes.push_back(i);
	}
	std::random_shuffle(indexes.begin(), indexes.end());

	QPainter painter(&sketch);
	for (int i = 0; i < grammarImages["window"].size(); ++i) {
		mainWin->addListItem("???", grammarImages["window"][indexes[i]], indexes[i]);
	}

	predictWindow(0);

	update();
}

void GLWidget3D::updateLedgeOptions() {
	mainWin->thumbsList->clear();

	std::vector<int> indexes;
	for (int i = 0; i < grammarImages["ledge"].size(); ++i) {
		indexes.push_back(i);
	}
	std::random_shuffle(indexes.begin(), indexes.end());

	QPainter painter(&sketch);
	for (int i = 0; i < grammarImages["ledge"].size(); ++i) {
		mainWin->addListItem("???", grammarImages["ledge"][indexes[i]], indexes[i]);
	}

	predictLedge(0);

	update();
}

/**
 * Use the sketch as an input to the pretrained network, and obtain the probabilities as output.
 * Then, display the options ordered by the probabilities.
 */
void GLWidget3D::predictBuilding(int grammar_id) {
	time_t start = clock();

	renderManager.removeObjects();

	// convert the sketch to grayscale
	cv::Mat mat(sketch.height(), sketch.width(), CV_8UC3, sketch.bits(), sketch.bytesPerLine());
	cv::Mat grayMat;
	cv::cvtColor(mat, grayMat, CV_BGR2GRAY);

	// resize the sketch
	cv::resize(grayMat, grayMat, cv::Size(256, 256));
	cv::threshold(grayMat, grayMat, 250, 255, CV_THRESH_BINARY);
	cv::resize(grayMat, grayMat, cv::Size(128, 128));
	cv::threshold(grayMat, grayMat, 250, 255, CV_THRESH_BINARY);

	// predict parameter values by deep learning
	//std::vector<float> params = regressions[grammar_id]->Predict(grayMat);

	//renderManager.renderingMode = RenderManager::RENDERING_MODE_BASIC;
	
	//////////////////////// DEBUG ////////////////////////
	std::vector<float> params(7);
	for (int i = 0; i < params.size(); ++i) params[i] = 0.5f;
	std::cout << demo_mode << endl;
	if (demo_mode == 0) {
		if (scene._currentObject == 0) {
			params[0] = 0.2f;
		}
		else if (scene._currentObject == 1) {
			//params[0] = 1.0f;
			params[0] = 0.0f;
		}
		else if (scene._currentObject > 1) {
			params[0] = 0.0;
			params[2] = 1.0f / 3.0f;
			params[4] = 0.0f;
		}
	}

	float offset_x = params[0] * 16 - 8;
	float offset_y = params[1] * 16 - 8;
	float object_width = params[2] * 24 + 4;
	float object_depth = params[3] * 24 + 4;
	offset_x -= object_width * 0.5f;
	offset_y -= object_depth * 0.5f;

	scene.currentObject().setFootprint(offset_x, offset_y, current_z, object_width, object_depth);
	if (scene.faceSelector->selected()) {
		scene.alignObjects(scene.faceSelector->selectedFaceCopy());
	}
	else {
		scene.alignObjects();
	}
	
	//std::cout << offset_x << "," << offset_y << "," << object_width << "," << object_depth << std::endl;

	// remove the first four parameters because they are not included in the grammar
	params.erase(params.begin(), params.begin() + 4);
	
	// set parameter values
	scene.currentObject().setGrammar("Start", grammars["building"][grammar_id], params, true);
	
	// set height
	std::vector<std::pair<float, float> > ranges = cga::CGA::getParamRanges(grammars["building"][grammar_id]);
	scene.currentObject().setHeight((ranges[0].second - ranges[0].first) * params[0] + ranges[0].first);
	
	generateGeometry();

	time_t end = clock();
	//std::cout << "Duration: " << (double)(end - start) / CLOCKS_PER_SEC << "sec." << std::endl;

	update();
}

void GLWidget3D::predictRoof(int grammar_id) {
	//if (scene._selectedFaceName.empty()) {
	if (!scene.faceSelector->selected()) {
		std::cout << "Warning: face is not selected." << std::endl;
		return;
	}

	time_t start = clock();

	renderManager.removeObjects();

	//////////////////////// DEBUG ////////////////////////
	scene.currentObject().setGrammar(scene.faceSelector->selectedFaceName(), grammars["roof"][grammar_id]);

	generateGeometry();

	// select the face again
	//faceSelector.reselectFace();
	//scene.updateGeometry(&renderManager, stage);

	time_t end = clock();
	//std::cout << "Duration: " << (double)(end - start) / CLOCKS_PER_SEC << "sec." << std::endl;

	update();
}

void GLWidget3D::predictFacade(int grammar_id, const std::vector<float>& params) {
	//if (scene._selectedFaceName.empty()) {
	if (!scene.faceSelector->selected()) {
		std::cout << "Warning: face is not selected." << std::endl;
		return;
	}

	// set grammar
	scene.currentObject().setGrammar(scene.faceSelector->selectedFaceName(), grammars["facade"][grammar_id], params, false);
	generateGeometry();

	// select the face again
	//faceSelector.reselectFace();
	//scene.updateGeometry(&renderManager, stage);

	update();
}

void GLWidget3D::predictFloor(int grammar_id, const std::vector<float>& params) {
	//if (scene._selectedFaceName.empty()) {
	if (!scene.faceSelector->selected()) {
		std::cout << "Warning: face is not selected." << std::endl;
		return;
	}

	// set grammar
	scene.currentObject().setGrammar(scene.faceSelector->selectedFaceName(), grammars["floor"][grammar_id], params, false);
	generateGeometry();

	// select the face again
	//faceSelector.reselectFace();
	//scene.updateGeometry(&renderManager, stage);

	update();
}

void GLWidget3D::predictWindow(int grammar_id) {
	//if (scene._selectedFaceName.empty()) {
	if (!scene.faceSelector->selected()) {
		std::cout << "Warning: face is not selected." << std::endl;
		return;
	}

	time_t start = clock();

	renderManager.removeObjects();

	//////////////////////// DEBUG ////////////////////////
	scene.currentObject().setGrammar(scene.faceSelector->selectedFaceName(), grammars["window"][grammar_id]);

	generateGeometry();

	// select the face again
	//faceSelector.reselectFace();
	//scene.updateGeometry(&renderManager, stage);

	time_t end = clock();
	//std::cout << "Duration: " << (double)(end - start) / CLOCKS_PER_SEC << "sec." << std::endl;

	update();
}

void GLWidget3D::predictLedge(int grammar_id) {
	//if (scene._selectedFaceName.empty()) {
	if (!scene.faceSelector->selected()) {
		std::cout << "Warning: face is not selected." << std::endl;
		return;
	}

	time_t start = clock();

	renderManager.removeObjects();

	//////////////////////// DEBUG ////////////////////////
	scene.currentObject().setGrammar(scene.faceSelector->selectedFaceName(), grammars["ledge"][grammar_id]);

	generateGeometry();

	// select the face again
	//faceSelector.reselectFace();
	//scene.updateGeometry(&renderManager, stage);

	time_t end = clock();
	//std::cout << "Duration: " << (double)(end - start) / CLOCKS_PER_SEC << "sec." << std::endl;

	update();
}

bool GLWidget3D::selectFace(const glm::vec2& mouse_pos) {
	// camera position in the world coordinates
	glm::vec3 cameraPos = camera.cameraPosInWorld();

	clearSketch();

	// view direction
	glm::vec3 view_dir = viewVector(mouse_pos, camera.mvMatrix, camera.f(), camera.aspect());

	if (stage == "building") {
		if (scene.faceSelector->selectFace(cameraPos, view_dir, stage, glm::vec3(0, 1, 0))) {
			scene.newObject();

			// shift the camera such that the selected face becomes a ground plane.
			intCamera = InterpolationCamera(camera, 30, -45, 0, computeDownwardedCameraPos(scene.faceSelector->selectedFace()->vertices[0].position.y + CAMERA_DEFAULT_HEIGHT, CAMERA_DEFAULT_DEPTH, intCamera.camera_end.xrot));
			current_z = scene.faceSelector->selectedFace()->vertices[0].position.y;

			scene.faceSelector->selectedFace()->select();
		}
		else {
			scene.newObject();

			// shift the camera such that the ground plane becomes really a ground plane.
			intCamera = InterpolationCamera(camera, 30, -45, 0, computeDownwardedCameraPos(CAMERA_DEFAULT_HEIGHT, CAMERA_DEFAULT_DEPTH, intCamera.camera_end.xrot));
			current_z = 0;
		}

		return true;
	}
	else if (stage == "roof") {
		if (scene.faceSelector->selectFace(cameraPos, view_dir, stage, glm::vec3(0, 1, 0))) {
			// make the yrot in the rage [-180,179]
			camera.yrot = (int)(camera.yrot + 360 * 10) % 360;
			if (camera.yrot > 180) camera.yrot -= 360;

			// find the nearest quadrant
			float yrot = 0.0f;
			if (camera.yrot >= -90 && camera.yrot <= 0) {
				yrot = -45.0f;
			}
			else if (camera.yrot > 0 && camera.yrot <= 90) {
				yrot = 45.0f;
			}
			else if (camera.yrot > 90) {
				yrot = 135.0f;
			}
			else {
				yrot = -135.0f;
			}

			// shift the camera such that the selected face becomes a ground plane.
			intCamera = InterpolationCamera(camera, 30, yrot, 0.0, glm::vec3(0, scene.faceSelector->selectedFace()->vertices[0].position.y, CAMERA_DEFAULT_DEPTH));
			current_z = scene.faceSelector->selectedFace()->vertices[0].position.y;

			scene.faceSelector->selectedFace()->select();

			return true;
		}
		else {
			return false;
		}
	}
	else if (stage == "facade") {
		if (scene.faceSelector->selectFace(cameraPos, view_dir, stage, glm::vec3(1, 0, 1))) {
			// compute appropriate camera distance for the selected face
			float rot_y = atan2f(scene.faceSelector->selectedFace()->vertices[0].normal.x, scene.faceSelector->selectedFace()->vertices[0].normal.z);
			glutils::Face rotatedFace = scene.faceSelector->selectedFace()->rotate(-rot_y, glm::vec3(0, 1, 0));
			float d1 = rotatedFace.bbox.sx() * 0.5f / tanf(camera.fovy * M_PI / 180.0f * 0.5f);
			float d2 = rotatedFace.bbox.sy() * 0.5f / tanf(camera.fovy * M_PI / 180.0f * 0.5f);
			float d = std::max(d1, d2) * 1.5f;

			// turn the camera such that the selected face becomes parallel to the image plane.
			intCamera = InterpolationCamera(camera, 0, -rot_y / M_PI * 180, 0, glm::vec3(rotatedFace.bbox.center().x, rotatedFace.bbox.center().y, rotatedFace.bbox.maxPt.z + d));

			scene.faceSelector->selectedFace()->select();

			return true;
		}
		else {
			return false;
		}
	}
	else if (stage == "floor") {
		if (scene.faceSelector->selectFace(cameraPos, view_dir, stage, glm::vec3(1, 0, 1))) {
			// compute appropriate camera distance for the selected face
			float rot_y = atan2f(scene.faceSelector->selectedFace()->vertices[0].normal.x, scene.faceSelector->selectedFace()->vertices[0].normal.z);
			glutils::Face rotatedFace = scene.faceSelector->selectedFace()->rotate(-rot_y, glm::vec3(0, 1, 0));
			float d1 = rotatedFace.bbox.sx() * 0.5f / tanf(camera.fovy * M_PI / 180.0f * 0.5f);
			float d2 = rotatedFace.bbox.sy() * 0.5f / tanf(camera.fovy * M_PI / 180.0f * 0.5f);
			float d = std::max(d1, d2) * 1.5f;

			// turn the camera such that the selected face becomes parallel to the image plane.
			intCamera = InterpolationCamera(camera, 0, -rot_y / M_PI * 180, 0, glm::vec3(rotatedFace.bbox.center().x, rotatedFace.bbox.center().y, rotatedFace.bbox.maxPt.z + d));

			scene.faceSelector->selectedFace()->select();

			return true;
		}
		else {
			return false;
		}
	}
	else if (stage == "window") {
		if (scene.faceSelector->selectFace(cameraPos, view_dir, stage, glm::vec3(1, 0, 1))) {
			// compute appropriate camera distance for the selected face
			float rot_y = atan2f(scene.faceSelector->selectedFace()->vertices[0].normal.x, scene.faceSelector->selectedFace()->vertices[0].normal.z);
			glutils::Face rotatedFace = scene.faceSelector->selectedFace()->rotate(-rot_y, glm::vec3(0, 1, 0));
			float d1 = rotatedFace.bbox.sx() * 0.5f / tanf(camera.fovy * M_PI / 180.0f * 0.5f);
			float d2 = rotatedFace.bbox.sy() * 0.5f / tanf(camera.fovy * M_PI / 180.0f * 0.5f);
			float d = std::max(d1, d2) * 1.5f;

			// turn the camera such that the selected face becomes parallel to the image plane.
			intCamera = InterpolationCamera(camera, 0, -rot_y / M_PI * 180, 0, glm::vec3(rotatedFace.bbox.center().x, rotatedFace.bbox.center().y, rotatedFace.bbox.maxPt.z + d));

			scene.faceSelector->selectedFace()->select();

			return true;
		}
		else {
			return false;
		}
	}
	else if (stage == "ledge") {
		if (scene.faceSelector->selectFace(cameraPos, view_dir, stage, glm::vec3(1, 0, 1))) {
			// compute appropriate camera distance for the selected face
			float rot_y = -M_PI * 0.4 + atan2f(scene.faceSelector->selectedFace()->vertices[0].normal.x, scene.faceSelector->selectedFace()->vertices[0].normal.z);
			glutils::Face rotatedFace = scene.faceSelector->selectedFace()->rotate(-rot_y, glm::vec3(0, 1, 0));
			float d1 = rotatedFace.bbox.sx() * 0.5f / tanf(camera.fovy * M_PI / 180.0f * 0.5f);
			float d2 = rotatedFace.bbox.sy() * 0.5f / tanf(camera.fovy * M_PI / 180.0f * 0.5f);
			float d = std::max(d1, d2) * 1.5f;

			// turn the camera such that the selected face becomes parallel to the image plane.
			intCamera = InterpolationCamera(camera, 0, -rot_y / M_PI * 180, 0, glm::vec3(rotatedFace.bbox.center().x, rotatedFace.bbox.center().y, rotatedFace.bbox.maxPt.z + d));

			scene.faceSelector->selectedFace()->select();

			return true;
		}
		else {
			return false;
		}
	}
}

bool GLWidget3D::selectBuilding(const glm::vec2& mouse_pos) {
	// camera position in the world coordinates
	glm::vec3 cameraPos = camera.cameraPosInWorld();

	clearSketch();

	// view direction
	glm::vec3 view_dir = viewVector(mouse_pos, camera.mvMatrix, camera.f(), camera.aspect());

	if (scene.buildingSelector->selectBuilding(cameraPos, view_dir)) {
		return true;
	}
	else {
		return false;
	}
}

bool GLWidget3D::selectBuildingControlPoint(const glm::vec2& mouse_pos) {
	// camera position in the world coordinates
	glm::vec3 cameraPos = camera.cameraPosInWorld();

	// view direction
	glm::vec3 view_dir = viewVector(mouse_pos, camera.mvMatrix, camera.f(), camera.aspect());

	// select a control point
	if (scene.buildingSelector->selectBuildingControlPoint(cameraPos, view_dir, mouse_pos, camera.mvpMatrix, width(), height())) {
		return true;
	}
	else {
		return false;
	}
}

void GLWidget3D::addBuildingMass() {
	if (stage != "building") return;

	clearSketch();
	scene.newObject();
	update();
}

glm::vec3 GLWidget3D::viewVector(const glm::vec2& point, const glm::mat4& mvMatrix, float focalLength, float aspect) {
	glm::vec3 dir((point.x - width() * 0.5f) * 2.0f / width() * aspect, (height() * 0.5f - point.y) * 2.0f / height(), -focalLength);
	return glm::vec3(glm::inverse(mvMatrix) * glm::vec4(dir, 0));
}

void GLWidget3D::changeStage(const std::string& stage) {
	this->stage = stage;
	clearSketch();

	// the selected building should be unselected.
	scene.buildingSelector->unselectBuilding();

	scene.faceSelector->unselect();

	if (stage == "building") {
		intCamera = InterpolationCamera(camera, 30, -45, 0, glm::vec3(0, CAMERA_DEFAULT_HEIGHT, CAMERA_DEFAULT_DEPTH));
		current_z = 0.0f;

		camera_timer = new QTimer(this);
		connect(camera_timer, SIGNAL(timeout()), mainWin, SLOT(camera_update()));
		camera_timer->start(20);
	}
	else if (stage == "roof") {
	}
	else if (stage == "facade") {
	}
	else if (stage == "floor") {
	}
	else if (stage == "window") {
	}
	else if (stage == "ledge") {
	}
	else if (stage == "final") {
		generateGeometry();
	}

	updateGeometry();

	update();
}

void GLWidget3D::changeMode(int new_mode) {
	if (new_mode == MODE_COPY) {
		if (scene.buildingSelector->isBuildingSelected()) {
			scene.buildingSelector->copy();
			generateGeometry();
		}
	}
	else if (new_mode == MODE_ERASER) {
		clearSketch();

		if (stage == "building") {
			scene.clearCurrentObject();
		}

	}
	else {
		if (scene.buildingSelector->isBuildingSelected()) {
			scene.buildingSelector->unselectBuilding();
			updateGeometry();
		}

		mode = new_mode;
	}

	renderManager.updateShadowMap(this, light_dir, light_mvpMatrix);

	update();
}

void GLWidget3D::camera_update() {
	if (intCamera.forward()) {
		camera_timer->stop();
	}

	camera = intCamera.currentCamera();
	camera.updateMVPMatrix();
	update();
}

glm::vec3 GLWidget3D::computeDownwardedCameraPos(float downward, float distToCamera, float camera_xrot) {
	return glm::vec3(0,
		downward * cosf(camera_xrot / 180.0f * M_PI),
		downward * sinf(camera_xrot / 180.0f * M_PI) + distToCamera);
}

void GLWidget3D::keyPressEvent(QKeyEvent *e) {
	ctrlPressed = false;

	switch (e->key()) {
	case Qt::Key_Control:
		ctrlPressed = true;
		break;
	case Qt::Key_0:
		demo_mode = 0;
		break;
	case Qt::Key_1:
		demo_mode = 1;
		break;
	default:
		break;
	}
}

void GLWidget3D::keyReleaseEvent(QKeyEvent* e) {
	ctrlPressed = false;
}

/**
 * This event handler is called when the mouse button is pressed.
 */
void GLWidget3D::mousePressEvent(QMouseEvent *e) {
	dragging = true;

	if (mode == MODE_CAMERA) { // move camera
		camera.mousePress(e->x(), e->y());
	}
	else if (mode == MODE_SELECT) {
		// do nothing
	}
	else if (mode == MODE_SELECT_BUILDING) {
		if (scene.buildingSelector->isBuildingSelected()) {
			selectBuildingControlPoint(glm::vec2(e->x(), e->y()));

			updateGeometry();
			update();
		}
	}
	else { // start drawing a stroke
		lastPos = e->pos();
		strokes.resize(strokes.size() + 1);
	}
}

/**
 * This event handler is called when the mouse button is released.
 */
void GLWidget3D::mouseReleaseEvent(QMouseEvent *e) {
	dragging = false;

	if (mode == MODE_CAMERA) {
		// do nothing
	} 
	else if (mode == MODE_SELECT) { // select a face
		if (selectFace(glm::vec2(e->x(), e->y()))) {
			updateGeometry();

			// When a face is selected, the user should start drawing.
			mode = MODE_SKETCH;
			mainWin->actionModes["sketch"]->setChecked(true);

			camera_timer = new QTimer(this);
			connect(camera_timer, SIGNAL(timeout()), mainWin, SLOT(camera_update()));
			camera_timer->start(20);

			update();
		}
	}
	else if (mode == MODE_SELECT_BUILDING) { // select a building
		if (scene.buildingSelector->isBuildingControlPointSelected()) {
			scene.buildingSelector->alignObjects();
			scene.buildingSelector->unselectBuildingControlPoint();
			generateGeometry();
		}
		else {
			if (selectBuilding(glm::vec2(e->x(), e->y()))) {
				std::cout << "A building is selected." << std::endl;
			}
		}

		updateGeometry();

		update();
	}
	else {
		if (stage == "building") {
			updateBuildingOptions();
		}
		else if (stage == "roof") {
			updateRoofOptions();
		}
		else if (stage == "facade") {
			updateFacadeOptions();
		}
		else if (stage == "floor") {
			updateFloorOptions();
		}
		else if (stage == "window") {
			updateWindowOptions();
		}
		else if (stage == "ledge") {
			updateLedgeOptions();
		}
	}
}

/**
 * This event handler is called when the mouse is dragged.
 */
void GLWidget3D::mouseMoveEvent(QMouseEvent *e) {
	if (dragging) {
		if (mode == MODE_CAMERA) {
			if (e->buttons() & Qt::LeftButton) { // Rotate
				camera.rotate(e->x(), e->y());
			}
			else if (e->buttons() & Qt::MidButton) { // Move
				camera.move(e->x(), e->y());
			}
			else if (e->buttons() & Qt::RightButton) { // Zoom
				camera.zoom(e->x(), e->y());
			}
			clearSketch();
		}
		else if (mode == MODE_SELECT) {
			// do nothing
		}
		else if (mode == MODE_SELECT_BUILDING) {
			if (scene.buildingSelector->isBuildingControlPointSelected()) {
				// resize the building
				scene.buildingSelector->resize(glm::vec2(e->x(), e->y()));
				generateGeometry();
			}
		}
		else { // keep drawing a stroke
			drawLineTo(e->pos());
		}

		update();
	}
	else {
		if (e->y() > height() - BOTTOM_AREA_HEIGHT) {
			if (stage != "peek_final") {
				preStage = stage;
				stage = "peek_final";

				// generate final geometry
				generateGeometry();

				update();
			}
		}
		else {
			if (stage == "peek_final") {
				stage = preStage;

				// generate geometry
				generateGeometry();

				update();
			}
		}
	}
}

/**
 * This function is called once before the first call to paintGL() or resizeGL().
 */
void GLWidget3D::initializeGL() {
	// init glew
	GLenum err = glewInit();
	if (err != GLEW_OK) {
		std::cout << "Error: " << glewGetErrorString(err) << std::endl;
	}

	if (glewIsSupported("GL_VERSION_4_2"))
		printf("Ready for OpenGL 4.2\n");
	else {
		printf("OpenGL 4.2 not supported\n");
		exit(1);
	}
	const GLubyte* text = glGetString(GL_VERSION);
	printf("VERSION: %s\n", text);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	glEnable(GL_TEXTURE_2D);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);

	glTexGenf(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGenf(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glDisable(GL_TEXTURE_2D);

	glEnable(GL_TEXTURE_3D);
	glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glDisable(GL_TEXTURE_3D);

	glEnable(GL_TEXTURE_2D_ARRAY);
	glTexParameterf(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameterf(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glDisable(GL_TEXTURE_2D_ARRAY);

	////////////////////////////////
	renderManager.init("", "", "", true, 8192);
	renderManager.resize(this->width(), this->height());

	glUniform1i(glGetUniformLocation(renderManager.programs["ssao"], "tex0"), 0);//tex0: 0





	sketch = QImage(this->width(), this->height(), QImage::Format_RGB888);
	sketch.fill(qRgba(255, 255, 255, 255));

	mode = MODE_SKETCH;
	
	camera.xrot = 30.0f;
	camera.yrot = -45.0f;
	camera.zrot = 0.0f;
	camera.pos = computeDownwardedCameraPos(CAMERA_DEFAULT_HEIGHT, CAMERA_DEFAULT_DEPTH, camera.xrot);
	current_z = 0.0f;
	scene.updateGeometry(&renderManager, "building");
	renderManager.updateShadowMap(this, light_dir, light_mvpMatrix);

	//changeStage("building");
	stage = "building";

	setMouseTracking(true);
}

/**
 * This function is called whenever the widget has been resized.
 */
void GLWidget3D::resizeGL(int width, int height) {
	height = height?height:1;

	glViewport(0, 0, (GLint)width, (GLint)height );
	camera.updatePMatrix(width, height);
	renderManager.resize(width, height);

	QImage newImage(width, height, QImage::Format_RGB888);
	newImage.fill(qRgba(255, 255, 255, 255));
	//QPainter painter(&newImage);
	//painter.drawImage(0, 0, sketch);
	sketch = newImage;
}

void GLWidget3D::paintEvent(QPaintEvent *event) {
	// OpenGLで描画
	makeCurrent();

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// PASS 1: Render to texture
	glUseProgram(renderManager.programs["pass1"]);

	glBindFramebuffer(GL_FRAMEBUFFER, renderManager.fragDataFB);
	glClearColor(0.95, 0.95, 0.95, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, renderManager.fragDataTex[0], 0);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, renderManager.fragDataTex[1], 0);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, renderManager.fragDataTex[2], 0);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, GL_TEXTURE_2D, renderManager.fragDataTex[3], 0);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, renderManager.fragDepthTex, 0);

	// Set the list of draw buffers.
	GLenum DrawBuffers[4] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2, GL_COLOR_ATTACHMENT3 };
	glDrawBuffers(4, DrawBuffers); // "3" is the size of DrawBuffers
	// Always check that our framebuffer is ok
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
		printf("+ERROR: GL_FRAMEBUFFER_COMPLETE false\n");
		exit(0);
	}

	glUniformMatrix4fv(glGetUniformLocation(renderManager.programs["pass1"], "mvpMatrix"), 1, false, &camera.mvpMatrix[0][0]);
	glUniform3f(glGetUniformLocation(renderManager.programs["pass1"], "lightDir"), light_dir.x, light_dir.y, light_dir.z);
	glUniformMatrix4fv(glGetUniformLocation(renderManager.programs["pass1"], "light_mvpMatrix"), 1, false, &light_mvpMatrix[0][0]);

	glUniform1i(glGetUniformLocation(renderManager.programs["pass1"], "shadowMap"), 6);
	glActiveTexture(GL_TEXTURE6);
	glBindTexture(GL_TEXTURE_2D, renderManager.shadow.textureDepth);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	drawScene();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// PASS 2: Create AO
	if (renderManager.renderingMode == RenderManager::RENDERING_MODE_SSAO) {
		glUseProgram(renderManager.programs["ssao"]);
		glBindFramebuffer(GL_FRAMEBUFFER, renderManager.fragDataFB_AO);

		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, renderManager.fragAOTex, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, renderManager.fragDepthTex_AO, 0);
		GLenum DrawBuffers[1] = { GL_COLOR_ATTACHMENT0 };
		glDrawBuffers(1, DrawBuffers); // "1" is the size of DrawBuffers

		glClearColor(1, 1, 1, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Always check that our framebuffer is ok
		if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
			printf("++ERROR: GL_FRAMEBUFFER_COMPLETE false\n");
			exit(0);
		}

		glDisable(GL_DEPTH_TEST);
		glDepthFunc(GL_ALWAYS);

		glUniform2f(glGetUniformLocation(renderManager.programs["ssao"], "pixelSize"), 2.0f / this->width(), 2.0f / this->height());

		glUniform1i(glGetUniformLocation(renderManager.programs["ssao"], "tex0"), 1);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[0]);

		glUniform1i(glGetUniformLocation(renderManager.programs["ssao"], "tex1"), 2);
		glActiveTexture(GL_TEXTURE2);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[1]);

		glUniform1i(glGetUniformLocation(renderManager.programs["ssao"], "tex2"), 3);
		glActiveTexture(GL_TEXTURE3);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[2]);

		glUniform1i(glGetUniformLocation(renderManager.programs["ssao"], "depthTex"), 8);
		glActiveTexture(GL_TEXTURE8);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDepthTex);

		glUniform1i(glGetUniformLocation(renderManager.programs["ssao"], "noiseTex"), 7);
		glActiveTexture(GL_TEXTURE7);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragNoiseTex);

		{
			glUniformMatrix4fv(glGetUniformLocation(renderManager.programs["ssao"], "mvpMatrix"), 1, false, &camera.mvpMatrix[0][0]);
			glUniformMatrix4fv(glGetUniformLocation(renderManager.programs["ssao"], "pMatrix"), 1, false, &camera.pMatrix[0][0]);
		}

		glUniform1i(glGetUniformLocation(renderManager.programs["ssao"], "uKernelSize"), renderManager.uKernelSize);
		glUniform3fv(glGetUniformLocation(renderManager.programs["ssao"], "uKernelOffsets"), renderManager.uKernelOffsets.size(), (const GLfloat*)renderManager.uKernelOffsets.data());

		glUniform1f(glGetUniformLocation(renderManager.programs["ssao"], "uPower"), renderManager.uPower);
		glUniform1f(glGetUniformLocation(renderManager.programs["ssao"], "uRadius"), renderManager.uRadius);

		glBindVertexArray(renderManager.secondPassVAO);

		glDrawArrays(GL_QUADS, 0, 4);
		glBindVertexArray(0);
		glDepthFunc(GL_LEQUAL);
	}
	else if (renderManager.renderingMode == RenderManager::RENDERING_MODE_LINE || renderManager.renderingMode == RenderManager::RENDERING_MODE_HATCHING) {
		glUseProgram(renderManager.programs["line"]);

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glClearColor(1, 1, 1, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glDisable(GL_DEPTH_TEST);
		glDepthFunc(GL_ALWAYS);

		glUniform2f(glGetUniformLocation(renderManager.programs["line"], "pixelSize"), 1.0f / this->width(), 1.0f / this->height());
		glUniformMatrix4fv(glGetUniformLocation(renderManager.programs["line"], "pMatrix"), 1, false, &camera.pMatrix[0][0]);
		if (renderManager.renderingMode == RenderManager::RENDERING_MODE_LINE) {
			glUniform1i(glGetUniformLocation(renderManager.programs["line"], "useHatching"), 0);
		}
		else {
			glUniform1i(glGetUniformLocation(renderManager.programs["line"], "useHatching"), 1);
		}

		glUniform1i(glGetUniformLocation(renderManager.programs["line"], "tex0"), 1);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[0]);

		glUniform1i(glGetUniformLocation(renderManager.programs["line"], "tex1"), 2);
		glActiveTexture(GL_TEXTURE2);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[1]);

		glUniform1i(glGetUniformLocation(renderManager.programs["line"], "tex2"), 3);
		glActiveTexture(GL_TEXTURE3);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[2]);

		glUniform1i(glGetUniformLocation(renderManager.programs["line"], "tex3"), 4);
		glActiveTexture(GL_TEXTURE4);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[3]);

		glUniform1i(glGetUniformLocation(renderManager.programs["line"], "depthTex"), 8);
		glActiveTexture(GL_TEXTURE8);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDepthTex);

		glUniform1i(glGetUniformLocation(renderManager.programs["line"], "hatchingTexture"), 5);
		glActiveTexture(GL_TEXTURE5);
		glBindTexture(GL_TEXTURE_3D, renderManager.hatchingTextures);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

		glBindVertexArray(renderManager.secondPassVAO);

		glDrawArrays(GL_QUADS, 0, 4);
		glBindVertexArray(0);
		glDepthFunc(GL_LEQUAL);
	}


	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Blur

	if (renderManager.renderingMode != RenderManager::RENDERING_MODE_LINE && renderManager.renderingMode != RenderManager::RENDERING_MODE_HATCHING) {
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		qglClearColor(QColor(0xFF, 0xFF, 0xFF));
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glDisable(GL_DEPTH_TEST);
		glDepthFunc(GL_ALWAYS);

		glUseProgram(renderManager.programs["blur"]);
		glUniform2f(glGetUniformLocation(renderManager.programs["blur"], "pixelSize"), 2.0f / this->width(), 2.0f / this->height());
		//printf("pixelSize loc %d\n", glGetUniformLocation(vboRenderManager.programs["blur"], "pixelSize"));

		glUniform1i(glGetUniformLocation(renderManager.programs["blur"], "tex0"), 1);//COLOR
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[0]);

		glUniform1i(glGetUniformLocation(renderManager.programs["blur"], "tex1"), 2);//NORMAL
		glActiveTexture(GL_TEXTURE2);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[1]);

		/*glUniform1i(glGetUniformLocation(renderManager.programs["blur"], "tex2"), 3);
		glActiveTexture(GL_TEXTURE3);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[2]);*/

		glUniform1i(glGetUniformLocation(renderManager.programs["blur"], "depthTex"), 8);
		glActiveTexture(GL_TEXTURE8);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDepthTex);

		glUniform1i(glGetUniformLocation(renderManager.programs["blur"], "tex3"), 4);//AO
		glActiveTexture(GL_TEXTURE4);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragAOTex);

		if (renderManager.renderingMode == RenderManager::RENDERING_MODE_SSAO) {
			glUniform1i(glGetUniformLocation(renderManager.programs["blur"], "ssao_used"), 1); // ssao used
		}
		else {
			glUniform1i(glGetUniformLocation(renderManager.programs["blur"], "ssao_used"), 0); // no ssao
		}

		glBindVertexArray(renderManager.secondPassVAO);

		glDrawArrays(GL_QUADS, 0, 4);
		glBindVertexArray(0);
		glDepthFunc(GL_LEQUAL);

	}

	// REMOVE
	glActiveTexture(GL_TEXTURE0);


	//printf("<<\n");
	//VBOUtil::disaplay_memory_usage();




	// OpenGLの設定を元に戻す
	glShadeModel(GL_FLAT);
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	// draw sketch
	QPainter painter(this);
	//painter.setOpacity(0.5);
	//painter.drawImage(0, 0, sketch);
	QPen pen(Qt::blue, 3);
	painter.setPen(pen);
	for (auto stroke : strokes) {
		for (int i = 0; i < (int)stroke.size() - 1; ++i) {
			painter.drawLine(stroke[i].x, height() - stroke[i].y, stroke[i + 1].x, height() - stroke[i + 1].y);
		}
	}

	// draw the bottom area
	painter.setOpacity(0.6);
	QRect bottomArea(0, height() - BOTTOM_AREA_HEIGHT, width(), BOTTOM_AREA_HEIGHT);
	painter.fillRect(bottomArea, Qt::white);
	QFont font = painter.font();
	font.setPointSize(font.pointSize() * 3);
	painter.setFont(font);
	painter.drawText(bottomArea, Qt::AlignCenter | Qt::AlignVCenter, tr("Peek a final view"));
	painter.end();

	glEnable(GL_DEPTH_TEST);
}
