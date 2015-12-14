﻿#include "Camera.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iostream>

#ifndef M_PI
#define M_PI	3.1415926535
#endif

Camera::Camera() {
	xrot = 30.0f;
	yrot = -45.0;
	zrot = 0.0f;
	pos = glm::vec3(0, 0, 50);
	fovy = 45.0f;
	_f = 1.0f / tan(fovy * M_PI / 360.0f);
}

/**
 * Call this function when the mouse button is pressed.
 */
void Camera::mousePress(int mouse_x, int mouse_y) {
	mouse_pos = glm::vec2(mouse_x, mouse_y);
}

/**
 * Call this function whenever the mouse moves while rotating the model.
 */
void Camera::rotate(int mouse_x, int mouse_y) {
	xrot += mouse_y - mouse_pos.y;
	yrot += mouse_x - mouse_pos.x;
	updateMVPMatrix();

	mouse_pos = glm::vec2(mouse_x, mouse_y);
}

/**
 * Call this function whenever the mouse moves while zooming.
 */
void Camera::zoom(int mouse_x, int mouse_y) {
	pos.z += (mouse_pos.y - mouse_y) * 0.5f;
	updateMVPMatrix();

	mouse_pos = glm::vec2(mouse_x, mouse_y);
}

/**
 * Call this function whenever the mouse moves while moving the model.
 */
void Camera::move(int mouse_x, int mouse_y) {
	pos.x -= (mouse_x - mouse_pos.x) * 0.1;
	pos.y += (mouse_y - mouse_pos.y) * 0.1;
	updateMVPMatrix();

	mouse_pos = glm::vec2(mouse_x, mouse_y);
}

/**
 * Update perspective projection matrix, and then, update the model view projection matrix.
 */
void Camera::updatePMatrix(int width,int height) {
	_aspect = (float)width / (float)height;
	float zfar = 3000.0f;
	float znear = 0.1f;

	// projection行列
	// ただし、mat4はcolumn majorなので、転置した感じで初期構築する。
	// つまり、下記の一行目は、mat4の一列目に格納されるのだ。
	pMatrix = glm::mat4(
		 _f/_aspect,	0,								0,									0,
				0,	_f,								0,						 			0,
			    0,	0,		(zfar+znear)/(znear-zfar),		                           -1,
			    0,	0, (2.0f*zfar*znear)/(znear-zfar),									0);

	updateMVPMatrix();
}

/**
 * Update the model view projection matrix
 */
void Camera::updateMVPMatrix() {
	// create model view matrix
	mvMatrix = glm::mat4();
	mvMatrix = glm::translate(mvMatrix, -pos);
	mvMatrix = glm::rotate(mvMatrix, xrot * (float)M_PI / 180.0f, glm::vec3(1, 0, 0));
	mvMatrix = glm::rotate(mvMatrix, yrot * (float)M_PI / 180.0f, glm::vec3(0, 1, 0));
	mvMatrix = glm::rotate(mvMatrix, zrot * (float)M_PI / 180.0f, glm::vec3(0, 0, 1));

	// create model view projection matrix
	mvpMatrix = pMatrix * mvMatrix;

	glm::vec4 p1(0, 0, 0, 1);
	glm::vec4 p2(-16, 0, 0, 1);
	glm::vec4 p3(0, 22, 0, 1);
	glm::vec4 p4(-16, 22, 0, 1);
	p1 = mvpMatrix * p1;
	p2 = mvpMatrix * p2;
	p3 = mvpMatrix * p3;
	p4 = mvpMatrix * p4;
	std::cout << "--------------------------" << std::endl;
	std::cout << "p1: " << p1.x/p1.w << "," << p1.y/p1.w << std::endl;
	std::cout << "p2: " << p2.x / p2.w << "," << p2.y / p2.w << std::endl;
	std::cout << "p3: " << p3.x / p3.w << "," << p3.y / p3.w << std::endl;
	std::cout << "p4: " << p4.x / p4.w << "," << p4.y / p4.w << std::endl;
	std::cout << "--------------------------" << std::endl;
}

glm::vec3 Camera::cameraPosInWorld() {
	return glm::vec3(glm::inverse(mvMatrix) * glm::vec4(0, 0, 0, 1));
}

glm::vec3 Camera::cameraViewDir() {
	return glm::normalize(glm::vec3(cosf(xrot * (float)M_PI / 180.0f) * sinf(yrot * (float)M_PI / 180.0f), -sinf(xrot * (float)M_PI / 180.0f), -cosf(xrot * (float)M_PI / 180.0f) * cosf(yrot * (float)M_PI / 180.0f)));
}