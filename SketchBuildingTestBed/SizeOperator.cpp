#include "SizeOperator.h"
#include "CGA.h"
#include "Shape.h"

namespace cga {

SizeOperator::SizeOperator(const Value& xSize, const Value& ySize, const Value& zSize, bool centered) {
	this->name = "size";
	this->xSize = xSize;
	this->ySize = ySize;
	this->zSize = zSize;
	this->centered = centered;
}

boost::shared_ptr<Shape> SizeOperator::apply(boost::shared_ptr<Shape>& shape, const Grammar& grammar, std::list<boost::shared_ptr<Shape> >& stack) {
	float actual_xSize;
	float actual_ySize;
	float actual_zSize;

	if (xSize.type == Value::TYPE_RELATIVE) {
		actual_xSize = shape->_scope.x * grammar.evalFloat(xSize.value, shape);
	} else {
		actual_xSize = grammar.evalFloat(xSize.value, shape);
	}

	if (ySize.type == Value::TYPE_RELATIVE) {
		actual_ySize = shape->_scope.y * grammar.evalFloat(ySize.value, shape);
	} else {
		actual_ySize = grammar.evalFloat(ySize.value, shape);
	}

	if (zSize.type == Value::TYPE_RELATIVE) {
		actual_zSize = shape->_scope.z * grammar.evalFloat(zSize.value, shape);
	} else {
		actual_zSize = grammar.evalFloat(zSize.value, shape);
	}

	shape->size(actual_xSize, actual_ySize, actual_zSize, centered);
	return shape;
}

std::string SizeOperator::to_string() {
	std::string ret = "size(";

	if (xSize.type == Value::TYPE_ABSOLUTE) {
		ret += xSize.value;
	}
	else if (xSize.type == Value::TYPE_RELATIVE) {
		ret += "'" + xSize.value;
	}
	else {
		ret += "~" + xSize.value;
	}
	ret += ", ";

	if (ySize.type == Value::TYPE_ABSOLUTE) {
		ret += ySize.value;
	}
	else if (ySize.type == Value::TYPE_RELATIVE) {
		ret += "'" + ySize.value;
	}
	else {
		ret += "~" + ySize.value;
	}
	ret += ", ";

	if (zSize.type == Value::TYPE_ABSOLUTE) {
		ret += zSize.value;
	}
	else if (zSize.type == Value::TYPE_RELATIVE) {
		ret += "'" + zSize.value;
	}
	else {
		ret += "~" + zSize.value;
	}

	if (centered) {
		ret += ", centered";
	}
	ret += ")";

	return ret;
}

}
