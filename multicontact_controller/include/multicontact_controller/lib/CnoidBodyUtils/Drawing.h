#ifndef MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_DRAWING_H
#define MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_DRAWING_H

#include <cnoid/SceneDrawables>
#include <cnoid/Body>

namespace multicontact_controller {
  namespace cnoidbodyutils {
    void drawCoordsLineCoords(cnoid::SgLineSetPtr& lines, const cnoid::Position& A_pos, const cnoid::Position& B_pos);

    void drawPolygon(cnoid::SgLineSetPtr& lines, const std::vector<cnoid::Vector2>& vertices, const cnoid::Vector3f color=cnoid::Vector3f(0.0,1.0,0.0));
  };
};

#endif
