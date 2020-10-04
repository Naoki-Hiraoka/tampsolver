#ifndef EDITCOLLISIONMODEL_H
#define EDITCOLLISIONMODEL_H

#include <cnoid/Body>
#include <cnoid/SceneDrawables>
#include <vclip.h>

namespace RobotConfig {
  cnoid::SgMesh* convertToSgMesh (cnoid::SgNode* collisionshape);

  cnoid::SgNode* convertToConvexHull (cnoid::SgNode* collisionshape);

  Vclip::Polyhedron* convertToVClipModel (cnoid::SgNode* collisionshape);
}
#endif
