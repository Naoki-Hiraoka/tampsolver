#ifndef WALK_CONTROLLER_H
#define WALK_CONTROLLER_H

#include <cnoid/Body>
#include <memory>

namespace multicontact_controller {

  class WalkController {
  public:
  protected:
    std::shared_ptr<cnoid::Body> robot_;
  };

};

#endif
