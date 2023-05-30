#pragma once

// #include "environment.h"
#include "robots.h"

#include <fcl/fcl.h>

class fclStateValidityChecker
  : public ompl::base::StateValidityChecker
{
public:
  fclStateValidityChecker(
      ompl::base::SpaceInformationPtr si,
      std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_environment,
      std::shared_ptr<Robot> robot,
      bool check_parts=false)
      : StateValidityChecker(si)
      , col_mng_environment_(col_mng_environment)
      , robot_(robot)
      , check_parts_(check_parts)
  {
    col_mng_parts_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
    for (size_t part = 0; part < robot_->numParts(); ++part){
      auto robot_part = new fcl::CollisionObjectf(robot_->getCollisionGeometry(part)); //, robot_->getTransform(state));
      robot_part->computeAABB();
      part_objs_.push_back(robot_part);
    }
    col_mng_parts_->registerObjects(part_objs_);
    col_mng_parts_->setup();
  }

  ~fclStateValidityChecker()
  {
    for (auto obj : part_objs_) {
      delete obj;
    }
  }

  bool isValid(const ompl::base::State* state) const override
  {
    if (!si_->satisfiesBounds(state)) {
      return false;
    }
    
    // part/environment checking
    for (size_t part = 0; part < robot_->numParts(); ++part) {
      const auto& transform = robot_->getTransform(state, part);
      auto robot = part_objs_[part];
      robot->setTranslation(transform.translation());
      robot->setRotation(transform.rotation());
      robot->computeAABB();
      fcl::DefaultCollisionData<float> collision_data;
      col_mng_environment_->collide(robot, &collision_data, fcl::DefaultCollisionFunction<float>);
      if (collision_data.result.isCollision()) {
        return false;
      }
    }

    // part/part checking
    if (check_parts_) {
      fcl::DefaultCollisionData<float> collision_data;
      col_mng_parts_->collide(&collision_data, fcl::DefaultCollisionFunction<float>);
      if (collision_data.result.isCollision()) {
        return false;
      } 
    }

    return true;
  }

private:
  std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_environment_;
  std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_parts_;
  std::vector<fcl::CollisionObjectf*> part_objs_;
  std::shared_ptr<Robot> robot_;
  bool check_parts_;
};
