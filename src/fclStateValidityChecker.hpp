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
      std::vector<fcl::CollisionObjectf *> obs_objs,
      std::vector<fcl::CollisionObjectf *> robot_objs,
      std::shared_ptr<Robot> robot)
      : StateValidityChecker(si)
      , obs_objs_(obs_objs)
      , robot_objs_(robot_objs) 
      , robot_(robot)
  {
  }

  bool isValid(const ompl::base::State* state) const override
  {
    if (!si_->satisfiesBounds(state)) {
      return false;
    }
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_robots(new fcl::DynamicAABBTreeCollisionManagerf());
    col_mng_robots->registerObjects(robot_objs_);
    col_mng_robots->setup();

    // std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_obs(new fcl::DynamicAABBTreeCollisionManagerf());
    fcl::BroadPhaseCollisionManagerf* col_mng_obs;
    col_mng_obs = new fcl::DynamicAABBTreeCollisionManagerf();
    col_mng_obs->registerObjects(obs_objs_);
    col_mng_obs->setup();

    for (size_t part = 0; part < robot_->numParts(); ++part) {
      const auto& transform = robot_->getTransform(state, part);
      fcl::CollisionObjectf robot(robot_->getCollisionGeometry(part)); 
      robot_objs_[part]->setTranslation(transform.translation());
      robot_objs_[part]->setRotation(transform.rotation());
      robot_objs_[part]->computeAABB();
    }
    col_mng_robots->update();
    // Robot-Obstacle
    fcl::DefaultCollisionData<float> collision_data_obstacles;
    col_mng_robots->collide(col_mng_obs, &collision_data_obstacles, fcl::DefaultCollisionFunction<float>);
    // Robot-Robot
    fcl::DefaultCollisionData<float> collision_data_robots;
    col_mng_robots->collide(&collision_data_robots, fcl::DefaultCollisionFunction<float>);

    if (collision_data_obstacles.result.isCollision() || collision_data_robots.result.isCollision()) {
      return false;
    }

    return true;
  }

private:
  std::vector<fcl::CollisionObjectf *> obs_objs_;
  std::vector<fcl::CollisionObjectf *> robot_objs_;
  std::shared_ptr<Robot> robot_;
};
