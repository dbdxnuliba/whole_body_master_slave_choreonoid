#ifndef WholeBodyPositionController_Collision_H
#define WholeBodyPositionController_Collision_H

#include <cnoid/Body>
#include <string>
#include <collision_checker_msgs/idl/Collision.hh>

namespace WholeBodyPosition {

  class Collision {
  public:
    void updateFromIdl(const collision_checker_msgs::CollisionIdl& idl);
    const std::string& link1() const { return link1_;}
    const std::string& link2() const { return link2_;}
    const cnoid::Vector3& point1() const { return point1_;}
    const cnoid::Vector3& point2() const { return point2_;}
    const cnoid::Vector3& direction21() const { return direction21_;}
    const double& distance() const { return distance_;}
  protected:
    std::string link1_="";
    std::string link2_="";
    cnoid::Vector3 point1_ = cnoid::Vector3::Zero();
    cnoid::Vector3 point2_ = cnoid::Vector3::Zero();
    cnoid::Vector3 direction21_ = cnoid::Vector3::UnitX();
    double distance_=0;
  };

}

#endif
