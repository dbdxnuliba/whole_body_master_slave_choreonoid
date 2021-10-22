#include "Collision.h"

#include <cnoid/EigenUtil>
#include <iostream>

namespace WholeBodyTorque {
  void Collision::updateFromIdl(const collision_checker_msgs::CollisionIdl& idl) {
    this->link1_ = idl.link1;
    this->link2_ = idl.link2;
    this->point1_[0] = idl.point1.x;
    this->point1_[1] = idl.point1.y;
    this->point1_[2] = idl.point1.z;
    this->point2_[0] = idl.point2.x;
    this->point2_[1] = idl.point2.y;
    this->point2_[2] = idl.point2.z;
    this->direction21_[0] = idl.direction21.x;
    this->direction21_[1] = idl.direction21.y;
    this->direction21_[2] = idl.direction21.z;
    this->distance_ = idl.distance;
  }

};
