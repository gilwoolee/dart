/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>,
 *            Tobias Kunz <tobias@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DART_COLLISION_COLLISIONNODE_H_
#define DART_COLLISION_COLLISIONNODE_H_

#include <cstddef>
#include <Eigen/Eigen>

namespace dart {
namespace dynamics {
class BodyNode;
class Shape;
}  // namespace dynamics
}  // namespace dart

namespace dart {
namespace collision {

///
class CollisionNode {
public:
  /// Default constructor
  explicit CollisionNode(dynamics::BodyNode* _bodyNode);

  /// Default destructor
  virtual ~CollisionNode();

  ///
  dynamics::BodyNode* getBodyNode() const;

  ///
  void setIndex(size_t _idx);

  ///
  size_t getIndex() const;

  const dynamics::Shape* getShape() const { return mShape; }
  const Eigen::Isometry3d& getTransform() const { return mTransform; }

protected:
  ///
  dynamics::BodyNode* mBodyNode;
  // TODO: Change to parent Frame

  ///
  size_t mIndex;

  const dynamics::Shape* mShape;
  const Eigen::Isometry3d mTransform;
};

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_COLLISIONNODE_H_
