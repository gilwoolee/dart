/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#ifndef  DART_COLLISION_DART_DARTCOLLISIONDETECTOR_H_
#define  DART_COLLISION_DART_DARTCOLLISIONDETECTOR_H_

#include <array>

#include "dart/common/Console.h"
#include "dart/common/Singletone.h"
#include "dart/collision/CollisionDetector.h"

namespace dart {
namespace collision {

using common::Singleton;

/// Geometry type
enum GeometryType
{
  GT_UNKNOWN = 0,
  GT_BOX,
  GT_SPHERE,
  GT_CAPSULE,
  GT_CONE,
  GT_CYLINDER,
  GT_CONVEX,
  GT_PLANE,
  GT_HALFSPACE,
  GT_TRIANGLE,
  GT_OCTREE,
  GT_COUNT
};

//==============================================================================
class CollisionObject
{
public:
  CollisionObject() {}
  ~CollisionObject() {}
private:
};

//==============================================================================
class CollisionGeometry
{
public:
  virtual ~CollisionGeometry() {}

  GeometryType getGeometryType() const { return mGeometryType; }

protected:
  CollisionGeometry() : mGeometryType(GT_UNKNOWN) {}

  GeometryType mGeometryType;
private:
};

//==============================================================================
class Box : public CollisionGeometry
{
public:
  Box()
    : CollisionGeometry(),
      size(Eigen::Vector3d(1.0, 1.0, 1.0))
  { mGeometryType = GT_BOX; }

  virtual ~Box() {}

  void setSize(const Eigen::Vector3d& _size) { size = _size; }

  const Eigen::Vector3d& getSize() const { return size; }

  Eigen::Vector3d size;
};

//==============================================================================
class Sphere : public CollisionGeometry
{
public:
  Sphere() : CollisionGeometry() { mGeometryType = GT_SPHERE; }
  virtual ~Sphere() {}
  double radius;
};

//==============================================================================
class Capsule : public CollisionGeometry
{
public:
  Capsule() : CollisionGeometry() { mGeometryType = GT_CAPSULE; }
  virtual ~Capsule() {}
  double radius;
  double height;
};

//==============================================================================
class Cone : public CollisionGeometry
{
public:
  Cone() : CollisionGeometry() { mGeometryType = GT_CONE; }
  virtual ~Cone() {}
  double radius;
  double height;
};

//==============================================================================
class Cylinder : public CollisionGeometry
{
public:
  Cylinder() : CollisionGeometry() { mGeometryType = GT_CYLINDER; }
  virtual ~Cylinder() {}
  double radius;
  double height;
};

//==============================================================================
class Convex : public CollisionGeometry
{
public:
  Convex() : CollisionGeometry() { mGeometryType = GT_CONVEX; }
  virtual ~Convex() {}

  Eigen::Vector3d* plane_normals;
  double* plane_dis;

  /// @brief An array of indices to the points of each polygon, it should be the number of vertices
  /// followed by that amount of indices to "points" in counter clockwise order
  int* polygons;

  Eigen::Vector3d* points;
  int num_points;
  int num_edges;
  int num_planes;

  struct Edge
  {
    int first, second;
  };

  Edge* edges;

  /// @brief center of the convex polytope, this is used for collision: center is guaranteed in the internal of the polytope (as it is convex)
  Eigen::Vector3d center;
};

//==============================================================================
class HalfSpace : public CollisionGeometry
{
public:
  HalfSpace() : CollisionGeometry() { mGeometryType = GT_HALFSPACE; }
  virtual ~HalfSpace() {}
};

//==============================================================================
class Plane : public CollisionGeometry
{
public:
  Plane() : CollisionGeometry() { mGeometryType = GT_PLANE; }
  virtual ~Plane() {}
};

//==============================================================================
class CollisionOptions
{
public:
  CollisionOptions() {}
  ~CollisionOptions() {}
private:
};

//==============================================================================
class CollisionResult
{
public:
  CollisionResult() {}
  ~CollisionResult() {}

  void addContact(const Contact& _contact)
  {
    mContacts.push_back(_contact);
  }

  const Contact& getContact(size_t _index)
  {
    return mContacts[_index];
  }

  void removeAllContacts()
  {
    mContacts.clear();
  }

  std::size_t getNumContacts() { return mContacts.size(); }
private:

  std::vector<Contact> mContacts;
};

//==============================================================================
using CollisionFunction = std::function<
    size_t(const CollisionGeometry* _geom1,
           const Eigen::Isometry3d& _tf1,
           const CollisionGeometry* _geom2,
           const Eigen::Isometry3d& _tf2,
           const CollisionOptions& _options,
           CollisionResult& _result)>;

//==============================================================================
///
class CollisionFunctionMatrix : public Singleton<CollisionFunctionMatrix>
{
public:

  friend class Singleton;

  const CollisionFunction& getCollisionFunction(GeometryType _geom1,
                                                GeometryType _geom2) const;

protected:

  CollisionFunctionMatrix();
  ~CollisionFunctionMatrix();

private:
  std::array<std::array<CollisionFunction, GT_COUNT>, GT_COUNT> mCollisionMatrix;
};

//==============================================================================
std::size_t collide(const CollisionGeometry* _geom1,
                    const Eigen::Isometry3d& _tf1,
                    const CollisionGeometry* _geom2,
                    const Eigen::Isometry3d& _tf2,
                    const CollisionOptions& _options,
                    CollisionResult& result);

//==============================================================================
class JSCollisionDetector
{
public:
  JSCollisionDetector() {}
  ~JSCollisionDetector() {}


private:
};

//==============================================================================
template<typename S1, typename S2>
std::size_t shapeIntersect(const CollisionGeometry* _geom1,
                           const Eigen::Isometry3d& _tf1,
                           const CollisionGeometry* _geom2,
                           const Eigen::Isometry3d& _tf2,
                           const CollisionOptions& _options,
                           CollisionResult& result)
{
  assert(_geom1 != nullptr);
  assert(_geom2 != nullptr);

  dtmsg << "Unsupported shape combination ["
        << _geom1->getGeometryType()
        << ", "
        << _geom2->getGeometryType()
        << "." << std::endl;

  return 0;
}

//==============================================================================
template <>
std::size_t shapeIntersect<Box, Box>(const CollisionGeometry* _geom1,
                                     const Eigen::Isometry3d& _tf1,
                                     const CollisionGeometry* _geom2,
                                     const Eigen::Isometry3d& _tf2,
                                     const CollisionOptions& _options,
                                     CollisionResult& _result);

//==============================================================================
template <>
std::size_t shapeIntersect<Sphere, Sphere>(const CollisionGeometry* _geom1,
                                           const Eigen::Isometry3d& _tf1,
                                           const CollisionGeometry* _geom2,
                                           const Eigen::Isometry3d& _tf2,
                                           const CollisionOptions& _options,
                                           CollisionResult& _result);

//==============================================================================
template <>
std::size_t shapeIntersect<Convex, Convex>(const CollisionGeometry* _geom1,
                                           const Eigen::Isometry3d& _tf1,
                                           const CollisionGeometry* _geom2,
                                           const Eigen::Isometry3d& _tf2,
                                           const CollisionOptions& _options,
                                           CollisionResult& _result);

//==============================================================================
//template <>
//std::size_t shapeIntersect<Box, Box>(const Box& _geom1,
//                                     const Eigen::Isometry3d& _tf1,
//                                     const Box& _geom2,
//                                     const Eigen::Isometry3d& _tf2,
//                                     const CollisionOptions& _options,
//                                     CollisionResult& result);

//==============================================================================
size_t collideBoxBox(const Box& _box1,
                     const Eigen::Isometry3d& _tf1,
                     const Box& _box2,
                     const Eigen::Isometry3d& _tf2,
                     const CollisionOptions& _options,
                     CollisionResult& _result);

//==============================================================================
/// \brief
class DARTCollisionDetector : public CollisionDetector
{
public:
  /// \brief Default constructor
  DARTCollisionDetector();

  /// \brief Default destructor
  virtual ~DARTCollisionDetector();

  // Documentation inherited
  virtual CollisionNode* createCollisionNode(dynamics::BodyNode* _bodyNode);

  // Documentation inherited
  virtual bool detectCollision(bool _checkAllCollisions,
                               bool _calculateContactPoints);

protected:
  // Documentation inherited
  virtual bool detectCollision(CollisionNode* _collNode1,
                               CollisionNode* _collNode2,
                               bool _calculateContactPoints);
};

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_DART_DARTCOLLISIONDETECTOR_H_
