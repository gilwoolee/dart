/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
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

#ifndef DART_COLLISION_DART_DARTCOLLIDECCD_H_
#define DART_COLLISION_DART_DARTCOLLIDECCD_H_

#include <vector>

#include <Eigen/Dense>

#include <ccd/ccd.h>
#include <ccd/quat.h>

#include "dart/common/common.h"
#include "dart/math/math.h"
#include "dart/collision/dart/DARTCollisionDetector.h"

namespace dart {
namespace dynamics {
class Shape;
}  // namespace dynamics
}  // namespace dart

namespace dart {
namespace collision {

struct ccd_obj_t
{
  ccd_vec3_t pos;
  ccd_quat_t rot, rot_inv;
};

struct ccd_box_t : public ccd_obj_t
{
  ccd_real_t dim[3];
};

struct ccd_cap_t : public ccd_obj_t
{
  ccd_real_t radius, height;
};

struct ccd_cyl_t : public ccd_obj_t
{
  ccd_real_t radius, height;
};

struct ccd_cone_t : public ccd_obj_t
{
  ccd_real_t radius, height;
};

struct ccd_sphere_t : public ccd_obj_t
{
  ccd_real_t radius;
};

struct ccd_convex_t : public ccd_obj_t
{
  const Convex* convex;
};

struct ccd_triangle_t : public ccd_obj_t
{
  ccd_vec3_t p[3];
  ccd_vec3_t c;
};

using dGeomID = int;
using dReal = double;

typedef void (*GJKSupportFunction)(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v);
typedef void (*GJKCenterFunction)(const void* obj, ccd_vec3_t* c);

/** Transforms geom to ccd struct */
template <typename T>
void* createGJKObject(const T& /* s */, const Eigen::Isometry3d& /*tf*/)
{ return NULL; }

//==============================================================================
template <typename S>
GJKSupportFunction getSupportFunction() {}

//==============================================================================
template <typename S>
GJKCenterFunction getCenterFunction() {}

//==============================================================================
size_t collideSphereSphereLibccd(const Sphere& _geom1,
                                 const Eigen::Isometry3d& _tf1,
                                 const Sphere& _geom2,
                                 const Eigen::Isometry3d& _tf2,
                                 const CollisionOptions& _options,
                                 CollisionResult& _result);

//==============================================================================
size_t collideConvexConvexLibccd(const Convex& _geom1,
                                 const Eigen::Isometry3d& _tf1,
                                 const Convex& _geom2,
                                 const Eigen::Isometry3d& _tf2,
                                 const CollisionOptions& _options,
                                 CollisionResult& _result);

//==============================================================================
bool ccdCollide(void *obj1, ccd_support_fn supp1, ccd_center_fn cen1,
                void *obj2, ccd_support_fn supp2, ccd_center_fn cen2,
                size_t max_iteration = 500,
                double tolerance = 1e-6,
                Eigen::Vector3d* _point = nullptr,
                Eigen::Vector3d* _normal = nullptr,
                double* penetration = nullptr);

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_DART_DARTCOLLIDECCD_H_
