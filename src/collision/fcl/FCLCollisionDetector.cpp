/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/01/2013
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#include "kinematics/Shape.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Skeleton.h"

#include "collision/fcl/FCLCollisionNode.h"
#include "collision/fcl/FCLCollisionDetector.h"

namespace collision
{

FCLCollisionDetector::FCLCollisionDetector()
    : CollisionDetector(),
      mNumMaxContacts(100) {
}

FCLCollisionDetector::~FCLCollisionDetector()
{
}

CollisionNode* FCLCollisionDetector::createCollisionNode(
        kinematics::BodyNode* _bodyNode)
{
    CollisionNode* collisionNode = NULL;

    collisionNode = new FCLCollisionNode(_bodyNode);

    return collisionNode;
}

bool FCLCollisionDetector::checkCollision(bool _checkAllCollisions,
                                          bool _calculateContactPoints)
{
    // TODO: _checkAllCollisions
    clearAllContacts();

    fcl::CollisionResult result;

    // only evaluate contact points if data structure for returning the contact
    // points was provided
    fcl::CollisionRequest request;
    request.enable_contact = _calculateContactPoints;
    request.num_max_contacts = mNumMaxContacts;
//    request.enable_cost;
//    request.num_max_cost_sources;
//    request.use_approximate_cost;

    unsigned int numCollisionNodePairs = mCollisionNodePairs.size();
    FCLCollisionNode* collNode1 = NULL;
    FCLCollisionNode* collNode2 = NULL;

    for (unsigned int i = 0; i < numCollisionNodePairs; ++i)
    {
        const CollisionNodePair& collisionNodePair = mCollisionNodePairs[i];

        if (collisionNodePair.collidable == false)
            continue;

        collNode1 = dynamic_cast<FCLCollisionNode*>(collisionNodePair.collisionNode1);
        collNode2 = dynamic_cast<FCLCollisionNode*>(collisionNodePair.collisionNode2);

        for(int j = 0; j < collNode1->getNumCollisionGeometries(); j++)
        for(int k = 0; k < collNode2->getNumCollisionGeometries(); k++)
        {
            fcl::collide(collNode1->getCollisionGeometry(j),
                         collNode1->getFCLTransform(j),
                         collNode2->getCollisionGeometry(k),
                         collNode2->getFCLTransform(k),
                         request, result);

            unsigned int numContacts = result.numContacts();
            for (unsigned int l = 0; l < numContacts; ++l)
            {
                const fcl::Contact& contact = result.getContact(l);

                Contact contactPair;
                contactPair.point(0) = contact.pos[0];
                contactPair.point(1) = contact.pos[1];
                contactPair.point(2) = contact.pos[2];
                contactPair.normal(0) = contact.normal[0];
                contactPair.normal(1) = contact.normal[1];
                contactPair.normal(2) = contact.normal[2];
                contactPair.collisionNode1 = findCollisionNode(contact.o1);
                contactPair.collisionNode2 = findCollisionNode(contact.o2);
                assert(contactPair.collisionNode1 != NULL);
                assert(contactPair.collisionNode2 != NULL);
                //contactPair.bdID1 = collisionNodePair.collisionNode1->getBodyNodeID();
                //contactPair.bdID2 = collisionNodePair.collisionNode2->getBodyNodeID();
                contactPair.penetrationDepth = contact.penetration_depth;

                mContacts.push_back(contactPair);
            }
        }
    }

    return !mContacts.empty();
}

CollisionNode* FCLCollisionDetector::findCollisionNode(
        const fcl::CollisionGeometry* _fclCollGeom) const
{
    int numCollNodes = mCollisionNodes.size();
    for (int i = 0; i < numCollNodes; ++i)
    {
        FCLCollisionNode* collisionNode = dynamic_cast<FCLCollisionNode*>(mCollisionNodes[i]);
        for(int j = 0; j < collisionNode->getNumCollisionGeometries(); j++) {
            if (collisionNode->getCollisionGeometry(j) == _fclCollGeom)
                return mCollisionNodes[i];
        }
    }
    return NULL;
}

} // namespace collision
