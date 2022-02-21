/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022,
 *  ETH Zurich - V4RL, Department of Mechanical and Process Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Matthias Busenhart
 *********************************************************************/

#ifndef __TPRM_OBSTACLE_IMPL_H__
#define __TPRM_OBSTACLE_IMPL_H__

#include <tprm/obstacle_base.h>

namespace tprm {

/**
 * @brief Static Sphere
 */
class StaticSphereObstacle : public StaticObstacle {
public:
    /**
     * @brief Constructor.
     */
    StaticSphereObstacle() = delete;

    /**
     * @brief Constructor.
     * @param COM Center of mass of the obstacle.
     * @param radius Radius of the sphere.
     */
    StaticSphereObstacle(const Vector3d& COM, double radius);

    /**
     * @brief Destructor.
     */
    virtual ~StaticSphereObstacle() = default;

    /**
     * @copydoc StaticObstacle::isColliding(const Vector3d&)
     */
    virtual bool isColliding(const Vector3d& point) const override;

    /**
     * @copydoc StaticObstacle::isColliding(const Vector3d&, const Vector3d&)
     */
    virtual bool isColliding(const Vector3d& segment_from, const Vector3d& segment_to) const override;

private:
    double m_radius;  ///< Radius of the sphere.
};

/**
 * @brief Dynamic Sphere
 */
class DynamicSphereObstacle : public DynamicObstacle {
public:
    /**
     * @brief Constructor.
     */
    DynamicSphereObstacle() = delete;

    /**
     * @brief Constructor.
     * @param COM Center of mass of the obstacle.
     * @param velocity Velocity of the obstacle.
     * @param radius Radius of the sphere.
     */
    DynamicSphereObstacle(const Vector3d& COM, const Vector3d& velocity, double radius);

    /**
     * @brief Destructor.
     */
    virtual ~DynamicSphereObstacle() = default;

    /**
     * @copydoc DynamicObstacle::isColliding(const Vector3d&, double&, double&)
     */
    virtual bool isColliding(const Vector3d& point, double& hitFromTime, double& hitToTime) const override;

private:
    double m_radius;  ///< Radius of the sphere.
};

} /* namespace tprm */

#endif /* __TPRM_OBSTACLE_IMPL_H__ */