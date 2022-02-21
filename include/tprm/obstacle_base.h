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

#ifndef __TPRM_OBSTACLE_BASE_H__
#define __TPRM_OBSTACLE_BASE_H__

#include <tprm/config.h>

namespace tprm {

/**
 * @brief Base class for all obstacles.
 */
class Obstacle {
public:
    /**
     * @brief Constructor.
     */
    Obstacle() = delete;

    /**
     * @brief Constructor.
     * @param COM Center of mass of the obstacle.
     */
    Obstacle(const Vector3d& COM);

    /**
     * @brief Destructor.
     */
    virtual ~Obstacle() = default;

    /**
     * @brief Returns the center of mass of the obstacle.
     * @return Center of mass of the obstacle.
     */
    Vector3d getCOM() const;

    /**
     * @brief Sets the center of mass of the obstacle.
     * @param COM Center of mass of the obstacle.
     */
    void setCOM(const Vector3d& COM);

protected:
    Vector3d m_COM;  ///< center of mass
};

/**
 * @brief Class to represent a static obstacle.
 */
class StaticObstacle : public Obstacle {
public:
    /**
     * @brief Constructor.
     */
    StaticObstacle() = delete;

    /**
     * @brief Constructor.
     * @param COM Center of mass of the obstacle.
     */
    StaticObstacle(const Vector3d& COM);

    /**
     * @brief Destructor.
     */
    virtual ~StaticObstacle() = default;

    /**
     * @brief Checks if the obstacle collides with a given position.
     * @param position The position to check.
     * @return True if the obstacle collides with the position, false otherwise.
     */
    virtual bool isColliding(const Vector3d& position) const = 0;

    /**
     * @brief Checks if the obstacle collides with a line segment.
     * @param segment_from Start of the line segment.
     * @param segment_to End of the line segment.
     * @return True if the obstacle collides with the line segment, false otherwise.
     */
    virtual bool isColliding(const Vector3d& segment_from, const Vector3d& segment_to) const = 0;
};

/**
 * @brief Class to represent a dynamic obstacle.
 */
class DynamicObstacle : public Obstacle {
public:
    /**
     * @brief Constructor.
     */
    DynamicObstacle() = delete;

    /**
     * @brief Constructor.
     * @param COM Center of mass of the obstacle.
     * @param velocity Velocity of the obstacle.
     */
    DynamicObstacle(const Vector3d& COM, const Vector3d& velocity);

    /**
     * @brief Destructor.
     */
    virtual ~DynamicObstacle() = default;

    /**
     * @brief Returns the velocity of the obstacle.
     * @return Velocity of the obstacle.
     */
    Vector3d getVelocity() const;

    /**
     * @brief Sets the velocity of the obstacle.
     * @param velocity Velocity of the obstacle.
     */
    void setVelocity(const Vector3d& velocity);

    Vector3d getCOM() = delete;  ///< getting static COM is not allowed

    /**
     * @brief Get the center of mass of the dynamic obstacle for a given time.
     * @param time Time.
     * @return Center of mass of the dynamic obstacle for the given time.
     */
    virtual Vector3d getCOM(double time) const;

    /**
     * @brief Checks if the obstacle collides with a given position.
     * @param position The position to check.
     * @param[out] hitTimeFrom Time when the obstacle starts colliding with the position.
     * @param[out] hitTimeTo Time when the obstacle stops colliding with the position.
     * @return True if the obstacle collides with the position, false otherwise.
     */
    virtual bool isColliding(const Vector3d& position, double& hitTimeFrom, double& hitTimeTo) const = 0;

private:
    Vector3d m_velocity;  ///< velocity
};

} /* namespace tprm */

#endif /* __TPRM_OBSTACLE_BASE_H__ */