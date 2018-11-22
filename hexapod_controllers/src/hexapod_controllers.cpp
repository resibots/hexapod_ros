/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  Copyright (c) 2018, INRIA
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 *********************************************************************/

#include <hexapod_controllers/closed_loop_cpg_controller_velocity.hpp>
#include <hexapod_controllers/keep_body_level_velocity_controller.hpp>
#include <hexapod_controllers/open_loop_cpg_controller.hpp>
#include <hexapod_controllers/open_loop_cpg_controller_velocity.hpp>
#include <hexapod_controllers/simple_closed_loop_cpg_controller_velocity.hpp>
#include <hexapod_controllers/simple_position_controller.hpp>
#include <pluginlib/class_list_macros.h>

namespace open_loop_cpg_controller {
    typedef open_loop_cpg_controller::OlCpgController<NoSafetyConstraints, 6> HexapodOlCpgController;
} // namespace open_loop_cpg_controller

namespace open_loop_cpg_controller_velocity {
    typedef open_loop_cpg_controller_velocity::OlCpgControllerV<NoSafetyConstraints, 6> HexapodOlCpgControllerV;
}

namespace closed_loop_cpg_controller_velocity {
    typedef closed_loop_cpg_controller_velocity::ClCpgControllerV<NoSafetyConstraints, 6> HexapodClCpgControllerV;
}

namespace simple_closed_loop_cpg_controller_velocity {
    typedef simple_closed_loop_cpg_controller_velocity::SClCpgControllerV<NoSafetyConstraints, 6> HexapodSClCpgControllerV;
}
namespace simple_position_controller {
    typedef simple_position_controller::SimplePositionController<NoSafetyConstraints> HexapodSimplePositionController;
}

namespace keep_body_level_velocity_controller {
    typedef keep_body_level_velocity_controller::KBLControllerV<NoSafetyConstraints, 6> HexapodKBLControllerV;
}
PLUGINLIB_EXPORT_CLASS(open_loop_cpg_controller::HexapodOlCpgController,
    controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(open_loop_cpg_controller_velocity::HexapodOlCpgControllerV,
    controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(closed_loop_cpg_controller_velocity::HexapodClCpgControllerV,
    controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(simple_closed_loop_cpg_controller_velocity::HexapodSClCpgControllerV,
    controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(simple_position_controller::HexapodSimplePositionController,
    controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(keep_body_level_velocity_controller::HexapodKBLControllerV,
    controller_interface::ControllerBase)
