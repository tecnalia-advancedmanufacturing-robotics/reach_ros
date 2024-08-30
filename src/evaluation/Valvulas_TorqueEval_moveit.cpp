/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <reach_ros/evaluation/Valvulas_TorqueEval_moveit.h>
#include <reach_ros/utils.h>

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/robot_model/joint_model_group.h>
#include <numeric>
#include <reach/plugin_utils.h>
#include <yaml-cpp/yaml.h>
#include <cmath>

static std::vector<Eigen::Index> getJacobianRowSubset(const YAML::Node& config, const std::string& key = "jacobian_row_"
                                                                                                         "subset")
{
  std::vector<Eigen::Index> jacobian_row_subset;

  const YAML::Node& jrs_config = config[key];
  if (jrs_config.IsDefined())
  {
    std::set<Eigen::Index> subset_rows;
    for (auto it = jrs_config.begin(); it != jrs_config.end(); ++it)
    {
      int row = (*it).as<Eigen::Index>();
      if (row < 0 || row >= 6)
      {
        std::stringstream ss;
        ss << "Invalid Jacobian row subset index provided: " << row << ". Must be on interval [0, 6)";
        throw std::runtime_error(ss.str());
      }

      subset_rows.insert(row);
    }

    if (subset_rows.empty())
      throw std::runtime_error("Jacobian row subset is empty");

    std::copy(subset_rows.begin(), subset_rows.end(), std::back_inserter(jacobian_row_subset));
  }
  else
  {
    jacobian_row_subset.resize(6);
    std::iota(jacobian_row_subset.begin(), jacobian_row_subset.end(), 0);
  }

  return jacobian_row_subset;
}

static std::vector<std::string> getExcludedLinks(const YAML::Node& config, const std::string& key = "excluded_links")
{
  try
  {
    return reach::get<std::vector<std::string>>(config, key);
  }
  catch (const std::exception& ex)
  {
    return {};
  }
}

namespace reach_ros
{
namespace evaluation
{
ValvulasTorqueEvalMoveit::ValvulasTorqueEvalMoveit(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                                           std::vector<Eigen::Index> jacobian_row_subset)
  : model_(std::move(model))
  , jmg_(model_->getJointModelGroup(planning_group))
  , jacobian_row_subset_(std::move(jacobian_row_subset))
{
  if (!jmg_)
    throw std::runtime_error("Failed to initialize joint model group pointer");
}

double ValvulasTorqueEvalMoveit::calculateScore(const std::map<std::string, double>& pose) const
{
  // Calculate manipulability of kinematic chain of input robot pose
  moveit::core::RobotState state(model_);

  // Take the subset of joints in the joint model group out of the input pose
  std::vector<double> pose_subset = utils::transcribeInputMap(pose, jmg_->getActiveJointModelNames());
  state.setJointGroupPositions(jmg_, pose_subset);
  state.update();

  // Get the Jacobian matrix
  Eigen::MatrixXd jacobian = state.getJacobian(jmg_);

  // Extract the partial jacobian
  if (jacobian_row_subset_.size() < 6)
  {
    Eigen::MatrixXd partial_jacobian(jacobian_row_subset_.size(), jacobian.cols());
    for (std::size_t i = 0; i < jacobian_row_subset_.size(); ++i)
    {
      partial_jacobian.row(i) = jacobian.row(jacobian_row_subset_[i]);
    }

    jacobian = partial_jacobian;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  Eigen::MatrixXd singular_values = svd.singularValues();




  return singular_values.array().prod();
  //return calculateScore(singular_values);
}

double ValvulasTorqueEvalMoveit::calculateScore(const Eigen::MatrixXd& jacobian_singular_values) const
{
  return jacobian_singular_values.array().prod();
}

reach::Evaluator::ConstPtr ValvulasTorqueEvalMoveitFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  std::vector<Eigen::Index> jacobian_row_subset = getJacobianRowSubset(config);

  utils::initROS();
  moveit::core::RobotModelConstPtr model = moveit::planning_interface::getSharedRobotModel("robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  return std::make_shared<ValvulasTorqueEvalMoveit>(model, planning_group, jacobian_row_subset);
}


}  // namespace evaluation
}  // namespace reach_ros
