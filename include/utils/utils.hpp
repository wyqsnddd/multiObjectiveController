/* Copyright 2018-2019 CNRS-UM LIRMM
 *
 * \author Yuquan Wang 
 *
 * 
 *
 * multiObjectiveController is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * pyQpController is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with multiObjectiveController. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#ifndef UTILS_HPP
#define UTILS_HPP

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <Eigen/Dense>

template<typename T>
std::vector<T> as_vector(boost::property_tree::ptree const & ptIn, boost::property_tree::ptree::key_type const & key)
{
  std::vector<T> r;
  for(auto & item : ptIn.get_child(key)) r.push_back(item.second.get_value<T>());
  return r;
}

inline Eigen::VectorXd & quatVector(const Eigen::Quaterniond & input)
{
  Eigen::VectorXd output(4);
  output(0) = input.w();
  output(1) = input.x();
  output(2) = input.y();
  output(3) = input.z();
  return output;
}

inline Eigen::Quaterniond & vecQuat(const Eigen::VectorXd & input)
{

  Eigen::Quaterniond output;
  output.w() = input(0);
  output.x() = input(1);
  output.y() = input(2);
  output.z() = input(3);

  return output;
}

#endif
