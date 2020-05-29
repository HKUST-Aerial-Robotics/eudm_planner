/**
 * @file behavior_tree.h
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-07-07
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_TREE_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_TREE_H_

#include <map>
#include <memory>
#include <string>
#include "common/basics/basics.h"
#include "common/basics/semantics.h"

namespace planning {

class DcpTree {
 public:
  using LateralBehavior = common::LateralBehavior;

  enum class BtLonAction {
    kMaintain = 0,
    kAccelerate,
    kDecelerate,
    MAX_COUNT = 3
  };

  enum class BtLatAction {
    kLaneKeeping = 0,
    kLaneChangeLeft,
    kLaneChangeRight,
    MAX_COUNT = 3
  };

  struct BtAction {
    BtLonAction lon = BtLonAction::kMaintain;
    BtLatAction lat = BtLatAction::kLaneKeeping;

    decimal_t t = 0.0;

    friend std::ostream& operator<<(std::ostream& os, const BtAction& action) {
      os << "(lon: " << static_cast<int>(action.lon)
         << ", lat: " << static_cast<int>(action.lat) << ", t: " << action.t
         << ")";
      return os;
    }

    BtAction() {}
    BtAction(const BtLonAction& lon_, const BtLatAction& lat_,
             const decimal_t& t_)
        : lon(lon_), lat(lat_), t(t_) {}
  };

  DcpTree(const int& tree_height, const decimal_t& layer_time);
  ~DcpTree() = default;

  void set_ongoing_action(const BtAction& a) { ongoing_action_ = a; }

  std::vector<std::vector<BtAction>> action_script() const {
    return action_script_;
  }

  int tree_height() const { return tree_height_; }

  decimal_t sim_time_per_layer() const { return layer_time_; }

  ErrorType UpdateScript();

  static std::string RetLonActionName(const BtLonAction a) {
    std::string a_str;
    switch (a) {
      case BtLonAction::kMaintain: {
        a_str = std::string("M");
        break;
      }
      case BtLonAction::kAccelerate: {
        a_str = std::string("A");
        break;
      }
      case BtLonAction::kDecelerate: {
        a_str = std::string("D");
        break;
      }
      default: {
        a_str = std::string("Null");
        break;
      }
    }
    return a_str;
  }

  static std::string RetLatActionName(const BtLatAction a) {
    std::string a_str;
    switch (a) {
      case BtLatAction::kLaneKeeping: {
        a_str = std::string("K");
        break;
      }
      case BtLatAction::kLaneChangeLeft: {
        a_str = std::string("L");
        break;
      }
      case BtLatAction::kLaneChangeRight: {
        a_str = std::string("R");
        break;
      }
      default: {
        a_str = std::string("Null");
        break;
      }
    }
    return a_str;
  }

 private:
  ErrorType GenerateActionScript();

  std::vector<BtAction> AppendActionSequence(
      const std::vector<BtAction>& seq_in, const BtAction& a,
      const int& n) const;

  int tree_height_ = 5;
  const decimal_t layer_time_ = 1.0;

  BtAction ongoing_action_;
  std::vector<std::vector<BtAction>> action_script_;
};
}  // namespace planning

#endif  //  _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_TREE_H_