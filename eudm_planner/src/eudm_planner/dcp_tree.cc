/**
 * @file behavior_tree.cc
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-07-07
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "eudm_planner/dcp_tree.h"

namespace planning {
DcpTree::DcpTree(const int& tree_height, const decimal_t& layer_time)
    : tree_height_(tree_height), layer_time_(layer_time) {
  GenerateActionScript();
}

ErrorType DcpTree::UpdateScript() { GenerateActionScript(); }

std::vector<DcpTree::BtAction> DcpTree::AppendActionSequence(
    const std::vector<BtAction>& seq_in, const BtAction& a,
    const int& n) const {
  std::vector<BtAction> seq = seq_in;
  for (int i = 0; i < n; ++i) {
    seq.push_back(a);
  }
  return seq;
}

ErrorType DcpTree::GenerateActionScript() {
  action_script_.clear();
  std::vector<BtAction> ongoing_action_seq;
  // int action_seq_cnt = 0;

  for (int lon = 0; lon < static_cast<int>(BtLonAction::MAX_COUNT); lon++) {
    ongoing_action_seq.clear();
    // if (sim_time_first_layer_ != layer_time_) {
    ongoing_action_seq.push_back(ongoing_action_);
    // } else {
    //   for (int lat = 0; lat < static_cast<int>(BtLatAction::MAX_COUNT);
    //   lat++) {
    //     if (lat != static_cast<int>(ongoing_action_.lat)) {
    //       auto actions =
    //           AppendActionSequence(ongoing_action_seq,
    //                                BtAction(BtLonAction(lon),
    //                                BtLatAction(lat),
    //                                         layer_time_),
    //                                tree_height_);
    //       action_script_.insert(
    //           std::pair<int, std::vector<BtAction>>(action_seq_cnt,
    //           actions));
    //       ++action_seq_cnt;
    //     }
    //   }
    //   ongoing_action_seq.push_back(BtAction(
    //       BtLonAction(lon), ongoing_action_.lat, layer_time_));
    // }

    for (int h = 1; h < tree_height_; ++h) {
      for (int lat = 0; lat < static_cast<int>(BtLatAction::MAX_COUNT); lat++) {
        if (lat != static_cast<int>(ongoing_action_.lat)) {
          auto actions = AppendActionSequence(
              ongoing_action_seq,
              BtAction(BtLonAction(lon), BtLatAction(lat), layer_time_),
              tree_height_ - h);
          action_script_.push_back(actions);
        }
      }
      ongoing_action_seq.push_back(
          BtAction(BtLonAction(lon), ongoing_action_.lat, layer_time_));
    }

    action_script_.push_back(ongoing_action_seq);
  }
  return kSuccess;
}
}  // namespace planning
