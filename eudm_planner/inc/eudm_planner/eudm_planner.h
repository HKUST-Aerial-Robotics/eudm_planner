#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_PLANNER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_PLANNER_H_

#include <algorithm>
#include <memory>
#include <string>
#include <thread>

#include "common/basics/basics.h"
#include "common/basics/ma_filter.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/state/state.h"
#include "eudm_planner/dcp_tree.h"
#include "eudm_planner/eudm_itf.h"
#include "eudm_planner/map_interface.h"
#include "forward_simulator/onlane_forward_simulation.h"

#include "eudm_config.pb.h"

namespace planning {

class EudmPlanner : public Planner {
 public:
  using State = common::State;
  using Lane = common::Lane;
  using Behavior = common::SemanticBehavior;
  using LateralBehavior = common::LateralBehavior;
  using LongitudinalBehavior = common::LongitudinalBehavior;
  using BtAction = DcpTree::BtAction;
  using BtLonAction = DcpTree::BtLonAction;
  using BtLatAction = DcpTree::BtLatAction;
  using Cfg = planning::eudm::Config;
  using LaneChangeInfo = planning::eudm::LaneChangeInfo;

  struct EfficiencyCost {
    decimal_t ego_to_desired_vel = 0.0;
    decimal_t leading_to_desired_vel = 0.0;
    decimal_t ave() const {
      return (ego_to_desired_vel + leading_to_desired_vel) / 2.0;
    }
  };

  struct SafetyCost {
    decimal_t rss = 0.0;
    decimal_t occu_lane = 0.0;
    decimal_t ave() const { return (rss + occu_lane) / 2.0; }
  };

  struct NavigationCost {
    decimal_t lane_change_preference = 0.0;
    decimal_t ave() const { return lane_change_preference; }
  };

  struct CostStructure {
    // * associate cost with micro action using this index
    int valid_sample_index_ub;
    // * efficiency
    EfficiencyCost efficiency;
    // * safety
    SafetyCost safety;
    // * navigation
    NavigationCost navigation;
    decimal_t weight = 1.0;
    decimal_t ave() const {
      return (efficiency.ave() + safety.ave() + navigation.ave()) * weight;
    }

    friend std::ostream& operator<<(std::ostream& os,
                                    const CostStructure& cost) {
      os << std::fixed;
      os << std::fixed;
      os << std::setprecision(3);
      os << "(efficiency: "
         << "ego (" << cost.efficiency.ego_to_desired_vel << ") + leading ("
         << cost.efficiency.leading_to_desired_vel << "), safety: ("
         << cost.safety.rss << "," << cost.safety.occu_lane
         << "), navigation: " << cost.navigation.lane_change_preference << ")";
      return os;
    }
  };

  struct PredictedVehicle {
    common::Vehicle vehicle;
    std::set<std::pair<decimal_t, LateralBehavior>> lat_intentions;

    PredictedVehicle() {}
    PredictedVehicle(
        const common::Vehicle& vehicle_,
        const std::set<std::pair<decimal_t, LateralBehavior>>& lat_intentions_)
        : vehicle(vehicle_), lat_intentions(lat_intentions_) {}
  };

  std::string Name() override;

  ErrorType Init(const std::string config) override;

  ErrorType RunOnce() override;

  void set_map_interface(EudmPlannerMapItf* itf);
  /**
   * @brief set desired velocity
   */
  void set_desired_velocity(const decimal_t desired_vel);

  void set_lane_change_info(const LaneChangeInfo& lc_info);

  ErrorType RunEudm();

  Behavior behavior() const;

  std::vector<BtAction> winner_action_seq() const { return winner_action_seq_; }

  decimal_t desired_velocity() const;

  vec_E<vec_E<common::Vehicle>> forward_trajs() const;

  int winner_id() const;

  std::vector<bool> sim_res() const {
    std::vector<bool> ret;
    for (auto& r : sim_res_) {
      if (r == 0) {
        ret.push_back(false);
      } else {
        ret.push_back(true);
      }
    }
    return ret;
  }

  std::vector<bool> risky_res() const {
    std::vector<bool> ret;
    for (auto& r : risky_res_) {
      if (r == 0) {
        ret.push_back(false);
      } else {
        ret.push_back(true);
      }
    }
    return ret;
  }
  std::vector<std::string> sim_info() const { return sim_info_; }
  std::vector<decimal_t> cost_val_res() const { return cost_val_res_; }
  std::vector<std::vector<CostStructure>> cost_structure_res() const {
    return cost_structure_res_;
  }
  std::vector<std::vector<LateralBehavior>> forward_lat_behaviors() const {
    return forward_lat_behaviors_;
  }
  std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors() const {
    return forward_lon_behaviors_;
  }
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs()
      const {
    return surround_trajs_;
  }
  common::State plan_state() {
    State ego_state;
    map_itf_->GetEgoState(&ego_state);
    return ego_state;
  }
  std::vector<std::vector<BtAction>> action_script() {
    return dcp_tree_ptr_->action_script();
  }

  std::unordered_map<int, PredictedVehicle> predicted_vehicles() const {
    return predicted_vehicles_;
  }

  const Cfg& cfg() const { return cfg_; }

  EudmPlannerMapItf* map_itf() const;

  void UpdateDcpTree(const BtAction& ongoing_action);

  ErrorType ClassifyActionSeq(const std::vector<BtAction>& action_seq,
                              decimal_t* operation_at_seconds,
                              common::LateralBehavior* lat_behavior,
                              bool* is_cancel_operation);

 protected:
  ErrorType ReadConfig(const std::string config_path);

  ErrorType GetSimParam(const planning::eudm::ForwardSimDetail& cfg,
                        OnLaneForwardSimulation::Param* sim_param);

  ErrorType GetPotentialLaneIds(const int source_lane_id,
                                const LateralBehavior& beh,
                                std::vector<int>* candidate_lane_ids) const;
  ErrorType UpdateEgoLaneId(const int new_ego_lane_id);

  ErrorType JudgeBehaviorByLaneId(const int ego_lane_id_by_pos,
                                  LateralBehavior* behavior_by_lane_id);

  ErrorType UpdateEgoBehavior(const LateralBehavior& behavior_by_lane_id);

  ErrorType TranslateBtActionToLonLatBehavior(const BtAction& action,
                                              LateralBehavior* lat,
                                              LongitudinalBehavior* lon) const;

  // * simulation control loop
  ErrorType InitIntentionBelief(
      const common::SemanticVehicleSet& semantic_key_vehicles,
      const common::Vehicle& ego_vehicle,
      const common::VehicleSet& surrounding_vehicles,
      std::unordered_map<int, PredictedVehicle>* predicted_vehicles);

  ErrorType SimulateActionSequence(
      const common::Vehicle& ego_vehicle,
      const std::unordered_map<int, PredictedVehicle>& predicted_vehicles,
      const std::vector<BtAction>& action_seq, const int& seq_id);

  ErrorType SimulateScenario(
      const common::Vehicle& ego_vehicle,
      const common::SemanticVehicleSet& semantic_vehicles,
      const std::vector<BtAction>& action_seq, const int& seq_id,
      const int& sub_seq_id, std::vector<int>* sub_sim_res,
      std::vector<int>* sub_risky_res, std::vector<std::string>* sub_sim_info,
      std::vector<std::vector<CostStructure>>* sub_cost_res,
      vec_E<vec_E<common::Vehicle>>* sub_forward_trajs,
      std::vector<std::vector<LateralBehavior>>* sub_forward_lat_behaviors,
      std::vector<std::vector<LongitudinalBehavior>>* sub_forward_lon_behaviors,
      vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>*
          sub_surround_trajs);

  ErrorType SimulateSingleAction(
      const BtAction& action, const decimal_t& ego_desired_vel,
      const std::unordered_map<int, double>& est_desired_vel_set,
      common::SemanticVehicle* ego_semantic_vehicle_this_layer,
      common::SemanticVehicleSet* semantic_vehicle_set_this_layer,
      vec_E<common::Vehicle>* ego_traj_multisteps,
      std::unordered_map<int, vec_E<common::Vehicle>>*
          surround_trajs_multisteps);

  ErrorType MultiAgentSimForward(
      const int ego_id, const decimal_t& ego_desired_vel,
      const common::SemanticVehicleSet& semantic_vehicle_set,
      const std::unordered_map<int, double>& est_desired_vel_set,
      const decimal_t& sim_time_resolution, const decimal_t& sim_time_total,
      vec_E<common::Vehicle>* traj,
      std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs);

  // * evaluation functions
  ErrorType CostFunction(
      const LongitudinalBehavior& ego_lon_behavior_this_layer,
      const LateralBehavior& ego_lat_behavior_this_layer,
      const decimal_t duration, const common::SemanticVehicle& ego_vehicle,
      const common::SemanticVehicleSet& agent_vehicles,
      const vec_E<common::Vehicle>& ego_traj,
      const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
      const common::LateralBehavior& seq_lat_behavior,
      const bool is_cancel_behavior, bool verbose, CostStructure* cost,
      bool* is_risky, std::set<int>* risky_ids);

  ErrorType StrictSafetyCheck(
      const vec_E<common::Vehicle>& ego_traj,
      const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
      bool* is_safe, int* collided_id);

  ErrorType EvaluateSafetyStatus(const vec_E<common::Vehicle>& traj_a,
                                 const vec_E<common::Vehicle>& traj_b,
                                 decimal_t* cost, bool* is_rss_safe,
                                 int* risky_id);
  ErrorType EvaluateSinglePolicyTrajs(
      const std::vector<CostStructure>& cost_res,
      const std::vector<BtAction>& action_seq, decimal_t* score);

  ErrorType EvaluateMultiThreadSimResults(int* winner_id,
                                          decimal_t* winner_cost);

  // * simulation util functions
  ErrorType UpdateSimulationSetup(
      const BtAction action, const common::State& ego_state_origin,
      LateralBehavior* ego_lat_behavior_this_layer,
      LongitudinalBehavior* ego_lon_behavior_this_layer,
      common::Lane* ego_ref_lane) const;

  bool CheckIfLateralActionFinished(const common::State& cur_state,
                                    const int& action_ref_lane_id,
                                    const LateralBehavior& lat_behavior,
                                    int* current_lane_id) const;

  ErrorType UpdateLateralActionSequence(
      const int cur_idx, std::vector<BtAction>* action_seq) const;

  ErrorType PrepareMultiThreadContainers(const int n_sequence);

  ErrorType GetSemanticVehiclesFromPredictedVehicles(
      const std::unordered_map<int, PredictedVehicle>& predicted_vehicles,
      common::SemanticVehicleSet* smv_set) const;

  // * map
  EudmPlannerMapItf* map_itf_{nullptr};
  // * action
  DcpTree* dcp_tree_ptr_;
  // * setup
  Cfg cfg_;
  LaneChangeInfo lc_info_;
  decimal_t desired_velocity_{5.0};
  decimal_t sim_time_total_ = 0.0;
  std::set<int> pre_deleted_seq_ids_;
  int ego_lane_id_{kInvalidLaneId};
  std::vector<int> potential_lcl_lane_ids_;
  std::vector<int> potential_lcr_lane_ids_;
  std::vector<int> potential_lk_lane_ids_;
  common::Lane rss_lane_;
  common::StateTransformer rss_stf_;
  // * result
  int winner_id_ = 0;
  decimal_t winner_score_ = 0.0;
  std::vector<BtAction> winner_action_seq_;
  std::vector<int> sim_res_;
  std::vector<int> risky_res_;
  std::vector<std::string> sim_info_;
  std::vector<decimal_t> cost_val_res_;
  std::vector<std::vector<CostStructure>> cost_structure_res_;
  vec_E<vec_E<common::Vehicle>> forward_trajs_;
  std::vector<std::vector<LateralBehavior>> forward_lat_behaviors_;
  std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors_;
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs_;
  // temporary
  common::VehicleSet nearby_vehicles_;
  std::unordered_map<int, PredictedVehicle> predicted_vehicles_;
};

}  // namespace planning

#endif  // _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_PLANNER_H_