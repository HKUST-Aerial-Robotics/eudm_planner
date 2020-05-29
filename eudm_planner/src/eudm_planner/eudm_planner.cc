#include "eudm_planner/eudm_planner.h"

#include <glog/logging.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include "common/rss/rss_checker.h"

namespace planning {

std::string EudmPlanner::Name() { return std::string("Eudm behavior planner"); }

ErrorType EudmPlanner::ReadConfig(const std::string config_path) {
  printf("\n[EudmPlanner] Loading eudm planner config\n");
  using namespace google::protobuf;
  int fd = open(config_path.c_str(), O_RDONLY);
  io::FileInputStream fstream(fd);
  TextFormat::Parse(&fstream, &cfg_);
  if (!cfg_.IsInitialized()) {
    LOG(ERROR) << "failed to parse config from " << config_path;
    assert(false);
  }
  return kSuccess;
}

ErrorType EudmPlanner::GetSimParam(const planning::eudm::ForwardSimDetail& cfg,
                                   OnLaneForwardSimulation::Param* sim_param) {
  sim_param->idm_param.kMinimumSpacing = cfg.lon().idm().min_spacing();
  sim_param->idm_param.kDesiredHeadwayTime = cfg.lon().idm().head_time();
  sim_param->idm_param.kAcceleration = cfg.lon().limit().acc();
  sim_param->idm_param.kComfortableBrakingDeceleration =
      cfg.lon().limit().soft_brake();
  sim_param->idm_param.kHardBrakingDeceleration =
      cfg.lon().limit().hard_brake();
  sim_param->idm_param.kExponent = cfg.lon().idm().exponent();
  sim_param->max_lon_acc_jerk = cfg.lon().limit().acc_jerk();
  sim_param->max_lon_brake_jerk = cfg.lon().limit().brake_jerk();
  sim_param->steer_control_gain = cfg.lat().pure_pursuit().gain();
  sim_param->steer_control_max_lookahead_dist =
      cfg.lat().pure_pursuit().max_lookahead_dist();
  sim_param->steer_control_min_lookahead_dist =
      cfg.lat().pure_pursuit().min_lookahead_dist();
  sim_param->max_lat_acceleration_abs = cfg.lat().limit().acc();
  sim_param->max_lat_jerk_abs = cfg.lat().limit().jerk();
  sim_param->max_curvature_abs = cfg.lat().limit().curvature();
  sim_param->max_steer_angle_abs = cfg.lat().limit().steer_angle();
  sim_param->max_steer_rate = cfg.lat().limit().steer_rate();
  sim_param->auto_decelerate_if_lat_failed = cfg.auto_dec_if_lat_failed();
  return kSuccess;
}

ErrorType EudmPlanner::Init(const std::string config) {
  ReadConfig(config);

  dcp_tree_ptr_ = new DcpTree(cfg_.sim().duration().tree_height(),
                              cfg_.sim().duration().layer());
  LOG(INFO) << "[Eudm]Init.";
  LOG(INFO) << "[Eudm]ActionScript size: "
            << dcp_tree_ptr_->action_script().size() << std::endl;
  return kSuccess;
}

ErrorType EudmPlanner::TranslateBtActionToLonLatBehavior(
    const BtAction& action, LateralBehavior* lat,
    LongitudinalBehavior* lon) const {
  switch (action.lat) {
    case BtLatAction::kLaneKeeping: {
      *lat = LateralBehavior::kLaneKeeping;
      break;
    }
    case BtLatAction::kLaneChangeLeft: {
      *lat = LateralBehavior::kLaneChangeLeft;
      break;
    }
    case BtLatAction::kLaneChangeRight: {
      *lat = LateralBehavior::kLaneChangeRight;
      break;
    }
    default: {
      LOG(ERROR) << "[Eudm]Lateral action translation error!";
      return kWrongStatus;
    }
  }

  switch (action.lon) {
    case BtLonAction::kMaintain: {
      *lon = LongitudinalBehavior::kMaintain;
      break;
    }
    case BtLonAction::kAccelerate: {
      *lon = LongitudinalBehavior::kAccelerate;
      break;
    }
    case BtLonAction::kDecelerate: {
      *lon = LongitudinalBehavior::kDecelerate;
      break;
    }
    default: {
      LOG(ERROR) << "[Eudm]Longitudinal action translation error!";
      return kWrongStatus;
    }
  }

  return kSuccess;
}

ErrorType EudmPlanner::ClassifyActionSeq(
    const std::vector<BtAction>& action_seq, decimal_t* operation_at_seconds,
    common::LateralBehavior* lat_behavior, bool* is_cancel_operation) {
  decimal_t duration = 0.0;
  decimal_t operation_at = 0.0;
  bool find_lat_active_behavior = false;
  *is_cancel_operation = false;
  for (auto& action : action_seq) {
    if (!find_lat_active_behavior) {
      if (action.lat == BtLatAction::kLaneChangeLeft) {
        *operation_at_seconds = duration;
        *lat_behavior = common::LateralBehavior::kLaneChangeLeft;
        find_lat_active_behavior = true;
      }
      if (action.lat == BtLatAction::kLaneChangeRight) {
        *operation_at_seconds = duration;
        *lat_behavior = common::LateralBehavior::kLaneChangeRight;
        find_lat_active_behavior = true;
      }
    } else {
      if (action.lat == BtLatAction::kLaneKeeping) {
        *is_cancel_operation = true;
      }
    }
    duration += action.t;
  }
  if (!find_lat_active_behavior) {
    *operation_at_seconds = duration + cfg_.sim().duration().layer();
    *lat_behavior = common::LateralBehavior::kLaneKeeping;
    *is_cancel_operation = false;
  }
  return kSuccess;
}

ErrorType EudmPlanner::PrepareMultiThreadContainers(const int n_sequence) {
  sim_res_.clear();
  sim_res_.resize(n_sequence, 0);

  risky_res_.clear();
  risky_res_.resize(n_sequence, 0);

  sim_info_.clear();
  sim_info_.resize(n_sequence, std::string(""));

  cost_val_res_.clear();
  cost_val_res_.resize(n_sequence, 0.0);

  cost_structure_res_.clear();
  cost_structure_res_.resize(n_sequence);

  forward_trajs_.clear();
  forward_trajs_.resize(n_sequence);

  forward_lat_behaviors_.clear();
  forward_lat_behaviors_.resize(n_sequence);

  forward_lon_behaviors_.clear();
  forward_lon_behaviors_.resize(n_sequence);

  surround_trajs_.clear();
  surround_trajs_.resize(n_sequence);
  return kSuccess;
}

ErrorType EudmPlanner::InitIntentionBelief(
    const common::SemanticVehicleSet& semantic_key_vehicles,
    const common::Vehicle& ego_vehicle,
    const common::VehicleSet& surrounding_vehicles,
    std::unordered_map<int, PredictedVehicle>* predicted_vehicles) {
  predicted_vehicles->clear();
  std::vector<int> uncertain_vehicle_ids_from_smm;
  map_itf_->GetUncertainVehicleIds(&uncertain_vehicle_ids_from_smm);
  // printf("[Eudm]number of uncertain vehicles from smm: %d.\n",
  //        static_cast<int>(uncertain_vehicle_ids_from_smm.size()));
  for (auto it = semantic_key_vehicles.semantic_vehicles.begin();
       it != semantic_key_vehicles.semantic_vehicles.end(); ++it) {
    auto uncertain_id_it =
        std::find(uncertain_vehicle_ids_from_smm.begin(),
                  uncertain_vehicle_ids_from_smm.end(), it->first);
    if (uncertain_id_it != uncertain_vehicle_ids_from_smm.end()) {
      std::set<std::pair<decimal_t, LateralBehavior>> res;
      res.insert(std::pair<decimal_t, LateralBehavior>(
          0.33, LateralBehavior::kLaneKeeping));
      res.insert(std::pair<decimal_t, LateralBehavior>(
          0.33, LateralBehavior::kLaneChangeLeft));
      res.insert(std::pair<decimal_t, LateralBehavior>(
          0.33, LateralBehavior::kLaneChangeRight));
      PredictedVehicle p_v(it->second.vehicle, res);
      predicted_vehicles->insert(
          std::pair<int, PredictedVehicle>(it->second.vehicle.id(), p_v));
    } else if (it->second.lat_behavior == LateralBehavior::kLaneChangeLeft ||
               it->second.lat_behavior == LateralBehavior::kLaneChangeRight) {
      // ~ Already have convincing lane changing prediction
      std::set<std::pair<decimal_t, LateralBehavior>> res;
      if (it->second.lat_behavior == LateralBehavior::kLaneChangeLeft) {
        res.insert(std::pair<decimal_t, LateralBehavior>(
            0.0, LateralBehavior::kLaneKeeping));
        res.insert(std::pair<decimal_t, LateralBehavior>(
            1.0, LateralBehavior::kLaneChangeLeft));
        res.insert(std::pair<decimal_t, LateralBehavior>(
            0.0, LateralBehavior::kLaneChangeRight));
      } else if (it->second.lat_behavior == LateralBehavior::kLaneChangeRight) {
        res.insert(std::pair<decimal_t, LateralBehavior>(
            0.0, LateralBehavior::kLaneKeeping));
        res.insert(std::pair<decimal_t, LateralBehavior>(
            0.0, LateralBehavior::kLaneChangeLeft));
        res.insert(std::pair<decimal_t, LateralBehavior>(
            1.0, LateralBehavior::kLaneChangeRight));
      }
      PredictedVehicle p_v(it->second.vehicle, res);
      predicted_vehicles->insert(
          std::pair<int, PredictedVehicle>(it->second.vehicle.id(), p_v));
    } else {
      std::set<std::pair<decimal_t, LateralBehavior>> res;
      if (cfg_.function().mobil_enable()) {
        auto vehicle_set_tmp = surrounding_vehicles;
        auto it_v = vehicle_set_tmp.vehicles.find(it->second.vehicle.id());
        if (it_v != vehicle_set_tmp.vehicles.end()) {
          vehicle_set_tmp.vehicles.erase(it_v);
        }
        vehicle_set_tmp.vehicles.insert(
            std::pair<int, common::Vehicle>(ego_vehicle.id(), ego_vehicle));

        if (map_itf_->LateralBehaviorPredictionUsingMobil(
                it->second.vehicle, vehicle_set_tmp, &res) != kSuccess) {
          // ~ default to be lane keeping
          res.insert(std::pair<decimal_t, LateralBehavior>(
              1.0, LateralBehavior::kLaneKeeping));
          res.insert(std::pair<decimal_t, LateralBehavior>(
              0.0, LateralBehavior::kLaneChangeLeft));
          res.insert(std::pair<decimal_t, LateralBehavior>(
              0.0, LateralBehavior::kLaneChangeRight));
        }
      } else {
        res.insert(std::pair<decimal_t, LateralBehavior>(
            1.0, LateralBehavior::kLaneKeeping));
        res.insert(std::pair<decimal_t, LateralBehavior>(
            0.0, LateralBehavior::kLaneChangeLeft));
        res.insert(std::pair<decimal_t, LateralBehavior>(
            0.0, LateralBehavior::kLaneChangeRight));
      }
      PredictedVehicle p_v(it->second.vehicle, res);
      predicted_vehicles->insert(
          std::pair<int, PredictedVehicle>(it->second.vehicle.id(), p_v));
    }
  }
  return kSuccess;
}

ErrorType EudmPlanner::RunEudm() {
  // * get relevant information
  common::SemanticVehicleSet nearby_semantic_vehicles;
  if (map_itf_->GetKeySemanticVehicles(&nearby_semantic_vehicles) != kSuccess) {
    LOG(ERROR) << "[Eudm][Fatal]fail to get key semantic vehicles. Exit";
    return kWrongStatus;
  }
  nearby_vehicles_.vehicles.clear();
  if (map_itf_->GetKeyVehicles(&nearby_vehicles_) != kSuccess) {
    LOG(ERROR) << "[Eudm][Fatal]fail to get key vehicles. Exit";
    return kWrongStatus;
  }
  common::VehicleSet surrounding_vehicles;
  if (map_itf_->GetSurroundingVehicles(&surrounding_vehicles) != kSuccess) {
    LOG(ERROR) << "[Eudm][Fatal]fail to get surrounding vehicles. Exit";
    return kWrongStatus;
  }

  common::Vehicle ego_vehicle;
  if (map_itf_->GetEgoVehicle(&ego_vehicle) != kSuccess) {
    LOG(ERROR) << "[Eudm][Fatal]fail to get ego vehicle. Exit";
    return kWrongStatus;
  }

  // * Get initial vehicle intention prediction
  InitIntentionBelief(nearby_semantic_vehicles, ego_vehicle,
                      surrounding_vehicles, &predicted_vehicles_);
  {
    std::ostringstream line_info;
    line_info << "[Eudm][MOT]id:<bh,p>";
    for (auto& v : predicted_vehicles_) {
      line_info << "[" << v.first << ":";
      for (auto it = v.second.lat_intentions.begin();
           it != v.second.lat_intentions.end(); it++) {
        line_info << "<"
                  << common::SemanticsUtils::RetLatBehaviorName(it->second)
                  << "," << it->first << ">";
      }
      line_info << "]";
    }
    LOG(WARNING) << line_info.str();
  }

  // * construct <vehicle, ref_lane> pairs for nearby vehicles
  auto action_script = dcp_tree_ptr_->action_script();
  int n_sequence = action_script.size();

  // * prepare for multi-threading
  std::vector<std::thread> thread_set(n_sequence);
  PrepareMultiThreadContainers(n_sequence);
  LOG(INFO) << "[Eudm][Process]Prepare multi-threading - " << n_sequence
            << " threads.";

  // * threading
  TicToc timer;
  for (int i = 0; i < n_sequence; ++i) {
    thread_set[i] =
        std::thread(&EudmPlanner::SimulateActionSequence, this, ego_vehicle,
                    predicted_vehicles_, action_script[i], i);
  }
  for (int i = 0; i < n_sequence; ++i) {
    thread_set[i].join();
  }

  LOG(INFO) << "[Eudm][Process]Multi-thread forward simulation finished!";

  // * finish multi-threading, summary simulation results
  bool sim_success = false;
  int num_valid_behaviors = 0;
  for (int i = 0; i < static_cast<int>(sim_res_.size()); ++i) {
    if (sim_res_[i] == 1) {
      sim_success = true;
      num_valid_behaviors++;
    }
  }

  for (int i = 0; i < n_sequence; ++i) {
    std::ostringstream line_info;
    line_info << "[Eudm][Result]" << i << " [";
    for (auto& a : action_script[i]) {
      line_info << DcpTree::RetLonActionName(a.lon);
    }
    line_info << "|";
    for (auto& a : action_script[i]) {
      line_info << DcpTree::RetLatActionName(a.lat);
    }
    line_info << "]";
    line_info << "[s:" << sim_res_[i] << "|r:" << risky_res_[i]
              << "|c:" << std::fixed << std::setprecision(3) << cost_val_res_[i]
              << "]";
    line_info << " " << sim_info_[i] << "\n";
    if (sim_res_[i]) {
      line_info << "[e;s;n;w:";
      for (auto& c : cost_structure_res_[i]) {
        line_info << std::fixed << std::setprecision(2)
                  << c.efficiency.ego_to_desired_vel << "_"
                  << c.efficiency.leading_to_desired_vel << ";" << c.safety.rss
                  << "_" << c.safety.occu_lane << ";"
                  << c.navigation.lane_change_preference << ";" << c.weight;
        line_info << "|";
      }
      line_info << "]";
    }
    LOG(WARNING) << line_info.str();
  }
  LOG(WARNING) << "[Eudm][Result]Sim status: " << sim_success << " with "
               << num_valid_behaviors << " behaviors.";
  if (!sim_success) {
    LOG(ERROR) << "[Eudm][Fatal]Fail to find any valid behavior. Exit";
    return kWrongStatus;
  }

  // * evaluate
  if (EvaluateMultiThreadSimResults(&winner_id_, &winner_score_) != kSuccess) {
    LOG(ERROR)
        << "[Eudm][Fatal]fail to evaluate multi-thread sim results. Exit";
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType EudmPlanner::SimulateSingleAction(
    const BtAction& action, const decimal_t& ego_desired_vel,
    const std::unordered_map<int, double>& est_desired_vel_set,
    common::SemanticVehicle* ego_semantic_vehicle_this_layer,
    common::SemanticVehicleSet* semantic_vehicle_set_this_layer,
    vec_E<common::Vehicle>* ego_traj_multisteps,
    std::unordered_map<int, vec_E<common::Vehicle>>*
        surround_trajs_multisteps) {
  common::SemanticVehicleSet semantic_vehicle_set_with_ego =
      *semantic_vehicle_set_this_layer;
  semantic_vehicle_set_with_ego.semantic_vehicles.insert(
      std::make_pair(ego_semantic_vehicle_this_layer->vehicle.id(),
                     *ego_semantic_vehicle_this_layer));

  if (MultiAgentSimForward(ego_semantic_vehicle_this_layer->vehicle.id(),
                           ego_desired_vel, semantic_vehicle_set_with_ego,
                           est_desired_vel_set, cfg_.sim().duration().step(),
                           action.t, ego_traj_multisteps,
                           surround_trajs_multisteps) != kSuccess) {
    return kWrongStatus;
  }

  // * Step 4: update
  ego_semantic_vehicle_this_layer->vehicle.set_state(
      ego_traj_multisteps->back().state());
  for (auto it = semantic_vehicle_set_this_layer->semantic_vehicles.begin();
       it != semantic_vehicle_set_this_layer->semantic_vehicles.end(); ++it) {
    it->second.vehicle.set_state(
        surround_trajs_multisteps->at(it->first).back().state());
  }
  return kSuccess;
}

ErrorType EudmPlanner::UpdateSimulationSetup(
    const BtAction action, const common::State& ego_state_origin,
    LateralBehavior* ego_lat_behavior_this_layer,
    LongitudinalBehavior* ego_lon_behavior_this_layer,
    common::Lane* ego_ref_lane) const {
  if (TranslateBtActionToLonLatBehavior(action, ego_lat_behavior_this_layer,
                                        ego_lon_behavior_this_layer) !=
      kSuccess) {
    printf("[Eudm]Translate action error\n");
    return kWrongStatus;
  }

  // * construct ego semantic vehicle this layer
  // ~ note that the ref lane is constructed w.r.t. ego vehicle at the origin
  decimal_t forward_lane_len =
      std::min(std::max(ego_state_origin.velocity *
                            cfg_.sim().ref_line().len_vel_coeff(),
                        cfg_.sim().ref_line().forward_len_min()),
               cfg_.sim().ref_line().forward_len_max());
  if (map_itf_->GetRefLaneForStateByBehavior(
          ego_state_origin, std::vector<int>(), *ego_lat_behavior_this_layer,
          forward_lane_len, cfg_.sim().ref_line().backward_len_max(), false,
          ego_ref_lane) != kSuccess) {
    LOG(INFO) << "[Eudm]Get ref lane error at state: "
              << ego_state_origin.vec_position(0) << " "
              << ego_state_origin.vec_position(1) << " "
              << ego_state_origin.angle;
    return kWrongStatus;
  }

  return kSuccess;
}

ErrorType EudmPlanner::GetSemanticVehiclesFromPredictedVehicles(
    const std::unordered_map<int, PredictedVehicle>& predicted_vehicles,
    common::SemanticVehicleSet* smv_set) const {
  for (const auto& p_pv : predicted_vehicles) {
    int v_id = p_pv.first;

    common::Vehicle vehicle = predicted_vehicles.at(v_id).vehicle;
    LateralBehavior lat_behavior =
        predicted_vehicles.at(v_id).lat_intentions.rbegin()->second;

    common::SemanticVehicle sv;
    sv.vehicle = vehicle;
    sv.lat_behavior = lat_behavior;

    decimal_t forward_lane_len =
        std::min(std::max(vehicle.state().velocity *
                              cfg_.sim().ref_line().len_vel_coeff(),
                          cfg_.sim().ref_line().forward_len_min()),
                 cfg_.sim().ref_line().forward_len_max());

    common::Lane ref_lane;
    if (map_itf_->GetRefLaneForStateByBehavior(
            vehicle.state(), std::vector<int>(), lat_behavior, forward_lane_len,
            cfg_.sim().ref_line().backward_len_max(), false,
            &ref_lane) == kSuccess) {
      sv.lane = ref_lane;
    }
    smv_set->semantic_vehicles.insert(
        std::pair<int, common::SemanticVehicle>(v_id, sv));
  }

  return kSuccess;
}

ErrorType EudmPlanner::SimulateScenario(
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
        sub_surround_trajs) {
  std::vector<BtAction> action_seq_sim = action_seq;
  // * declare variables which will be updated per action layer
  common::SemanticVehicle ego_semantic_vehicle_this_layer;
  ego_semantic_vehicle_this_layer.vehicle = ego_vehicle;

  common::SemanticVehicleSet semantic_vehicle_set_this_layer =
      semantic_vehicles;

  // * declare variables which will be used to track traces from multiple layers
  vec_E<common::Vehicle> ego_traj_multilayers{ego_vehicle};
  std::unordered_map<int, vec_E<common::Vehicle>> surround_trajs_multilayers;
  for (const auto v : semantic_vehicle_set_this_layer.semantic_vehicles) {
    surround_trajs_multilayers.insert(std::pair<int, vec_E<common::Vehicle>>(
        v.first, vec_E<common::Vehicle>({v.second.vehicle})));
  }

  std::vector<LateralBehavior> ego_lat_behavior_multilayers;
  std::vector<LongitudinalBehavior> ego_lon_behavior_multilayers;
  std::vector<CostStructure> cost_multilayers;

  int action_ref_lane_id = ego_lane_id_;
  decimal_t ego_desired_vel = std::floor(ego_vehicle.state().velocity);

  if (action_seq[1].lon == BtLonAction::kAccelerate) {
    ego_desired_vel = std::min(ego_desired_vel + cfg_.sim().acc_cmd_vel_gap(),
                               desired_velocity_);
  } else if (action_seq[1].lon == BtLonAction::kMaintain) {
    ego_desired_vel = std::min(ego_desired_vel, desired_velocity_);
  } else {
    ego_desired_vel =
        std::min(std::max(ego_desired_vel - cfg_.sim().dec_cmd_vel_gap(), 0.0),
                 desired_velocity_);
  }

  std::unordered_map<int, double> est_desired_vel_set;
  for (const auto& v : semantic_vehicle_set_this_layer.semantic_vehicles) {
    auto state = v.second.vehicle.state();
    auto id = v.first;
    // ~ If other vehicles' acc > 0, we assume constant velocity
    if (state.acceleration >= 0) {
      est_desired_vel_set.insert(std::pair<int, double>(id, state.velocity));
    } else {
      double est_vel =
          std::max(0.0, state.velocity + state.acceleration * sim_time_total_);
      est_desired_vel_set.insert(std::pair<int, double>(id, est_vel));
    }
  }

  common::LateralBehavior seq_lat_behavior;
  decimal_t operation_at_seconds;
  bool is_cancel_behavior;
  ClassifyActionSeq(action_seq, &operation_at_seconds, &seq_lat_behavior,
                    &is_cancel_behavior);

  // * iterate through multi-layers
  bool is_sub_seq_risky = false;
  std::set<int> risky_ids;
  for (int i = 0; i < static_cast<int>(action_seq_sim.size()); ++i) {
    // * update setup
    LateralBehavior ego_lat_behavior_this_layer;
    LongitudinalBehavior ego_lon_behavior_this_layer;
    // OnLaneForwardSimulation::Param sim_param;
    // GetSimParam(cfg_.sim().ego(), &sim_param);
    if (UpdateSimulationSetup(
            action_seq_sim[i], ego_semantic_vehicle_this_layer.vehicle.state(),
            &ego_lat_behavior_this_layer, &ego_lon_behavior_this_layer,
            &ego_semantic_vehicle_this_layer.lane) != kSuccess) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Update setup F)");
      return kWrongStatus;
    }

    // * simulate this action (layer)
    vec_E<common::Vehicle> ego_traj_multisteps;
    std::unordered_map<int, vec_E<common::Vehicle>> surround_trajs_multisteps;
    if (SimulateSingleAction(
            action_seq_sim[i], ego_desired_vel, est_desired_vel_set,
            &ego_semantic_vehicle_this_layer, &semantic_vehicle_set_this_layer,
            &ego_traj_multisteps, &surround_trajs_multisteps) != kSuccess) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] +=
          std::string("(Sim ") + std::to_string(i) + std::string(" F)");
      return kWrongStatus;
    }

    // * enforce strict safety check
    bool is_strictly_safe = false;
    int collided_id = 0;
    TicToc timer;
    if (StrictSafetyCheck(ego_traj_multisteps, surround_trajs_multisteps,
                          &is_strictly_safe, &collided_id) != kSuccess) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Check F)");
      return kWrongStatus;
    }
    // LOG(INFO) << "[RssTime]safety check time per action: " << timer.toc();

    if (!is_strictly_safe) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Strict F:") +
                                     std::to_string(collided_id) +
                                     std::string(")");
      return kWrongStatus;
    }

    // * If lateral action finished, update simulation action sequence
    int current_lane_id;
    if (CheckIfLateralActionFinished(
            ego_semantic_vehicle_this_layer.vehicle.state(), action_ref_lane_id,
            ego_lat_behavior_this_layer, &current_lane_id)) {
      action_ref_lane_id = current_lane_id;
      if (kSuccess != UpdateLateralActionSequence(i, &action_seq_sim)) {
        (*sub_sim_res)[sub_seq_id] = 0;
        (*sub_sim_info)[sub_seq_id] += std::string("(Update Lat F)");
        return kWrongStatus;
      }
    }

    // * trace
    ego_traj_multilayers.insert(ego_traj_multilayers.end(),
                                ego_traj_multisteps.begin(),
                                ego_traj_multisteps.end());
    ego_lat_behavior_multilayers.push_back(ego_lat_behavior_this_layer);
    ego_lon_behavior_multilayers.push_back(ego_lon_behavior_this_layer);

    for (const auto v : semantic_vehicle_set_this_layer.semantic_vehicles) {
      int id = v.first;
      surround_trajs_multilayers.at(id).insert(
          surround_trajs_multilayers.at(id).end(),
          surround_trajs_multisteps.at(id).begin(),
          surround_trajs_multisteps.at(id).end());
    }

    CostStructure cost;
    bool verbose = false;
    bool is_risky_action = false;
    CostFunction(ego_lon_behavior_this_layer, ego_lat_behavior_this_layer,
                 action_seq_sim[i].t, ego_semantic_vehicle_this_layer,
                 semantic_vehicle_set_this_layer, ego_traj_multisteps,
                 surround_trajs_multisteps, seq_lat_behavior,
                 is_cancel_behavior, verbose, &cost, &is_risky_action,
                 &risky_ids);
    if (is_risky_action) {
      is_sub_seq_risky = true;
    }
    cost.weight = cost.weight * pow(cfg_.cost().disount_factor(), i);
    cost.valid_sample_index_ub = ego_traj_multilayers.size();
    cost_multilayers.push_back(cost);
  }

  (*sub_sim_res)[sub_seq_id] = 1;
  (*sub_risky_res)[sub_seq_id] = is_sub_seq_risky ? 1 : 0;
  if (is_sub_seq_risky) {
    std::string risky_id_list;
    for (auto it = risky_ids.begin(); it != risky_ids.end(); it++) {
      risky_id_list += " " + std::to_string(*it);
    }
    (*sub_sim_info)[sub_seq_id] += std::string("(Risky)") + risky_id_list;
  }
  (*sub_cost_res)[sub_seq_id] = cost_multilayers;
  (*sub_forward_trajs)[sub_seq_id] = ego_traj_multilayers;
  (*sub_forward_lat_behaviors)[sub_seq_id] = ego_lat_behavior_multilayers;
  (*sub_forward_lon_behaviors)[sub_seq_id] = ego_lon_behavior_multilayers;
  (*sub_surround_trajs)[sub_seq_id] = surround_trajs_multilayers;

  return kSuccess;
}

ErrorType EudmPlanner::SimulateActionSequence(
    const common::Vehicle& ego_vehicle,
    const std::unordered_map<int, PredictedVehicle>& predicted_vehicles,
    const std::vector<BtAction>& action_seq, const int& seq_id) {
  if (pre_deleted_seq_ids_.find(seq_id) != pre_deleted_seq_ids_.end()) {
    sim_res_[seq_id] = 0;
    sim_info_[seq_id] = std::string("(Pre-deleted)");
    return kWrongStatus;
  }

  // ~ For each ego sequence, we may further branch here, which will create
  // ~ multiple sub threads. Currently, we use n_sub_threads = 1
  int n_sub_threads = 1;
  common::SemanticVehicleSet semantic_vehicles;
  GetSemanticVehiclesFromPredictedVehicles(predicted_vehicles,
                                           &semantic_vehicles);
  std::vector<int> sub_sim_res(n_sub_threads);
  std::vector<int> sub_risky_res(n_sub_threads);
  std::vector<std::string> sub_sim_info(n_sub_threads);
  std::vector<std::vector<CostStructure>> sub_cost_res(n_sub_threads);
  vec_E<vec_E<common::Vehicle>> sub_forward_trajs(n_sub_threads);
  std::vector<std::vector<LateralBehavior>> sub_forward_lat_behaviors(
      n_sub_threads);
  std::vector<std::vector<LongitudinalBehavior>> sub_forward_lon_behaviors(
      n_sub_threads);
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> sub_surround_trajs(
      n_sub_threads);

  SimulateScenario(ego_vehicle, semantic_vehicles, action_seq, seq_id, 0,
                      &sub_sim_res, &sub_risky_res, &sub_sim_info,
                      &sub_cost_res, &sub_forward_trajs,
                      &sub_forward_lat_behaviors, &sub_forward_lon_behaviors,
                      &sub_surround_trajs);

  if (sub_sim_res.front() == 0) {
    sim_res_[seq_id] = 0;
    sim_info_[seq_id] = sub_sim_info.front();
    return kWrongStatus;
  }

  decimal_t cost_val = 0.0;
  EvaluateSinglePolicyTrajs(sub_cost_res.front(), action_seq, &cost_val);

  sim_res_[seq_id] = 1;
  risky_res_[seq_id] = sub_risky_res.front();
  sim_info_[seq_id] = sub_sim_info.front();
  cost_val_res_[seq_id] = cost_val;
  cost_structure_res_[seq_id] = sub_cost_res.front();
  forward_trajs_[seq_id] = sub_forward_trajs.front();
  forward_lat_behaviors_[seq_id] = sub_forward_lat_behaviors.front();
  forward_lon_behaviors_[seq_id] = sub_forward_lon_behaviors.front();
  surround_trajs_[seq_id] = sub_surround_trajs.front();

  return kSuccess;
}

ErrorType EudmPlanner::UpdateLateralActionSequence(
    const int cur_idx, std::vector<BtAction>* action_seq) const {
  if (cur_idx == static_cast<int>(action_seq->size()) - 1) {
    return kSuccess;
  }

  switch ((*action_seq)[cur_idx].lat) {
    case BtLatAction::kLaneKeeping: {
      // * no need to update
      break;
    }
    case BtLatAction::kLaneChangeLeft: {
      for (int i = cur_idx + 1; i < static_cast<int>(action_seq->size()); ++i) {
        if ((*action_seq)[i].lat == BtLatAction::kLaneChangeLeft) {
          // * LLLLL -> LLKKK
          (*action_seq)[i].lat = BtLatAction::kLaneKeeping;
        } else if ((*action_seq)[i].lat == BtLatAction::kLaneKeeping) {
          // * LLKKK -> LLRRR
          (*action_seq)[i].lat = BtLatAction::kLaneChangeRight;
        } else if ((*action_seq)[i].lat == BtLatAction::kLaneChangeRight) {
          // * LLRRR -> x
          return kWrongStatus;
        }
      }
      break;
    }
    case BtLatAction::kLaneChangeRight: {
      for (int i = cur_idx + 1; i < static_cast<int>(action_seq->size()); ++i) {
        if ((*action_seq)[i].lat == BtLatAction::kLaneChangeRight) {
          // * RRRRR -> RRKKK
          (*action_seq)[i].lat = BtLatAction::kLaneKeeping;
        } else if ((*action_seq)[i].lat == BtLatAction::kLaneKeeping) {
          // * RRKKK -> RRLLL
          (*action_seq)[i].lat = BtLatAction::kLaneChangeLeft;
        } else if ((*action_seq)[i].lat == BtLatAction::kLaneChangeLeft) {
          // * RRLLL -> x
          return kWrongStatus;
        }
      }
      break;
    }

    default: {
      std::cout << "[Eudm]Error - Invalid lateral behavior" << std::endl;
      assert(false);
    }
  }

  return kSuccess;
}

bool EudmPlanner::CheckIfLateralActionFinished(
    const common::State& cur_state, const int& action_ref_lane_id,
    const LateralBehavior& lat_behavior, int* current_lane_id) const {
  decimal_t current_lane_dist;
  decimal_t arc_len;
  map_itf_->GetNearestLaneIdUsingState(cur_state.ToXYTheta(),
                                       std::vector<int>(), current_lane_id,
                                       &current_lane_dist, &arc_len);
  // std::cout << "[Eudm] current_lane_id: " << *current_lane_id << std::endl;
  // std::cout << "[Eudm] action_ref_lane_id: " << action_ref_lane_id <<
  // std::endl;

  std::vector<int> potential_lane_ids;
  GetPotentialLaneIds(action_ref_lane_id, lat_behavior, &potential_lane_ids);

  if (lat_behavior == LateralBehavior::kLaneKeeping) {
    return true;
  }

  // ~ Lane change
  auto it = std::find(potential_lane_ids.begin(), potential_lane_ids.end(),
                      *current_lane_id);
  if (it == potential_lane_ids.end()) {
    return false;
  } else {
    return true;
  }
}

ErrorType EudmPlanner::RunOnce() {
  // * Get current nearest lane id
  if (!map_itf_) {
    LOG(ERROR) << "[Eudm]map interface not initialized. Exit";
    return kWrongStatus;
  }

  common::Vehicle ego_vehicle;
  if (map_itf_->GetEgoVehicle(&ego_vehicle) != kSuccess) {
    LOG(ERROR) << "[Eudm]no ego vehicle found.";
    return kWrongStatus;
  }
  decimal_t time_stamp = ego_vehicle.state().time_stamp;
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Eudm]------ Eudm Cycle Begins (stamp): " << time_stamp
               << " ------- ";

  int ego_lane_id_by_pos = kInvalidLaneId;
  if (map_itf_->GetEgoLaneIdByPosition(std::vector<int>(),
                                       &ego_lane_id_by_pos) != kSuccess) {
    LOG(ERROR) << "[Eudm]Fatal (Exit) ego not on lane.";
    return kWrongStatus;
  }
  LOG(WARNING) << std::fixed << std::setprecision(3)
               << "[Eudm][Input]Ego plan state (x,y,theta,v,a,k):("
               << ego_vehicle.state().vec_position[0] << ","
               << ego_vehicle.state().vec_position[1] << ","
               << ego_vehicle.state().angle << ","
               << ego_vehicle.state().velocity << ","
               << ego_vehicle.state().acceleration << ","
               << ego_vehicle.state().curvature << ")"
               << " lane id:" << ego_lane_id_by_pos;
  LOG(WARNING) << "[Eudm][Setup]Desired vel:" << desired_velocity_
               << " sim_time total:" << sim_time_total_
               << " lc info[f_l,f_r,us_ol,us_or,solid_l,solid_r]:"
               << lc_info_.forbid_lane_change_left << ","
               << lc_info_.forbid_lane_change_right << ","
               << lc_info_.lane_change_left_unsafe_by_occu << ","
               << lc_info_.lane_change_right_unsafe_by_occu << ","
               << lc_info_.left_solid_lane << "," << lc_info_.right_solid_lane;
  ego_lane_id_ = ego_lane_id_by_pos;

  const decimal_t forward_rss_check_range = 130.0;
  const decimal_t backward_rss_check_range = 130.0;
  const decimal_t forward_lane_len = forward_rss_check_range;
  const decimal_t backward_lane_len = backward_rss_check_range;
  if (map_itf_->GetRefLaneForStateByBehavior(
          ego_vehicle.state(), std::vector<int>(),
          LateralBehavior::kLaneKeeping, forward_lane_len, backward_lane_len,
          false, &rss_lane_) != kSuccess) {
    LOG(ERROR) << "[Eudm]No Rss lane available. Rss disabled";
  }

  if (rss_lane_.IsValid()) {
    rss_stf_ = common::StateTransformer(rss_lane_);
  }

  // ! Put lane change complete check in eudm_manager, notice that currently it
  // ! can only check simple lane structure
  // * Compare to last potential_lane_ids to get the instantaneous behavior
  // LateralBehavior behavior_by_lane_id;
  // if (JudgeBehaviorByLaneId(ego_lane_id_by_pos, &behavior_by_lane_id) !=
  //     kSuccess) {
  //   LOG(ERROR) << "[Eudm]fail to judge current behavior by lane id! Exit";
  //   return kWrongStatus;
  // }
  // LOG(WARNING) << "[Eudm]behavior_by_lane_id: "
  //              << static_cast<int>(behavior_by_lane_id);

  pre_deleted_seq_ids_.clear();
  int n_sequence = dcp_tree_ptr_->action_script().size();
  for (int i = 0; i < n_sequence; i++) {
    auto action_seq = dcp_tree_ptr_->action_script()[i];
    int num_actions = action_seq.size();
    for (int j = 1; j < num_actions; j++) {
      if ((action_seq[j - 1].lat == BtLatAction::kLaneChangeLeft &&
           action_seq[j].lat == BtLatAction::kLaneChangeRight) ||
          (action_seq[j - 1].lat == BtLatAction::kLaneChangeRight &&
           action_seq[j].lat == BtLatAction::kLaneChangeLeft)) {
        pre_deleted_seq_ids_.insert(i);
      }
    }
  }

  TicToc timer;
  if (RunEudm() != kSuccess) {
    LOG(ERROR) << std::fixed << std::setprecision(4)
               << "[Eudm]****** Eudm Cycle FAILED (stamp): " << time_stamp
               << " time cost " << timer.toc() << " ms.";
    return kWrongStatus;
  }
  auto action_script = dcp_tree_ptr_->action_script();
  std::ostringstream line_info;
  line_info << "[Eudm]SUCCESS id:" << winner_id_ << " [";
  for (auto& a : action_script[winner_id_]) {
    line_info << DcpTree::RetLonActionName(a.lon);
  }
  line_info << "|";
  for (auto& a : action_script[winner_id_]) {
    line_info << DcpTree::RetLatActionName(a.lat);
  }
  line_info << "] cost: " << std::fixed << std::setprecision(3) << winner_score_
            << " time cost: " << timer.toc() << " ms.";
  LOG(WARNING) << line_info.str();
  return kSuccess;
}

ErrorType EudmPlanner::EvaluateSinglePolicyTrajs(
    const std::vector<CostStructure>& cost_res,
    const std::vector<BtAction>& action_seq, decimal_t* score) {
  decimal_t score_tmp = 0.0;
  for (auto& c : cost_res) {
    score_tmp += c.ave();
  }
  *score = score_tmp;
  return kSuccess;
}

ErrorType EudmPlanner::EvaluateMultiThreadSimResults(int* winner_id,
                                                     decimal_t* winner_cost) {
  decimal_t min_cost = kInf;
  int best_id = 0;
  int num_sequences = sim_res_.size();
  for (int i = 0; i < num_sequences; ++i) {
    if (sim_res_[i] == 0) {
      continue;
    }

    decimal_t cost = cost_val_res_[i];
    if (cost < min_cost) {
      min_cost = cost;
      best_id = i;
    }
  }
  *winner_cost = min_cost;
  *winner_id = best_id;
  return kSuccess;
}

ErrorType EudmPlanner::EvaluateSafetyStatus(
    const vec_E<common::Vehicle>& traj_a, const vec_E<common::Vehicle>& traj_b,
    decimal_t* cost, bool* is_rss_safe, int* risky_id) {
  if (traj_a.size() != traj_b.size()) {
    return kWrongStatus;
  }
  if (!cfg_.safety().rss_check_enable() || !rss_lane_.IsValid()) {
    return kSuccess;
  }
  int num_states = static_cast<int>(traj_a.size());
  decimal_t cost_tmp = 0.0;
  bool ret_is_rss_safe = true;
  const int check_per_state = 1;
  for (int i = 0; i < num_states; i += check_per_state) {
    bool is_rss_safe = true;
    common::RssChecker::LongitudinalViolateType type;
    decimal_t rss_vel_low, rss_vel_up;
    common::RssChecker::RssCheck(
        traj_a[i], traj_b[i], rss_stf_,
        common::RssChecker::RssConfig(
            cfg_.safety().rss().response_time(),
            cfg_.safety().rss().longitudinal_acc_max(),
            cfg_.safety().rss().longitudinal_brake_min(),
            cfg_.safety().rss().longitudinal_brake_max(),
            cfg_.safety().rss().lateral_acc_max(),
            cfg_.safety().rss().lateral_brake_min(),
            cfg_.safety().rss().lateral_brake_max(),
            cfg_.safety().rss().lateral_miu()),
        &is_rss_safe, &type, &rss_vel_low, &rss_vel_up);
    if (!is_rss_safe) {
      ret_is_rss_safe = false;
      *risky_id = traj_b.size() ? traj_b[0].id() : 0;
      if (cfg_.cost().safety().rss_cost_enable()) {
        if (type == common::RssChecker::LongitudinalViolateType::TooFast) {
          cost_tmp +=
              pow(10, cfg_.cost().safety().rss_over_speed_power_coeff() *
                          fabs(traj_a[i].state().velocity - rss_vel_up));
        } else if (type ==
                   common::RssChecker::LongitudinalViolateType::TooSlow) {
          cost_tmp +=
              pow(10, cfg_.cost().safety().rss_lack_speed_power_coeff() *
                          fabs(traj_a[i].state().velocity - rss_vel_low));
        }
      }
    }
  }
  *is_rss_safe = ret_is_rss_safe;
  *cost = cost_tmp;
  return kSuccess;
}

ErrorType EudmPlanner::StrictSafetyCheck(
    const vec_E<common::Vehicle>& ego_traj,
    const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
    bool* is_safe, int* collided_id) {
  if (!cfg_.safety().strict_check_enable()) {
    *is_safe = true;
    return kSuccess;
  }

  int num_points_ego = ego_traj.size();
  if (num_points_ego == 0) {
    *is_safe = true;
    return kSuccess;
  }
  // strict collision check
  for (auto it = surround_trajs.begin(); it != surround_trajs.end(); it++) {
    int num_points_other = it->second.size();
    if (num_points_other != num_points_ego) {
      *is_safe = false;
      LOG(ERROR) << "[Eudm]unsafe due to incomplete sim record for vehicle";
      return kSuccess;
    }
    for (int i = 0; i < num_points_ego; i++) {
      common::Vehicle inflated_a, inflated_b;
      common::SemanticsUtils::InflateVehicleBySize(
          ego_traj[i], cfg_.safety().strict().inflation_w(),
          cfg_.safety().strict().inflation_h(), &inflated_a);
      common::SemanticsUtils::InflateVehicleBySize(
          it->second[i], cfg_.safety().strict().inflation_w(),
          cfg_.safety().strict().inflation_h(), &inflated_b);
      bool is_collision = false;
      map_itf_->CheckCollisionUsingState(inflated_a.param(), inflated_a.state(),
                                         inflated_b.param(), inflated_b.state(),
                                         &is_collision);
      if (is_collision) {
        *is_safe = false;
        *collided_id = it->second[i].id();
        return kSuccess;
      }
    }
  }
  *is_safe = true;
  return kSuccess;
}

ErrorType EudmPlanner::CostFunction(
    const LongitudinalBehavior& ego_lon_behavior_this_layer,
    const LateralBehavior& ego_lat_behavior_this_layer,
    const decimal_t duration, const common::SemanticVehicle& ego_vehicle,
    const common::SemanticVehicleSet& agent_vehicles,
    const vec_E<common::Vehicle>& ego_traj,
    const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
    const common::LateralBehavior& seq_lat_behavior,
    const bool is_cancel_behavior, bool verbose, CostStructure* cost,
    bool* is_risky, std::set<int>* risky_ids) {
  common::VehicleSet vehicle_set;
  for (auto& v : agent_vehicles.semantic_vehicles) {
    vehicle_set.vehicles.insert(std::make_pair(v.first, v.second.vehicle));
  }

  // f = c1 * fabs(v_ego - v_user), if v_ego < v_user
  // f = c2 * fabs(v_ego - v_user - vth), if v_ego > v_user + vth
  // unit of this cost is velocity (finally multiplied by duration)
  CostStructure cost_tmp;
  if (ego_vehicle.vehicle.state().velocity < desired_velocity_) {
    cost_tmp.efficiency.ego_to_desired_vel =
        cfg_.cost().effciency().ego_lack_speed_to_desired_unit_cost() *
        fabs(ego_vehicle.vehicle.state().velocity - desired_velocity_);
  } else {
    if (ego_vehicle.vehicle.state().velocity >
        desired_velocity_ +
            cfg_.cost().effciency().ego_desired_speed_tolerate_gap()) {
      cost_tmp.efficiency.ego_to_desired_vel =
          cfg_.cost().effciency().ego_over_speed_to_desired_unit_cost() *
          fabs(ego_vehicle.vehicle.state().velocity - desired_velocity_ -
               cfg_.cost().effciency().ego_desired_speed_tolerate_gap());
    }
  }

  // f = ratio * c1 * fabs(v_ego - v_user) , if v_ego < v_user && v_ego
  // > v_leading
  // unit of this cost is velocity (finally multiplied by duration)
  common::Vehicle leading_vehicle;
  decimal_t distance_residual_ratio = 0.0;
  decimal_t lat_range = cfg_.sim().ego().cooperative_lat_range();
  if (map_itf_->GetLeadingVehicleOnLane(
          ego_vehicle.lane, ego_vehicle.vehicle.state(), vehicle_set, lat_range,
          &leading_vehicle, &distance_residual_ratio) == kSuccess) {
    decimal_t distance_to_leading_vehicle =
        (leading_vehicle.state().vec_position -
         ego_vehicle.vehicle.state().vec_position)
            .norm();
    if (ego_vehicle.vehicle.state().velocity < desired_velocity_ &&
        leading_vehicle.state().velocity < desired_velocity_ &&
        distance_to_leading_vehicle <
            cfg_.cost().effciency().leading_distance_th()) {
      decimal_t ego_blocked_by_leading_velocity =
          ego_vehicle.vehicle.state().velocity >
                  leading_vehicle.state().velocity
              ? ego_vehicle.vehicle.state().velocity -
                    leading_vehicle.state().velocity
              : 0.0;
      decimal_t leading_to_desired_velocity =
          leading_vehicle.state().velocity < desired_velocity_
              ? desired_velocity_ - leading_vehicle.state().velocity
              : 0.0;
      cost_tmp.efficiency.leading_to_desired_vel =
          distance_residual_ratio *
          (cfg_.cost().effciency().ego_speed_blocked_by_leading_unit_cost() *
               ego_blocked_by_leading_velocity +
           cfg_.cost()
                   .effciency()
                   .leading_speed_blocked_desired_vel_unit_cost() *
               leading_to_desired_velocity);
    }
  }

  // * safety
  for (auto& surround_traj : surround_trajs) {
    decimal_t safety_cost = 0.0;
    bool is_safe = true;
    int risky_id = 0;
    EvaluateSafetyStatus(ego_traj, surround_traj.second, &safety_cost, &is_safe,
                         &risky_id);
    if (!is_safe) {
      risky_ids->insert(risky_id);
      *is_risky = true;
    }
    cost_tmp.safety.rss += safety_cost;
  }

  if (cfg_.cost().safety().occu_lane_enable()) {
    if (lc_info_.forbid_lane_change_left &&
        seq_lat_behavior == LateralBehavior::kLaneChangeLeft) {
      cost_tmp.safety.occu_lane =
          desired_velocity_ * cfg_.cost().safety().occu_lane_unit_cost();
    } else if (lc_info_.forbid_lane_change_right &&
               seq_lat_behavior == LateralBehavior::kLaneChangeRight) {
      cost_tmp.safety.occu_lane =
          desired_velocity_ * cfg_.cost().safety().occu_lane_unit_cost();
    }
  }

  // * navigation
  if (seq_lat_behavior == LateralBehavior::kLaneChangeLeft) {
    cost_tmp.navigation.lane_change_preference =
        desired_velocity_ *
        cfg_.cost().navigation().lane_change_left_unit_cost();
  } else if (seq_lat_behavior == LateralBehavior::kLaneChangeRight) {
    cost_tmp.navigation.lane_change_preference =
        desired_velocity_ *
        cfg_.cost().navigation().lane_change_right_unit_cost();
  }

  cost_tmp.weight = duration;
  *cost = cost_tmp;
  return kSuccess;
}

ErrorType EudmPlanner::MultiAgentSimForward(
    const int ego_id, const decimal_t& ego_desired_vel,
    const common::SemanticVehicleSet& semantic_vehicle_set,
    const std::unordered_map<int, double>& est_desired_vel_set,
    const decimal_t& sim_time_resolution, const decimal_t& sim_time_total,
    vec_E<common::Vehicle>* traj,
    std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs) {
  traj->clear();
  surround_trajs->clear();
  for (const auto v : semantic_vehicle_set.semantic_vehicles) {
    if (v.first == ego_id) continue;
    surround_trajs->insert(std::pair<int, vec_E<common::Vehicle>>(
        v.first, vec_E<common::Vehicle>()));
  }

  common::SemanticVehicleSet semantic_vehicle_set_tmp = semantic_vehicle_set;

  int n_1 = std::floor(sim_time_total / sim_time_resolution);
  decimal_t dt_remain = sim_time_total - n_1 * sim_time_resolution;
  std::vector<decimal_t> dt_steps(n_1, sim_time_resolution);
  if (fabs(dt_remain) > kEPS) {
    dt_steps.insert(dt_steps.begin(), dt_remain);
  }

  OnLaneForwardSimulation::Param ego_sim_param;
  OnLaneForwardSimulation::Param agent_sim_param;
  GetSimParam(cfg_.sim().ego(), &ego_sim_param);
  GetSimParam(cfg_.sim().agent(), &agent_sim_param);

  // printf("[TEST]ego min spacing:
  // %lf.\n",ego_sim_param.idm_param.kMinimumSpacing); printf("[TEST]agent min
  // spacing: %lf.\n",agent_sim_param.idm_param.kMinimumSpacing);

  for (int i = 0; i < static_cast<int>(dt_steps.size()); i++) {
    decimal_t sim_time_step = dt_steps[i];
    // timer.tic();
    std::unordered_map<int, State> state_cache;
    for (auto& v : semantic_vehicle_set_tmp.semantic_vehicles) {
      decimal_t desired_vel = semantic_vehicle_set.semantic_vehicles.at(v.first)
                                  .vehicle.state()
                                  .velocity;
      decimal_t lat_range = 2.2;
      OnLaneForwardSimulation::Param sim_param;
      if (v.first == ego_id) {
        desired_vel = ego_desired_vel;
        sim_param = ego_sim_param;
        lat_range = cfg_.sim().ego().cooperative_lat_range();
      } else {
        desired_vel = est_desired_vel_set.at(v.first);
        sim_param = agent_sim_param;
        lat_range = cfg_.sim().agent().cooperative_lat_range();
      }

      common::VehicleSet vehicle_set;
      for (auto& v_other : semantic_vehicle_set_tmp.semantic_vehicles) {
        // ~ get the subset of vehicles excluding the simulating one
        if (v_other.first != v.first)
          vehicle_set.vehicles.insert(
              std::make_pair(v_other.first, v_other.second.vehicle));
      }

      common::Vehicle leading_vehicle;
      common::State state;
      decimal_t distance_residual_ratio = 0.0;
      if (map_itf_->GetLeadingVehicleOnLane(
              v.second.lane, v.second.vehicle.state(), vehicle_set, lat_range,
              &leading_vehicle, &distance_residual_ratio) != kSuccess) {
        if (planning::OnLaneForwardSimulation::PropagateOnce(
                v.second.lane, desired_vel, v.second.vehicle, sim_time_step,
                sim_param, &state) != kSuccess) {
          // LOG(ERROR) << "[Eudm]failed to forward without leading vehicle.";
          return kWrongStatus;
        }
      } else {
        bool is_collision = false;
        map_itf_->CheckCollisionUsingState(
            v.second.vehicle.param(), v.second.vehicle.state(),
            leading_vehicle.param(), leading_vehicle.state(), &is_collision);
        if (is_collision && v.first == ego_id) {
          // LOG(ERROR) << "[Eudm]failed to forward due to collision between "
          //            << v.first << " and " << leading_vehicle.id();
          return kWrongStatus;
        }

        if (planning::OnLaneForwardSimulation::PropagateOnce(
                v.second.lane, desired_vel, v.second.vehicle, sim_time_step,
                leading_vehicle, sim_param, &state) != kSuccess) {
          // LOG(ERROR) << "[Eudm]failed to forward with leading vehicle.";
          return kWrongStatus;
        }
      }

      // update state
      state_cache.insert(std::make_pair(v.first, state));
    }

    // use state cache to update vehicle set
    for (auto& s : state_cache) {
      semantic_vehicle_set_tmp.semantic_vehicles.at(s.first).vehicle.set_state(
          s.second);
      if (s.first == ego_id) {
        traj->push_back(
            semantic_vehicle_set_tmp.semantic_vehicles.at(s.first).vehicle);
      } else {
        surround_trajs->at(s.first).push_back(
            semantic_vehicle_set_tmp.semantic_vehicles.at(s.first).vehicle);
      }
    }
  }  // end num_steps_forward for
  return kSuccess;
}

ErrorType EudmPlanner::JudgeBehaviorByLaneId(
    const int ego_lane_id_by_pos, LateralBehavior* behavior_by_lane_id) {
  if (ego_lane_id_by_pos == ego_lane_id_) {
    *behavior_by_lane_id = common::LateralBehavior::kLaneKeeping;
    return kSuccess;
  }

  auto it = std::find(potential_lk_lane_ids_.begin(),
                      potential_lk_lane_ids_.end(), ego_lane_id_by_pos);
  auto it_lcl = std::find(potential_lcl_lane_ids_.begin(),
                          potential_lcl_lane_ids_.end(), ego_lane_id_by_pos);
  auto it_lcr = std::find(potential_lcr_lane_ids_.begin(),
                          potential_lcr_lane_ids_.end(), ego_lane_id_by_pos);

  if (it != potential_lk_lane_ids_.end()) {
    // ~ if routing information is available, here
    // ~ we still need to check whether the change is consist with the
    *behavior_by_lane_id = common::LateralBehavior::kLaneKeeping;
    return kSuccess;
  }

  if (it_lcl != potential_lcl_lane_ids_.end()) {
    *behavior_by_lane_id = common::LateralBehavior::kLaneChangeLeft;
    return kSuccess;
  }

  if (it_lcr != potential_lcr_lane_ids_.end()) {
    *behavior_by_lane_id = common::LateralBehavior::kLaneChangeRight;
    return kSuccess;
  }

  *behavior_by_lane_id = common::LateralBehavior::kUndefined;
  return kSuccess;
}

ErrorType EudmPlanner::UpdateEgoLaneId(const int new_ego_lane_id) {
  ego_lane_id_ = new_ego_lane_id;
  // GetPotentialLaneIds(ego_lane_id_, common::LateralBehavior::kLaneKeeping,
  //                     &potential_lk_lane_ids_);
  // GetPotentialLaneIds(ego_lane_id_, common::LateralBehavior::kLaneChangeLeft,
  //                     &potential_lcl_lane_ids_);
  // GetPotentialLaneIds(ego_lane_id_,
  // common::LateralBehavior::kLaneChangeRight,
  //                     &potential_lcr_lane_ids_);
  return kSuccess;
}

ErrorType EudmPlanner::GetPotentialLaneIds(
    const int source_lane_id, const LateralBehavior& beh,
    std::vector<int>* candidate_lane_ids) const {
  candidate_lane_ids->clear();
  if (beh == common::LateralBehavior::kUndefined ||
      beh == common::LateralBehavior::kLaneKeeping) {
    map_itf_->GetChildLaneIds(source_lane_id, candidate_lane_ids);
  } else if (beh == common::LateralBehavior::kLaneChangeLeft) {
    int l_lane_id;
    if (map_itf_->GetLeftLaneId(source_lane_id, &l_lane_id) == kSuccess) {
      map_itf_->GetChildLaneIds(l_lane_id, candidate_lane_ids);
      candidate_lane_ids->push_back(l_lane_id);
    }
  } else if (beh == common::LateralBehavior::kLaneChangeRight) {
    int r_lane_id;
    if (map_itf_->GetRightLaneId(source_lane_id, &r_lane_id) == kSuccess) {
      map_itf_->GetChildLaneIds(r_lane_id, candidate_lane_ids);
      candidate_lane_ids->push_back(r_lane_id);
    }
  } else {
    assert(false);
  }
  return kSuccess;
}

void EudmPlanner::set_map_interface(EudmPlannerMapItf* itf) { map_itf_ = itf; }

void EudmPlanner::set_desired_velocity(const decimal_t desired_vel) {
  desired_velocity_ = std::max(0.0, desired_vel);
}

void EudmPlanner::set_lane_change_info(const LaneChangeInfo& lc_info) {
  lc_info_ = lc_info;
}

decimal_t EudmPlanner::desired_velocity() const { return desired_velocity_; }

vec_E<vec_E<common::Vehicle>> EudmPlanner::forward_trajs() const {
  return forward_trajs_;
}

int EudmPlanner::winner_id() const { return winner_id_; }

void EudmPlanner::UpdateDcpTree(const BtAction& ongoing_action) {
  dcp_tree_ptr_->set_ongoing_action(ongoing_action);
  dcp_tree_ptr_->UpdateScript();
  sim_time_total_ =
      (dcp_tree_ptr_->tree_height() - 1) * dcp_tree_ptr_->sim_time_per_layer() +
      ongoing_action.t;
}

EudmPlannerMapItf* EudmPlanner::map_itf() const { return map_itf_; }

}  // namespace planning