/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/collated_trajectory_builder.h"

#include "cartographer/common/time.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

namespace {

constexpr double kSensorDataRatesLoggingPeriodSeconds = 15.;

}  // namespace

/**
 * @brief Construct a new Collated Trajectory Builder:: Collated Trajectory Builder object
 * 
 * @param[in] trajectory_options 轨迹的参数配置
 * @param[in] sensor_collator 传入的整理传感器的类,有2种类型
 * @param[in] trajectory_id 新生成的轨迹的id
 * @param[in] expected_sensor_ids 所有需要的topic的名字的集合
 * @param[in] wrapped_trajectory_builder 完整的slam GlobalTrajectoryBuilder
 */
CollatedTrajectoryBuilder::CollatedTrajectoryBuilder(
    const proto::TrajectoryBuilderOptions& trajectory_options,
    sensor::CollatorInterface* const sensor_collator, const int trajectory_id,
    const std::set<SensorId>& expected_sensor_ids,
    std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder)
    : sensor_collator_(sensor_collator),
      collate_landmarks_(trajectory_options.collate_landmarks()),
      collate_fixed_frame_(trajectory_options.collate_fixed_frame()),
      trajectory_id_(trajectory_id),
      wrapped_trajectory_builder_(std::move(wrapped_trajectory_builder)),
      last_logging_time_(std::chrono::steady_clock::now()) {
  absl::flat_hash_set<std::string> expected_sensor_id_strings;
  for (const auto& sensor_id : expected_sensor_ids) {
    if (sensor_id.type == SensorId::SensorType::LANDMARK &&
        !collate_landmarks_) {
      continue;
    }
    if (sensor_id.type == SensorId::SensorType::FIXED_FRAME_POSE &&
        !collate_fixed_frame_) {
      continue;
    }
    expected_sensor_id_strings.insert(sensor_id.id);
  }
  sensor_collator_->AddTrajectory(
      trajectory_id, expected_sensor_id_strings,
      [this](const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
        HandleCollatedSensorData(sensor_id, std::move(data));
      });
}

// 将数据传入sensor_collator_的AddSensorData进行排序
void CollatedTrajectoryBuilder::AddData(std::unique_ptr<sensor::Data> data) {
  sensor_collator_->AddSensorData(trajectory_id_, std::move(data));
}

/**
 * @brief 处理 按照时间顺序分发出来的传感器数据
 * 
 * @param[in] sensor_id 传感器的topic的名字
 * @param[in] data 需要处理的数据(Data是个类模板,可处理多种不同数据类型的数据)
 */
void CollatedTrajectoryBuilder::HandleCollatedSensorData(
    const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
  auto it = rate_timers_.find(sensor_id);
  if (it == rate_timers_.end()) {
    it = rate_timers_
             .emplace(
                 std::piecewise_construct, std::forward_as_tuple(sensor_id),
                 std::forward_as_tuple(
                     common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)))
             .first;
  }
  it->second.Pulse(data->GetTime());

  if (std::chrono::steady_clock::now() - last_logging_time_ >
      common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)) {
    for (const auto& pair : rate_timers_) {
      LOG(INFO) << pair.first << " rate: " << pair.second.DebugString();
    }
    last_logging_time_ = std::chrono::steady_clock::now();
  }

  // 将排序好的数据送入 GlobalTrajectoryBuilder中的AddSensorData()函数中进行使用
  data->AddToTrajectoryBuilder(wrapped_trajectory_builder_.get());
}

}  // namespace mapping
}  // namespace cartographer
