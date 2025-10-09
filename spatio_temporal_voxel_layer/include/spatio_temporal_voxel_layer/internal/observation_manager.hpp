/*
 * ObservationManager consolidates management of measurement buffers,
 * subscribers, and helper services associated with sensor observations.
 * Centralizing this behavior removes a substantial amount of bookkeeping
 * from the main layer implementation and opens up unit-testing seams for
 * future work.
 */

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__INTERNAL__OBSERVATION_MANAGER_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__INTERNAL__OBSERVATION_MANAGER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/message_filter.h"

#include "spatio_temporal_voxel_layer/measurement_buffer.hpp"
#include "spatio_temporal_voxel_layer/measurement_reading.h"

namespace spatio_temporal_voxel_layer::internal
{

class ObservationManager
{
public:
  using BufferPtr = std::shared_ptr<buffer::MeasurementBuffer>;
  using SubscriberPtr = std::shared_ptr<message_filters::SubscriberBase<rclcpp_lifecycle::LifecycleNode>>;
  using NotifierPtr = std::shared_ptr<tf2_ros::MessageFilterBase>;
  using ServicePtr = rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr;

  ObservationManager() = default;

  void registerBuffer(const BufferPtr & buffer, bool marking, bool clearing);
  void addSubscriber(const SubscriberPtr & subscriber);
  void addNotifier(const NotifierPtr & notifier);
  void addEnableService(const ServicePtr & service);

  bool collectMarkingObservations(
    std::vector<observation::MeasurementReading> & output) const;
  bool collectClearingObservations(
    std::vector<observation::MeasurementReading> & output) const;
  void resetBuffersAfterReading() const;

  void addStaticObservation(const observation::MeasurementReading & obs);
  bool clearStaticObservations();

  void forEachBuffer(const std::function<void(const BufferPtr &)> & fn) const;
  void forEachBuffer(const std::function<void(BufferPtr &)> & fn);
  BufferPtr bufferBySource(const std::string & source) const;

  void activateSubscribers();
  void deactivateSubscribers();

  void resetLastUpdatedTime();

  const std::vector<BufferPtr> & buffers() const {return observation_buffers_;}
  std::vector<BufferPtr> & buffers() {return observation_buffers_;}
  const std::vector<SubscriberPtr> & subscribers() const {return observation_subscribers_;}
  const std::vector<NotifierPtr> & notifiers() const {return observation_notifiers_;}
  const std::vector<ServicePtr> & enableServices() const {return buffer_enabler_servers_;}
  std::vector<NotifierPtr> & notifiers() {return observation_notifiers_;}

private:
  struct BufferRegistration
  {
    BufferPtr buffer;
    bool marking{false};
    bool clearing{false};
  };

  std::vector<BufferRegistration> buffer_registrations_{};
  std::vector<BufferPtr> observation_buffers_{};
  std::vector<SubscriberPtr> observation_subscribers_{};
  std::vector<NotifierPtr> observation_notifiers_{};
  std::vector<ServicePtr> buffer_enabler_servers_{};
  std::vector<BufferPtr> marking_buffers_{};
  std::vector<BufferPtr> clearing_buffers_{};
  std::vector<observation::MeasurementReading> static_observations_{};
};

}  // namespace spatio_temporal_voxel_layer::internal

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__INTERNAL__OBSERVATION_MANAGER_HPP_
