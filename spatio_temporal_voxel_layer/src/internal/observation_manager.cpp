#include "spatio_temporal_voxel_layer/internal/observation_manager.hpp"

#include <algorithm>
#include <utility>

namespace spatio_temporal_voxel_layer::internal
{

void ObservationManager::registerBuffer(const BufferPtr & buffer, bool marking, bool clearing)
{
  if (!buffer) {
    return;
  }

  observation_buffers_.push_back(buffer);
  buffer_registrations_.push_back(BufferRegistration{buffer, marking, clearing});

  if (marking) {
    marking_buffers_.push_back(buffer);
  }

  if (clearing) {
    clearing_buffers_.push_back(buffer);
  }
}

void ObservationManager::addSubscriber(const SubscriberPtr & subscriber)
{
  if (subscriber) {
    observation_subscribers_.push_back(subscriber);
  }
}

void ObservationManager::addNotifier(const NotifierPtr & notifier)
{
  if (notifier) {
    observation_notifiers_.push_back(notifier);
  }
}

void ObservationManager::addEnableService(const ServicePtr & service)
{
  if (service) {
    buffer_enabler_servers_.push_back(service);
  }
}

bool ObservationManager::collectMarkingObservations(
  std::vector<observation::MeasurementReading> & output) const
{
  bool current = true;

  for (const auto & buffer : marking_buffers_) {
    if (!buffer) {
      continue;
    }

    buffer->Lock();
    buffer->GetReadings(output);
    current = current && buffer->UpdatedAtExpectedRate();
    buffer->Unlock();
  }

  output.insert(output.end(), static_observations_.begin(), static_observations_.end());
  return current;
}

bool ObservationManager::collectClearingObservations(
  std::vector<observation::MeasurementReading> & output) const
{
  bool current = true;

  for (const auto & buffer : clearing_buffers_) {
    if (!buffer) {
      continue;
    }

    buffer->Lock();
    buffer->GetReadings(output);
    current = current && buffer->UpdatedAtExpectedRate();
    buffer->Unlock();
  }

  return current;
}

void ObservationManager::resetBuffersAfterReading() const
{
  for (const auto & buffer : clearing_buffers_) {
    if (!buffer) {
      continue;
    }

    buffer->Lock();
    if (buffer->ClearAfterReading()) {
      buffer->ResetAllMeasurements();
    }
    buffer->Unlock();
  }

  for (const auto & buffer : marking_buffers_) {
    if (!buffer) {
      continue;
    }

    buffer->Lock();
    if (buffer->ClearAfterReading()) {
      buffer->ResetAllMeasurements();
    }
    buffer->Unlock();
  }
}

void ObservationManager::addStaticObservation(const observation::MeasurementReading & obs)
{
  static_observations_.push_back(obs);
}

bool ObservationManager::clearStaticObservations()
{
  const bool had_observations = !static_observations_.empty();
  static_observations_.clear();
  return had_observations;
}

void ObservationManager::forEachBuffer(const std::function<void(const BufferPtr &)> & fn) const
{
  for (const auto & buffer : observation_buffers_) {
    fn(buffer);
  }
}

void ObservationManager::forEachBuffer(const std::function<void(BufferPtr &)> & fn)
{
  for (auto & buffer : observation_buffers_) {
    fn(buffer);
  }
}

ObservationManager::BufferPtr ObservationManager::bufferBySource(const std::string & source) const
{
  auto it = std::find_if(
    buffer_registrations_.begin(), buffer_registrations_.end(),
    [&source](const BufferRegistration & registration) {
      return registration.buffer && registration.buffer->GetSourceName() == source;
    });

  if (it == buffer_registrations_.end()) {
    return nullptr;
  }

  return it->buffer;
}

void ObservationManager::activateSubscribers()
{
  for (auto & subscriber : observation_subscribers_) {
    if (subscriber) {
      subscriber->subscribe();
    }
  }
}

void ObservationManager::deactivateSubscribers()
{
  for (auto & subscriber : observation_subscribers_) {
    if (subscriber) {
      subscriber->unsubscribe();
    }
  }
}

void ObservationManager::resetLastUpdatedTime()
{
  for (auto & buffer : observation_buffers_) {
    if (buffer) {
      buffer->ResetLastUpdatedTime();
    }
  }
}

}  // namespace spatio_temporal_voxel_layer::internal
