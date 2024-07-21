#include "comparator/vicon.hpp"
#include <DataStreamClient.h>
#include <IDataStreamClientBase.h>
#include <iostream>
#include <memory>


namespace vicon
{

template<typename T>
bool is_enabled(T result)
{
  return result.Result == datastream::Result::Success;
}

std::unique_ptr<datastream::Client> connect_to_server(
  const std::string & server)
{
  auto client = std::make_unique<datastream::Client>();
  auto result = client->Connect(server);

  if (!is_enabled(result)) {
    throw std::runtime_error("Failed to connect to the server: " + server);
  }

  return client;
}

bool configure_datastream(datastream::Client & client)
{
  // Enable Marker data
  auto enable_marker_result = client.EnableMarkerData();
  if (enable_marker_result.Result != datastream::Result::Success) {
    return false;
  }

  // Enable Segment Data
  auto enable_segment_result = client.EnableSegmentData();
  if (enable_segment_result.Result != datastream::Result::Success) {
    return false;
  }

  client.SetStreamMode(datastream::StreamMode::ClientPull);
  return true;
}


std::unique_ptr<datastream::Client> & operator|(
  std::unique_ptr<datastream::Client> & client,
  const ConfigureOption & option)
{
  bool success = false;
  switch (option) {
    case EnableMarkerData:
      success = is_enabled(client->EnableMarkerData());
      break;
    case EnableSegmentData:
      success = is_enabled(client->EnableSegmentData());
      break;
  }

  if (!success) {
    auto str = configure_option_strings.at(option);
    throw std::runtime_error("Failed to enable " + str);
  }

  return client;
}


}
