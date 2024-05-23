#include "comparator/vicon.hpp"
#include <DataStreamClient.h>
#include <IDataStreamClientBase.h>


namespace vicon
{

std::optional<datastream::Client> connect_to_server(const std::string & server)
{
  auto client = datastream::Client{};
  auto result = client.Connect(server);

  if (result.Result != datastream::Result::Success) {
    return {};
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


}
