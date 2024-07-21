#ifndef INC_GUARD_VICON_HPP
#define INC_GUARD_VICON_HPP

#include "DataStreamClient.h"
#include <memory>
#include <optional>
#include <map>

namespace datastream = ViconDataStreamSDK::CPP;

namespace vicon
{

/// \brief Options for configuring the datastream
enum ConfigureOption
{
  EnableMarkerData,
  EnableSegmentData
};

/// \brief string representations of the ConfigureOption enum
const std::map<ConfigureOption,
  std::string> configure_option_strings {
  {EnableMarkerData, "EnableMarkerData"},
  {EnableSegmentData, "EnableSegmentData"}
};

/// \brief Tries to connect to the vicon server
/// \param server The server IP and port to connect to
/// \throws std::runtime_error if the connection fails
/// \return A client if the connection was successful
std::unique_ptr<datastream::Client> connect_to_server(
  const std::string & server);

/// \brief Configures the datastreams for the client
/// \param client The client to initialize the datastreams for
/// \return True if the datastreams were successfully initialized, otherwise false
bool configure_datastream(datastream::Client & client);

/// \brief Enables the datastreams for the client
/// \param client The client to initialize the datastreams for
/// \param option The option to configure the datastream with
/// \throws std::runtime_error if the configuration fails
/// \return The client with the datastream enabled
std::unique_ptr<datastream::Client> & operator|(
  std::unique_ptr<datastream::Client> & client,
  const ConfigureOption & option);

}


#endif
