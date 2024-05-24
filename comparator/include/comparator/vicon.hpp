#ifndef INC_GUARD_VICON_HPP
#define INC_GUARD_VICON_HPP

#include "DataStreamClient.h"
#include <memory>
#include <optional>

namespace vicon
{
namespace datastream = ViconDataStreamSDK::CPP;

/// \brief Tries to connect to the vicon server
/// \param server The server IP and port to connect to
/// \return A client if the connection was successful, otherwise an empty optional
std::optional<std::unique_ptr<datastream::Client>> connect_to_server(const std::string & server);

/// \brief Configures the datastreams for the client
/// \param client The client to initialize the datastreams for
/// \return True if the datastreams were successfully initialized, otherwise false
bool configure_datastream(datastream::Client & client);

}


#endif
