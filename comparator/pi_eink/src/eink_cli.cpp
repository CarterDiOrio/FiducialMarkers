#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <iostream>
#include <boost/program_options.hpp>
#include <stdexcept>
#include <fstream>
#include <nlohmann/json.hpp>
#include "pi_eink/client.hpp"
#include "pi_eink/api.hpp"

namespace po = boost::program_options;

enum FiducialType {
    Chess,
    April,
    Aruco
};

template<typename Request, typename Response>
Response display_from_json(const nlohmann::json & json, pi_eink::EinkClient &  client) {
    const Request request = json.template get<Request>();
    const pi_eink::EinkClient::Response<Response> response = client.draw(request);
    return response.response.value();
}

template<typename Request, typename Response>
void display_and_save(const nlohmann::json & json, pi_eink::EinkClient & client, std::ofstream & outfile) {
    const auto response = display_from_json<Request, Response>(json, client);
    nlohmann::json j = response;
    outfile << std::setw(4) << j << std::endl;
}

int main(int argc, char** argv) {

    po::options_description desc("Allowed options");
    desc.add_options()("help", "produce help message")(
        "address",
        po::value<std::string>(),
        "The ip address and port to the eink display"
    )(
        "type",
        po::value<std::string>(),
        "the type of fiducial to render"
    )
    (
        "input",
        po::value<std::string>(),
        "The input json description file of what to render"
    )(
        "output",
        po::value<std::string>(),
        "The name of the output file to put the response"
    );

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    } catch (po::error & e) {
        std::cerr << "Error: "  << e.what() << std::endl;
        return 1;
    }

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    const auto address = vm["address"].as<std::string>();
    const auto type = vm["type"].as<std::string>();
    const auto in_file = vm["input"].as<std::string>();
    const auto out_file = vm["output"].as<std::string>();

    pi_eink::EinkClient client{address};

    const auto fiducial_type = [&type]() {
        if (type == "chess") {
            return FiducialType::Chess;
        }
        else if (type == "april") {
            return FiducialType::April;
        }
        else if (type == "aruco") {
            return FiducialType::Chess;
        }
        throw std::runtime_error("unsupported fiducial type");
    }();

    // open input file
    std::ifstream input_file{in_file};
    const auto data = nlohmann::json::parse(input_file);
    input_file.close();

    client.clear();

    std::cout << std::setw(4) <<  data.dump() << std::endl;

    // open output file
    std::ofstream outfile{out_file};
    switch (fiducial_type) {
        case FiducialType::Chess:
            display_and_save<pi_eink::ChessboardRequest, pi_eink::ChessboardResponse>(data, client, outfile);
            break;
        case FiducialType::April:
            display_and_save<pi_eink::AprilTagRequest, pi_eink::AprilTagResponse>(data, client, outfile);
            break;
        case FiducialType::Aruco:
            display_and_save<pi_eink::ArucoTagRequest, pi_eink::ArucoTagResponse>(data, client, outfile);
    }

    client.sleep();
    outfile.close();


    return 1;
}
