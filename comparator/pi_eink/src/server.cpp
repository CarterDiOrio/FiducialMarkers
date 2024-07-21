#include <chrono>
#include <csignal>
#include <functional>
#include <future>
#include <iostream>
#include <mutex>
#include <signal.h>
#include <thread>
#include "httplib.h"
#include "pi_eink/epaper.hpp"
#include "pi_eink/chessboard.hpp"
#include "pi_eink/april_tags.hpp"
#include "pi_eink/aruco.hpp"
#include "pi_eink/api.hpp"
#include "pi_eink/drawing_utils.hpp"
#include <queue>
#include <nlohmann/json.hpp>


using json = nlohmann::json;
using namespace pi_eink;

bool running = true;
httplib::Server server;

void signal_handler(int signum)
{
  std::cout << "Interrupt signal (" << signum << ") received." << std::endl;
  running = false;
  server.stop();
}

class DisplayCommandQueue
{

public:
  void push(std::function<void(pi_eink::EPaper &)> command)
  {
    std::lock_guard<std::mutex> lock(mutex);
    commands.push(command);
  }

  std::function<void(pi_eink::EPaper &)> pop()
  {
    std::lock_guard<std::mutex> lock(mutex);
    auto command = commands.front();
    commands.pop();
    return command;
  }

  bool empty()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return commands.empty();
  }

private:
  std::queue<std::function<void(pi_eink::EPaper &)>> commands;
  std::mutex mutex;
};

struct CornerResponse
{
  double actual_size;
  std::vector<std::pair<double, double>> corners;
};

void to_json(json & j, const CornerResponse & p)
{
  j = json{{"corners", p.corners},
    {"actual_size", p.actual_size}};
}

void from_json(const json & j, CornerResponse & p)
{
  j.at("corners").get_to(p.corners);
  j.at("actual_size").get_to(p.actual_size);
}

int main()
{
  signal(SIGINT, signal_handler);

  DisplayCommandQueue command_queue;

  std::jthread display_thread([&command_queue]() {

      std::cout << "Starting http server" << std::endl;

      // display a chessboard
      server.Post(
        "/chessboard", [&command_queue](const httplib::Request & req,
        httplib::Response & res) {

          auto j = json::parse(req.body);
          const auto chess_request = j.template get<ChessboardRequest>();

          std::promise<ChessboardResponse> promise;
          std::future<ChessboardResponse> future = promise.get_future();

          command_queue.push(
            [&promise, &chess_request](pi_eink::EPaper & display) {
              std::cout << "Drawing Chessboard" << std::endl;

              auto fiducial =
              pi_eink::create_gingham_chessboard(
                display,
                chess_request.desired_square_size,
                chess_request.num_corners);

              auto bytes = pi_eink::render_image(
                fiducial.image,
                pi_eink::EPaper::BytesPerPixel::ONE);

              display.display_bytes(
                bytes,
                0,
                0,
                display.get_panel_width(),
                display.get_panel_height(),
                pi_eink::EPaper::BytesPerPixel::ONE,
                pi_eink::EPaper::Mode::A2);

              promise.set_value(
                ChessboardResponse{
          .actual_square_size = fiducial.square_size,
          .corners = fiducial.points
        }
              );
            });

          future.wait();
          const auto response = future.get();
          json r = response;
          res.set_content(r.dump(), "application/json");
        });

      server.Post(
        "/top_left", [&command_queue](const httplib::Request &,
        httplib::Response & res) {
          command_queue.push(
            [](pi_eink::EPaper & display) {
              std::cout << "Drawing Top Left" << std::endl;
              auto image = image_from_display(display);

              pi_eink::draw_square(
                image,
                0, 0,
                100,
                0.0);

              auto bytes = pi_eink::render_image(
                image,
                pi_eink::EPaper::BytesPerPixel::ONE);

              display_image(display, image);
            });
          res.set_content("Top Left", "text/plain");
        });

      server.Post(
        "/april_tag", [&command_queue](const httplib::Request & req,
        httplib::Response & res) {
          auto json = json::parse(req.body);
          const auto request = json.template get<AprilTagRequest>();

          std::promise<AprilTagResponse> promise;
          std::future<AprilTagResponse> future = promise.get_future();

          command_queue.push(
            [&promise, &request](EPaper & display) {
              april_tags::AprilTagDataSource data_source{
                "/home/student/apriltag-imgs-master"};

              const auto & [img,
              info] =
              draw_april_tag(
                data_source,
                display,
                request.desired_square_size,
                april_tags::from_string(request.family),
                request.id);

              display_image(display, img);

              promise.set_value(
                AprilTagResponse{info.size, info.x, info.y});
            });

          const auto april_response = future.get();
          nlohmann::json j = april_response;
          res.set_content(j.dump(), "application/json");
        }
      );

      server.Post(
        "/aruco_tag", [&command_queue](const httplib::Request & req,
        httplib::Response & res) {
          auto json = json::parse(req.body);
          const auto request = json.template get<ArucoTagRequest>();

          std::promise<ArucoTagResponse> promise;
          std::future<ArucoTagResponse> future = promise.get_future();

          command_queue.push(
            [&promise, &request](EPaper & display) {
              std::cout << "Drawing Aruco Tag" << std::endl;
              const auto [img, info] = aruco_tags::generate_tag(
                display,
                aruco_tags::from_string(request.dictionary_type),
                request.id,
                request.desired_square_size);

              display_image(display, img);

              promise.set_value(
                ArucoTagResponse{info.size, info.x, info.y});
            });

          const auto aruco_response = future.get();
          nlohmann::json j = aruco_response;
          res.set_content(j.dump(), "application/json");
        }
      );

      // clear command
      server.Post(
        "/clear", [&command_queue](const httplib::Request &,
        httplib::Response & res) {
          command_queue.push(
            [](pi_eink::EPaper & display) {
              std::cout << "Clearing Display" << std::endl;
              display.clear();
            });
          res.set_content("Cleared", "text/plain");
        });

      // Sleep display
      server.Post(
        "/sleep", [&command_queue](const httplib::Request &,
        httplib::Response & res) {
          command_queue.push(
            [](pi_eink::EPaper & display) {
              std::cout << "Putting Display to sleep" << std::endl;
              display.sleep();
            });
          res.set_content("Sleeping", "text/plain");
        });

      server.listen("169.254.100.135", 8080);
    });


  std::cout << "Initializing E-Ink Display" << std::endl;
  pi_eink::EPaper display(-1.55, 0.112, true, false);

  std::cout << "Clearing display" << std::endl;
  display.clear();
  display.sleep();

  while (running) {
    while (!command_queue.empty()) {
      std::cout << "Running Display Command" << std::endl;
      auto command = command_queue.pop();
      command(display);
    }
  }

  return 0;
}
