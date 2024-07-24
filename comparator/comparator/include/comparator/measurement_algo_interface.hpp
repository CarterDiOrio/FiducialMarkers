#ifndef INC_GUARD_MEASUREMENT_ALGO_INTERFACE_HPP
#define INC_GUARD_MEASUREMENT_ALGO_INTERFACE_HPP

#include <string>
#include <sophus/se3.hpp>
#include <optional>

class MeasurementAlgoInterface
{
public:
  virtual ~MeasurementAlgoInterface() = default;

  virtual std::string get_name() = 0;
  virtual void display() = 0;
  virtual std::optional<Sophus::SE3d> detect() = 0;
};


#endif