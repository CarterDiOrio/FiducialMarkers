#ifndef INC_GUARD_EIGEN_JSON_HPP
#define INC_GUARD_EIGEN_JSON_HPP

#include <Eigen/Dense>
#include <nlohmann/adl_serializer.hpp>
#include <nlohmann/json.hpp>
#include <iostream>

namespace nlohmann
{

template<typename T, int S, int S2>
struct adl_serializer<Eigen::Matrix<T, S, S2>>
{
  static void to_json(json & j, const Eigen::Matrix<T, S, S2> & value)
  {
    std::vector<T> data{value.data(), value.data() + value.size()};
    j = data;
  }

  static void from_json(const json & j, Eigen::Matrix<T, S, S2> & p)
  {
    std::vector<T> data = j.get<std::vector<T>>();

    for (int c = 0; c < S2; c++) {
      for (int r = 0; r < S; r++) {
        p(r, c) = data[r + c * S2];
      }
    }
  }
};

// template<typename T>
// struct adl_serializer<Eigen::Vector2<T>>
// {
//   static void to_json(json & j, const Eigen::Vector2<T> & value)
//   {
//     std::vector<T> data{value.data(), value.data() + value.size()};
//     j = data;
//   }

//   static void from_json(const json & j, Eigen::Vector2<T> & p)
//   {
//     std::vector<T> data = j.get<std::vector<T>>();
//     p = Eigen::Map<Eigen::Vector2<T>>(data.data());
//   }
// };

}

#endif
