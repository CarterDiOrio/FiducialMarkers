cmake_minimum_required(VERSION 3.22)
project(pi_eink)


option(BUILD_SERVER "Build the server" OFF)
option(BUILD_CLIENT "Build the client" OFF)

# this is meant to be built on the raspberry pi or similar enviorment
if(BUILD_SERVER)

  find_package(httplib REQUIRED)
  find_package(nlohmann_json 3.2.0 REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(aruco REQUIRED)
  find_library(LGPIO_LIB NAMES lgpio)


  message(STATUS "Found lgpio at ${LGPIO_LIB}")
  message(STATUS "Found OpenCV at ${OpenCV_LIBS}")

  #adapting the IT8951 library to be used with cmake
  add_library(IT8951
    IT8951-ePaper/Config/DEV_Config.c
    IT8951-ePaper/Config/dev_hardware_SPI.c
    IT8951-ePaper/Config/RPI_gpiod.c
    IT8951-ePaper/e-Paper/EPD_IT8951.c
  )

  target_compile_definitions(IT8951 PRIVATE LGPIO)
  target_include_directories(IT8951
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/IT8951-ePaper/>
    $<INSTALL_INTERFACE:include/>)

  add_executable(server
    src/server.cpp
    src/epaper.cpp
    src/image.cpp
    src/chessboard.cpp
    src/april_tags.cpp
    src/aruco_tags.cpp
    src/drawing_utils.cpp
  )

  target_link_libraries(server PRIVATE IT8951
                               httplib::httplib
                               ${LGPIO_LIB}
                               nlohmann_json::nlohmann_json
                               aruco
                               ${OpenCV_LIBS})

  target_include_directories(server PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_include_directories(server
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)


  target_compile_options(server PUBLIC -Wall -Wextra -pedantic)
  target_compile_features(server PUBLIC cxx_std_20)

  install(DIRECTORY include/pi_eink DESTINATION include)
  install(TARGETS server EXPORT pi_eink-targets)

endif()

if(BUILD_CLIENT)
  find_package(cpr REQUIRED)
  find_package(nlohmann_json 3.2.0 REQUIRED)
  find_package(OpenSSL REQUIRED COMPONENTS SSL Crypto)

  set(BOOST_REQUIRED_COMPONENTS
    program_options
  )
  set(BOOST_MIN_VERSION 1.70.0)
  find_package(
    Boost ${BOOST_MIN_VERSION} REQUIRED
    COMPONENTS ${BOOST_REQUIRED_COMPONENTS}
  )

  add_library(eink_client
    src/client.cpp
  )


  target_link_libraries(eink_client PRIVATE
                                    cpr::cpr
                                    nlohmann_json::nlohmann_json
                                    ${OPENSSL_LIBRARIES})


  target_include_directories(eink_client
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

  target_compile_options(eink_client PUBLIC -Wall -Wextra -pedantic -fPIC)
  target_compile_features(eink_client PUBLIC cxx_std_23)

  install(DIRECTORY include/pi_eink DESTINATION include)
  install(TARGETS eink_client EXPORT pi_eink-targets)

  if (BUILD_CLIENT_TEST)

    add_executable(client_test src/client_test.cpp src/client.cpp)
    target_link_libraries(client_test PRIVATE eink_client)

    target_include_directories(client_test
      PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
      $<INSTALL_INTERFACE:include/>)

    install(TARGETS client_test EXPORT pi_eink-targets)

  endif()
endif()

if (NOT DEFINED CMAKE_PREFIX_PATH)
  set(CMAKE_PREFIX_PATH "")
endif()

install(EXPORT pi_eink-targets
         FILE pi_eink-targets.cmake
         NAMESPACE pi_eink::
         DESTINATION ${CMAKE_PREFIX_PATH}/lib/cmake/${PROJECT_NAME})

configure_file(pi_eink-config.cmake.in pi_eink-config.cmake @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/pi_eink-config.cmake
        DESTINATION ${CMAKE_PREFIX_PATH}/lib/cmake/${PROJECT_NAME})
