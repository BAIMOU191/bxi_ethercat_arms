cmake_minimum_required(VERSION 3.16)
project(arms_driver VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

option(ARMS_WITH_ETHERLAB "Build the EtherLab backend" ON)
set(ETHERLAB_ROOT "/usr/local/etherlab" CACHE PATH "EtherLab installation prefix")
find_package(Threads REQUIRED)

set(ARMS_HAVE_ETHERLAB OFF)
if(ARMS_WITH_ETHERLAB)
  find_path(ETHERLAB_INCLUDE_DIR ecrt.h HINTS ${ETHERLAB_ROOT} PATH_SUFFIXES include)
  find_library(ETHERLAB_LIBRARY ethercat HINTS ${ETHERLAB_ROOT} PATH_SUFFIXES lib lib64)
  if(ETHERLAB_INCLUDE_DIR AND ETHERLAB_LIBRARY)
    set(ARMS_HAVE_ETHERLAB ON)
    message(STATUS "EtherLab: ${ETHERLAB_LIBRARY}")
  else()
    message(WARNING "EtherLab not found: building with the mock backend only")
  endif()
endif()

find_package(Git QUIET)
set(ARMS_GIT "nogit")
if(GIT_FOUND)
  execute_process(COMMAND ${GIT_EXECUTABLE} describe --always --dirty
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                  OUTPUT_VARIABLE ARMS_GIT OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
  if(NOT ARMS_GIT)
    set(ARMS_GIT "nogit")
  endif()
endif()

add_library(arms_driver STATIC
  src/arms.c
  src/config.c
  src/bus_mock.c
  third_party/cjson/cJSON.c)
target_include_directories(arms_driver
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  PRIVATE src third_party/cjson)
target_compile_definitions(arms_driver PRIVATE ARMS_VERSION_STRING="${PROJECT_VERSION}-${ARMS_GIT}")
target_link_libraries(arms_driver PUBLIC Threads::Threads m)
if(ARMS_HAVE_ETHERLAB)
  target_sources(arms_driver PRIVATE src/bus_etherlab.c)
  target_compile_definitions(arms_driver PRIVATE ARMS_HAVE_ETHERLAB=1)
  target_include_directories(arms_driver PRIVATE ${ETHERLAB_INCLUDE_DIR})
  target_link_libraries(arms_driver PUBLIC ${ETHERLAB_LIBRARY})
endif()
set_target_properties(arms_driver PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(arms_test tools/arms_test/main.c tools/arms_test/motion.c tools/arms_test/joint_select.c tools/arms_test/power.c
               tools/arms_test/runtime_env.c tools/arms_test/tlog.c)
target_link_libraries(arms_test PRIVATE arms_driver)

# Motor main power for the test tool (BXI PCI SDK, aarch64 static library)
set(BXI_PCI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/bxi_pci_drv)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$" AND EXISTS ${BXI_PCI_DIR}/arm64/libbxi_pci_drv.a)
  target_compile_definitions(arms_test PRIVATE ARMS_HAVE_PCI_POWER=1)
  target_include_directories(arms_test PRIVATE ${BXI_PCI_DIR})
  target_link_libraries(arms_test PRIVATE ${BXI_PCI_DIR}/arm64/libbxi_pci_drv.a)
  message(STATUS "PCI power control: enabled")
else()
  message(STATUS "PCI power control: disabled (no aarch64 SDK)")
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(test_config tests/test_config.c tools/arms_test/joint_select.c)
  target_include_directories(test_config PRIVATE src tools/arms_test third_party/cjson)
  target_link_libraries(test_config PRIVATE arms_driver)
  add_test(NAME config COMMAND test_config ${CMAKE_CURRENT_SOURCE_DIR}/config/arms.json)
  add_executable(test_external tests/test_external.c)
  target_link_libraries(test_external PRIVATE arms_driver)
  add_test(NAME external_mode COMMAND test_external ${CMAKE_CURRENT_SOURCE_DIR}/config/arms.json)
  add_executable(test_control tests/test_control.c)
  target_link_libraries(test_control PRIVATE arms_driver m)
  add_test(NAME control COMMAND test_control ${CMAKE_CURRENT_SOURCE_DIR}/config/arms.json)
  add_executable(test_recover tests/test_recover.c)
  target_link_libraries(test_recover PRIVATE arms_driver)
  add_test(NAME recover COMMAND test_recover ${CMAKE_CURRENT_SOURCE_DIR}/config/arms.json)
  add_test(NAME mock_session COMMAND arms_test --mock -c ${CMAKE_CURRENT_SOURCE_DIR}/config/arms.json
           -e "status" -e "bus" -e "enable all" -e "mit 17 0.1 -m 0.3" -e "home all -m 0.5"
           -e "swing 17 0.05 1 1" -e "mit 18 0.1 2 0.1 0.05" -e "hold" -e "commtest 2 --enabled"
           -e "disable" -e "status" -e "help")
endif()

install(TARGETS arms_test RUNTIME DESTINATION bin)
install(TARGETS arms_driver ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES config/arms.json DESTINATION share/arms_driver/config)
