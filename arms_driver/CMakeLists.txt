cmake_minimum_required(VERSION 3.16)
project(arms_driver VERSION 0.2.0 LANGUAGES C)

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
  src/lease.c
  src/arms.c
  src/pn.c
  src/calibration.c
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

add_executable(arms_test tools/arms_test/main.c tools/arms_test/motion.c tools/arms_test/joint_select.c
               tools/arms_test/lineedit.c tools/arms_test/power.c
               tools/arms_test/runtime_env.c tools/arms_test/tlog.c)
target_link_libraries(arms_test PRIVATE arms_driver)

# Motor main power for the test tool (BXI PCI SDK, x86-64 static library)
set(BXI_PCI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/bxi_pci_drv)
if(EXISTS ${BXI_PCI_DIR}/x86_64/libbxi_pci_drv.a)
  target_compile_definitions(arms_test PRIVATE ARMS_HAVE_PCI_POWER=1)
  target_include_directories(arms_test PRIVATE ${BXI_PCI_DIR})
  target_link_libraries(arms_test PRIVATE ${BXI_PCI_DIR}/x86_64/libbxi_pci_drv.a)
  add_executable(arms_power tools/arms_power.c)
  target_include_directories(arms_power PRIVATE ${BXI_PCI_DIR})
  target_link_libraries(arms_power PRIVATE ${BXI_PCI_DIR}/x86_64/libbxi_pci_drv.a pthread)
  install(TARGETS arms_power RUNTIME DESTINATION bin)
  message(STATUS "PCI power control: enabled")
else()
  message(STATUS "PCI power control: disabled (no x86-64 SDK)")
endif()

include(CTest)
if(BUILD_TESTING)
  # Only the test supplies PCI symbols; this executable cannot issue hardware requests.
  add_executable(test_power tests/test_power.c tools/arms_test/power.c)
  target_compile_definitions(test_power PRIVATE ARMS_HAVE_PCI_POWER=1)
  target_include_directories(test_power PRIVATE tools/arms_test ${BXI_PCI_DIR})
  add_test(NAME power_retry_stub COMMAND test_power)
  add_executable(test_config tests/test_config.c tools/arms_test/joint_select.c)
  target_include_directories(test_config PRIVATE src tools/arms_test third_party/cjson)
  target_link_libraries(test_config PRIVATE arms_driver)
  add_test(NAME config COMMAND test_config ${CMAKE_CURRENT_SOURCE_DIR}/tests/mock_arms.json)
  add_executable(test_external tests/test_external.c)
  target_link_libraries(test_external PRIVATE arms_driver)
  add_test(NAME external_mode COMMAND test_external ${CMAKE_CURRENT_SOURCE_DIR}/tests/mock_arms.json)
  add_executable(test_control tests/test_control.c)
  target_link_libraries(test_control PRIVATE arms_driver m)
  add_test(NAME control COMMAND test_control ${CMAKE_CURRENT_SOURCE_DIR}/tests/mock_arms.json)
  add_executable(test_recover tests/test_recover.c)
  target_link_libraries(test_recover PRIVATE arms_driver)
  add_test(NAME recover COMMAND test_recover ${CMAKE_CURRENT_SOURCE_DIR}/tests/mock_arms.json)
  add_executable(test_check tests/test_check.c)
  target_link_libraries(test_check PRIVATE arms_driver)
  add_test(NAME startup_check COMMAND test_check ${CMAKE_CURRENT_SOURCE_DIR}/tests/mock_arms.json)
  add_executable(test_protect tests/test_protect.c)
  target_link_libraries(test_protect PRIVATE arms_driver)
  add_test(NAME overspeed_protect COMMAND test_protect ${CMAKE_CURRENT_SOURCE_DIR}/tests/mock_arms.json)
  add_test(NAME help_lists_check COMMAND arms_test --mock -c ${CMAKE_CURRENT_SOURCE_DIR}/tests/mock_arms.json -e "help")
  set_tests_properties(help_lists_check PROPERTIES PASS_REGULAR_EXPRESSION "  check +检查已打开关节")
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME script_cli COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_script_cli.py)
  add_test(NAME cli_contract COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_cli_contract.py
    $<TARGET_FILE:arms_test> ${CMAKE_CURRENT_SOURCE_DIR}/tests/mock_arms.json)
  add_test(NAME parameter_cli COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_parameters.py
    $<TARGET_FILE:arms_test> ${CMAKE_CURRENT_SOURCE_DIR}/tests/mock_arms.json)
  add_test(NAME tab_completion COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_completion.py
    $<TARGET_FILE:arms_test> ${CMAKE_CURRENT_SOURCE_DIR}/tests/mock_arms.json)
  add_test(NAME maintenance_cli COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_maintenance.py
    $<TARGET_FILE:arms_test> ${CMAKE_CURRENT_SOURCE_DIR}/tests/mock_arms.json)
  add_test(NAME drive_monitor COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_diag.py
    $<TARGET_FILE:arms_test> ${CMAKE_CURRENT_SOURCE_DIR}/tests/mock_arms.json)
  add_test(NAME comm_event_log COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_events.py
    $<TARGET_FILE:arms_test> ${CMAKE_CURRENT_SOURCE_DIR}/tests/mock_arms.json)
  add_test(NAME shared_link COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_shared_link.py
    $<TARGET_FILE:arms_test> ${CMAKE_CURRENT_SOURCE_DIR}/tests/mock_arms.json)
  add_test(NAME mock_session COMMAND arms_test --mock -c ${CMAKE_CURRENT_SOURCE_DIR}/tests/mock_arms.json
           -e "status" -e "bus" -e "enable all" -e "mit 17 0.1 5 0.5"
           -e "swing 17 0.05 1 1 5 0.5" -e "mit 18 0.1 2 0.1 0.05" -e "hold" -e "commtest 2 --enabled 5 0.5"
           -e "disable" -e "status" -e "help")
endif()

if(ARMS_HAVE_ETHERLAB)
  get_filename_component(ETHERLAB_LIBDIR "${ETHERLAB_LIBRARY}" DIRECTORY)
  set_property(TARGET arms_test APPEND PROPERTY INSTALL_RPATH "${ETHERLAB_LIBDIR}")
endif()
install(PROGRAMS scripts/rt_setup.sh DESTINATION share/arms_driver/scripts)
install(TARGETS arms_test RUNTIME DESTINATION bin)
install(TARGETS arms_driver ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES config/arms.json DESTINATION share/arms_driver/config)
