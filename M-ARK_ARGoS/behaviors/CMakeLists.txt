if(ARGOS_BUILD_FOR_SIMULATOR)
  include_directories(${CMAKE_SOURCE_DIR}/plugins/robots/kilobot/control_interface)

  
  #
  # ARK loop function : client server dhtf
  #
  add_executable(kilobot_ALF_dhtf kilobot_ALF_dhtf.c)  
  target_link_libraries(kilobot_ALF_dhtf argos3plugin_simulator_kilolib)

  endif(ARGOS_BUILD_FOR_SIMULATOR)
