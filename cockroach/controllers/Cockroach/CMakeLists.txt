add_compile_options(-std=c++11)



add_library(Cockroach_Move_Free MODULE Cockroach.h  Cockroach_Free.cpp)
target_link_libraries(Cockroach_Move_Free
  argos3core_simulator
  argos3plugin_simulator_footbot  
  argos3plugin_simulator_genericrobot)

