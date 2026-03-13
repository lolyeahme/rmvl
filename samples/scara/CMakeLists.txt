if(NOT BUILD_rmvl_lpss)
  return()
endif()

rmvl_add_exe(
  rmvl_scara_simulator
  SOURCES scara_simulator.cpp
  DEPENDS lpss
)

rmvl_add_exe(
  rmvl_scara_controller
  SOURCES scara_controller.cpp
  DEPENDS lpss
)
