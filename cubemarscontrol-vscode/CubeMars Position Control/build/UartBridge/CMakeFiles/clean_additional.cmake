# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "/home/ianshrub/HIER/Remote_Control_Code/cubemarscontrol-vscode/CubeMars Position Control/build/Debug/CubeMars_Position_Control.map"
  )
endif()
