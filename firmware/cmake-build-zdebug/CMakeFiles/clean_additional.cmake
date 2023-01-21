# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "ZDebug")
  file(REMOVE_RECURSE
  "zephyr/include/generated/syscalls"
  )
endif()
