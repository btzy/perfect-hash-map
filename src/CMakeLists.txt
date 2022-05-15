set(TARGET_NAME phm)

set(PUBLIC_HEADER_FILES
  phm/phm.hpp
  phm/map.hpp)

# set(SOURCE_FILES ${PUBLIC_HEADER_FILES} tmp.cpp)

# Define the library
add_library(${TARGET_NAME} INTERFACE)

target_include_directories(${TARGET_NAME} INTERFACE .)

set_target_properties(${TARGET_NAME} PROPERTIES PUBLIC_HEADER "${PUBLIC_HEADER_FILES}")

install(TARGETS ${TARGET_NAME} PUBLIC_HEADER DESTINATION .)
