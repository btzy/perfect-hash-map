set(TEST_LIST
  test_phm.cpp)

# Use latest C++ by default (should be the best one), but let user override it
if(NOT DEFINED CMAKE_CXX_STANDARD)
  set (CMAKE_CXX_STANDARD 23)
endif()
  
foreach (TEST ${TEST_LIST})
  string(REPLACE "." "_" CLEAN_TEST_NAME ${TEST})
  add_executable(${CLEAN_TEST_NAME}
    ${TEST})
  target_link_libraries(${CLEAN_TEST_NAME}
    PRIVATE phm)
  target_link_libraries(${CLEAN_TEST_NAME}
    PRIVATE Catch2::Catch2WithMain)
endforeach()
