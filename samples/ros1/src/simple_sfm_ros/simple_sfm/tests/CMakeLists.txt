add_executable(simple_sfm_types_test types_test.cpp)
target_link_libraries(simple_sfm_types_test PRIVATE Catch2::Catch2WithMain simple_sfm)
add_test(NAME simple_sfm_types_test COMMAND simple_sfm_types_test)

add_executable(simple_sfm_utils_test utils_test.cpp)
target_link_libraries(simple_sfm_utils_test PRIVATE Catch2::Catch2WithMain simple_sfm)
target_compile_definitions(simple_sfm_utils_test
                           PRIVATE TEST_BAL_DATA="${CMAKE_CURRENT_SOURCE_DIR}/test_data/problem-21-11315-pre.txt"
)
add_test(NAME simple_sfm_utils_test COMMAND simple_sfm_utils_test)
