find_package(PkgConfig)
pkg_check_modules(roboptim-core roboptim-core REQUIRED)

target_include_directories(${target} PUBLIC ${roboptim-core_INCLUDE_DIRS})
target_link_libraries(${target} ${roboptim-core_LIBRARIES})