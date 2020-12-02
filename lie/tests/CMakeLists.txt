cmake_minimum_required(VERSION 3.17)
PROJECT(lieTest  LANGUAGES CXX)

include(CTest)
find_package(Catch2 REQUIRED)
include(Catch)

# note: tests are not installed by default, 

# add these lines block for each test, or if you need more complex structure, create a subfolder
#add_executable(<<TARGET_NAME>>)
#target_sources(<<TARGET_NAME>> PRIVATE <<...>>)
#if (WIN32)
#	set_target_properties(<<TARGET_NAME>> PROPERTIES FOLDER lie/test)
#endif()
#target_link_libraries(<<TARGET_NAME>> PRIVATE <<...>> Boost::unit_test_framework)
#add_test(NAME <<TARGET_NAME>> COMMAND <<TARGET_NAME>>)

# and/or compile this list to have simple 1-file test

set(TESTS_LIST 
	SE2.cpp
)

foreach(s ${TESTS_LIST})
	get_filename_component(sn "${s}" NAME_WE)
	add_executable(lieTest_${sn})
	target_sources(lieTest_${sn} PRIVATE ${s})
	if (WIN32)
		set_target_properties(lieTest_${sn} PROPERTIES FOLDER lie/test)
	endif()	
	target_link_libraries(lieTest_${sn} PRIVATE lie::lie Catch2::Catch2WithMain )
  
  # add to tests
  catch_discover_tests(lieTest_${sn})
endforeach()
