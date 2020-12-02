cmake_minimum_required(VERSION 3.17)
project(lie LANGUAGES CXX)

#------------------------------------------------------------------------------------------
# Declare dependencies, use as find_package
# find_package_wrapper(...)
find_package_wrapper(Eigen3 REQUIRED CONFIG)

#------------------------------------------------------------------------------------------
# prepare headers and sources

# define the library
add_library(lie STATIC)

# use the target_sources_wrapper, which wrap cmake native target_sources. 
# Wrapping is necessary to cover both build and installation step
# Normally headers are PUBLIC and cpp are PRIVATE, but you might have private headers too
# If the option NO_AUTO_SOURCE_GROUP is omitted the target sources and headers are 
# automatically grouped respectively in "Sources" and "Headers" folder in the IDE
# environment whereas the dependency headers (i.e. those from linked targets) are grouped
# in the "Dependency Headers" folder.
# Specify NO_AUTO_SOURCE_GROUP if you want all sources and headers grouped in the IDE
# standard folders ("Source Files" and "Header Files" in Visual Studio).
# note: you can repeat target_sources_wrapper if you want to organize your files
target_sources_wrapper(lie
	PUBLIC 
		Adjoint.h
		Constants.h
		SE2Alg.h
		SE2Exp.h
		SE2.h
		SE2Params.h
		SE3Alg.h
		SE3Exp.h
		SE3.h
		SE3Log.h
		SE3Params.h
		SkewSymm.h
		SO3Alg.h
		SO3Exp.h
		SO3.h
		SO3Log.h
		SO3Params.h
		Utils.h
    Tagged.h
	PRIVATE
		Adjoint.cpp
		Constants.cpp
		SE2Exp.cpp
		SE2Params.cpp
		SE3Exp.cpp
		SE3Log.cpp
		SE3Params.cpp
		SkewSymm.cpp
		SO3Exp.cpp
		SO3Log.cpp
		SO3Params.cpp
		Utils.cpp)

# if you do not want to have lied.{dll,lib,so} in debug disable this 
set_target_properties(lie  PROPERTIES DEBUG_POSTFIX "d")

# special compiler flags for this target
# PUBLIC will propagate do dependent, PRIVATE if they are only for this project
#if(MSVC)
#	target_compile_options(lie <<PUBLIC|PRIVATE>> /bigobj)
#endif()
#target_compile_features(lie PUBLIC cxx_std_11) # cxx_std_14

#Add an alias so that library can be used inside the build tree
add_library(lie::lie ALIAS lie)

# link libraries
target_link_libraries(lie
	PUBLIC
		Eigen3::Eigen
	PRIVATE
)

# add qt automoc per target if you need (or remove this section if qt independent)
# AUTOMOC for signal slot, AUTOUIC for User Interfaces, AUTORCC for resource files
# set_target_properties(lie PROPERTIES AUTOMOC ON)
# set_target_properties(lie PROPERTIES AUTOUIC ON)
# set_target_properties(lie PROPERTIES AUTORCC ON)

#------------------------------------------------------------------------------------------
# GLOBAL, SELF or a custom export name allowed
installAllTargets(SELF)

#------------------------------------------------------------------------------------------
# this will add the samples and tests folder (if existing) and will perfom common operations:
# - link the target to the lieVersion library, which allow to have the root include directory added
# - create folder structure for MSVC IDE
# - analyze Dll dependencies per target and globally, which in turn will be used for deep installation and PATH generation
commonEndProjectCMake(lieSol)
