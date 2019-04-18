project(contacts_opt)

# CMake minimum version and policies
cmake_minimum_required(VERSION 3.0.0)


# Add cmake dir and include custom macros
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroYCMInstallLibrary)

find_package(XCM REQUIRED)
find_package(OpenSoT REQUIRED)
find_package(PkgConfig)

find_package(ifopt REQUIRED)

find_package(XBotInterface REQUIRED)
find_package(XCM QUIET)
find_package(OpenSoT REQUIRED)
find_package(ReflexxesTypeII REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        std_msgs
                                        geometry_msgs
                                        sensor_msgs)
                                        
                                        
catkin_package(
   INCLUDE_DIRS include 

)

find_package(cartesian_interface REQUIRED)

include_directories(include
                    ${OpenSoT_INCLUDE_DIRS}
                    ${XBotInterface_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${ReflexxesTypeII_INCLUDE_DIRS}
                    ${cartesian_interface_INCLUDE_DIRS}
                    )

set(ENABLE_XENO CACHE BOOL "ON")
if ( ${ENABLE_XENO} )
   find_package(Xenomai)
   find_package(RTnet)
endif()

# C++ 11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


# find packages

# include
include_directories(include
                    ${XCM_INCLUDE_DIRS} ${OpenSoT_INCLUDE_DIRS} ${trajectory_utils_INCLUDE_DIRS}  /usr/include/eigen3
                    )

###########
## Build ##
###########
add_executable(ifopt_contacts_node src/ifopt_contacts_node.cpp)
add_executable(forza_giusta_node src/ForzaGiustaFeedForward.cpp)
add_executable(leg_lift_node src/leg_lift_node.cpp)

set_target_properties(ifopt_contacts_node PROPERTIES COMPILE_FLAGS -std=c++14)

add_executable(ifopt_posture_opt src/ifopt_posture_opt.cpp)


##########
## Link ##
##########
target_link_libraries(ifopt_contacts_node ${OpenSoT_LIBRARIES}
                                          ${XBotInterface_LIBRARIES}
                                          ${catkin_LIBRARIES}
                                          ${ReflexxesTypeII_LIBRARIES}
                                          ${cartesian_interface_LIBRARIES}  
                                          ifopt::ifopt_ipopt)
                                          
target_link_libraries(ifopt_posture_opt ${OpenSoT_LIBRARIES}
                                          ${XBotInterface_LIBRARIES}
                                          ${catkin_LIBRARIES}
                                          ${ReflexxesTypeII_LIBRARIES}
                                          ${cartesian_interface_LIBRARIES}  
                                          ifopt::ifopt_ipopt)
                                          
target_link_libraries(forza_giusta_node ${OpenSoT_LIBRARIES}
                                        ${XBotInterface_LIBRARIES}
                                        ${catkin_LIBRARIES}
                                        ${ReflexxesTypeII_LIBRARIES}
                                        ${cartesian_interface_LIBRARIES})
                                        
target_link_libraries(leg_lift_node ${OpenSoT_LIBRARIES}
                                          ${XBotInterface_LIBRARIES}
                                          ${catkin_LIBRARIES}
                                          ${ReflexxesTypeII_LIBRARIES}
                                          ${cartesian_interface_LIBRARIES})                                        

#############
## Install ##
#############

install(TARGETS ifopt_contacts_node ifopt_contacts_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS forza_giusta_node forza_giusta_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS leg_lift_node leg_lift_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/ifopt_problem/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
			  					  					  
## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY problem/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/problem
  FILES_MATCHING PATTERN "*.yaml"
)


