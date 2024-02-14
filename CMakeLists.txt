cmake_minimum_required(VERSION 3.5)
project(gymfc_plugins)

add_subdirectory(gymfc/envs/assets/gazebo/plugins ${CMAKE_CURRENT_BINARY_DIR}/gymfc_plugins)
add_subdirectory(examples/gymfc_nf/twins/nf1/gymfc-aircraft-plugins ${CMAKE_CURRENT_BINARY_DIR}/gymfc_nf1_aircraft_plugins)

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/gymfc/envs/assets/gazebo/plugins/build)

function(print_all_targets DIR)
    get_property(TGTS DIRECTORY "${DIR}" PROPERTY BUILDSYSTEM_TARGETS)
    foreach(TGT IN LISTS TGTS)
        #message(STATUS "Target: ${TGT}")
        install(TARGETS ${TGT} DESTINATION ${CMAKE_INSTALL_PREFIX})
        message("TARGET="${TGT}" WILL INSTALL TO ${CMAKE_INSTALL_PREFIX}")
        # TODO: Do something about it
    endforeach()

    get_property(SUBDIRS DIRECTORY "${DIR}" PROPERTY SUBDIRECTORIES)
    foreach(SUBDIR IN LISTS SUBDIRS)
        print_all_targets("${SUBDIR}")
    endforeach()
endfunction()

print_all_targets(.)

