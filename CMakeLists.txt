cmake_minimum_required(VERSION 3.0)

project (offsetConstraint) 

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
add_compile_options(/W0)

find_package( Maya REQUIRED )

set(source
    source/offsetConstraint.cpp 
    source/offsetConstraint.h
    source/main.cpp)

add_library(offsetConstraint SHARED ${source})

target_include_directories(offsetConstraint PUBLIC 
  ${MAYA_INCLUDE_DIR})

target_link_libraries(offsetConstraint  
  ${MAYA_Foundation_LIBRARY} 
  ${MAYA_OpenMaya_LIBRARY} 
  ${MAYA_OpenMayaAnim_LIBRARY} 
  ${MAYA_OpenMayaUI_LIBRARY})

set_target_properties( offsetConstraint PROPERTIES OUTPUT_NAME offsetConstraint CLEAN_DIRECT_OUTPUT 1 SUFFIX ".mll" CXX_STANDARD 17)

MAYA_SET_PLUGIN_PROPERTIES( offsetConstraint )

install(TARGETS offsetConstraint RUNTIME DESTINATION .)

