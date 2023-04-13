# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "courseworks_answer_w1: 1 messages, 0 services")

set(MSG_I_FLAGS "-Icourseworks_answer_w1:/home/estebanp/catkin_ws/src/courseworks_answer_w1/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(courseworks_answer_w1_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/estebanp/catkin_ws/src/courseworks_answer_w1/msg/signal_msg.msg" NAME_WE)
add_custom_target(_courseworks_answer_w1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "courseworks_answer_w1" "/home/estebanp/catkin_ws/src/courseworks_answer_w1/msg/signal_msg.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(courseworks_answer_w1
  "/home/estebanp/catkin_ws/src/courseworks_answer_w1/msg/signal_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/courseworks_answer_w1
)

### Generating Services

### Generating Module File
_generate_module_cpp(courseworks_answer_w1
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/courseworks_answer_w1
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(courseworks_answer_w1_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(courseworks_answer_w1_generate_messages courseworks_answer_w1_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/estebanp/catkin_ws/src/courseworks_answer_w1/msg/signal_msg.msg" NAME_WE)
add_dependencies(courseworks_answer_w1_generate_messages_cpp _courseworks_answer_w1_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(courseworks_answer_w1_gencpp)
add_dependencies(courseworks_answer_w1_gencpp courseworks_answer_w1_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS courseworks_answer_w1_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(courseworks_answer_w1
  "/home/estebanp/catkin_ws/src/courseworks_answer_w1/msg/signal_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/courseworks_answer_w1
)

### Generating Services

### Generating Module File
_generate_module_eus(courseworks_answer_w1
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/courseworks_answer_w1
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(courseworks_answer_w1_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(courseworks_answer_w1_generate_messages courseworks_answer_w1_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/estebanp/catkin_ws/src/courseworks_answer_w1/msg/signal_msg.msg" NAME_WE)
add_dependencies(courseworks_answer_w1_generate_messages_eus _courseworks_answer_w1_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(courseworks_answer_w1_geneus)
add_dependencies(courseworks_answer_w1_geneus courseworks_answer_w1_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS courseworks_answer_w1_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(courseworks_answer_w1
  "/home/estebanp/catkin_ws/src/courseworks_answer_w1/msg/signal_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/courseworks_answer_w1
)

### Generating Services

### Generating Module File
_generate_module_lisp(courseworks_answer_w1
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/courseworks_answer_w1
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(courseworks_answer_w1_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(courseworks_answer_w1_generate_messages courseworks_answer_w1_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/estebanp/catkin_ws/src/courseworks_answer_w1/msg/signal_msg.msg" NAME_WE)
add_dependencies(courseworks_answer_w1_generate_messages_lisp _courseworks_answer_w1_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(courseworks_answer_w1_genlisp)
add_dependencies(courseworks_answer_w1_genlisp courseworks_answer_w1_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS courseworks_answer_w1_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(courseworks_answer_w1
  "/home/estebanp/catkin_ws/src/courseworks_answer_w1/msg/signal_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/courseworks_answer_w1
)

### Generating Services

### Generating Module File
_generate_module_nodejs(courseworks_answer_w1
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/courseworks_answer_w1
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(courseworks_answer_w1_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(courseworks_answer_w1_generate_messages courseworks_answer_w1_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/estebanp/catkin_ws/src/courseworks_answer_w1/msg/signal_msg.msg" NAME_WE)
add_dependencies(courseworks_answer_w1_generate_messages_nodejs _courseworks_answer_w1_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(courseworks_answer_w1_gennodejs)
add_dependencies(courseworks_answer_w1_gennodejs courseworks_answer_w1_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS courseworks_answer_w1_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(courseworks_answer_w1
  "/home/estebanp/catkin_ws/src/courseworks_answer_w1/msg/signal_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/courseworks_answer_w1
)

### Generating Services

### Generating Module File
_generate_module_py(courseworks_answer_w1
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/courseworks_answer_w1
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(courseworks_answer_w1_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(courseworks_answer_w1_generate_messages courseworks_answer_w1_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/estebanp/catkin_ws/src/courseworks_answer_w1/msg/signal_msg.msg" NAME_WE)
add_dependencies(courseworks_answer_w1_generate_messages_py _courseworks_answer_w1_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(courseworks_answer_w1_genpy)
add_dependencies(courseworks_answer_w1_genpy courseworks_answer_w1_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS courseworks_answer_w1_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/courseworks_answer_w1)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/courseworks_answer_w1
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(courseworks_answer_w1_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/courseworks_answer_w1)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/courseworks_answer_w1
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(courseworks_answer_w1_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/courseworks_answer_w1)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/courseworks_answer_w1
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(courseworks_answer_w1_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/courseworks_answer_w1)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/courseworks_answer_w1
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(courseworks_answer_w1_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/courseworks_answer_w1)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/courseworks_answer_w1\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/courseworks_answer_w1
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(courseworks_answer_w1_generate_messages_py std_msgs_generate_messages_py)
endif()
