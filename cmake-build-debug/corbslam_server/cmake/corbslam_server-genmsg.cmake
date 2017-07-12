# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "corbslam_server: 1 messages, 2 services")

set(MSG_I_FLAGS "-Icorbslam_server:/home/lifu/catkin_ws/src/corbslam_server/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(corbslam_server_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lifu/catkin_ws/src/corbslam_server/msg/corbslam_message.msg" NAME_WE)
add_custom_target(_corbslam_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "corbslam_server" "/home/lifu/catkin_ws/src/corbslam_server/msg/corbslam_message.msg" ""
)

get_filename_component(_filename "/home/lifu/catkin_ws/src/corbslam_server/srv/corbslam_insert.srv" NAME_WE)
add_custom_target(_corbslam_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "corbslam_server" "/home/lifu/catkin_ws/src/corbslam_server/srv/corbslam_insert.srv" ""
)

get_filename_component(_filename "/home/lifu/catkin_ws/src/corbslam_server/srv/corbslam_update.srv" NAME_WE)
add_custom_target(_corbslam_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "corbslam_server" "/home/lifu/catkin_ws/src/corbslam_server/srv/corbslam_update.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(corbslam_server
  "/home/lifu/catkin_ws/src/corbslam_server/msg/corbslam_message.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/corbslam_server
)

### Generating Services
_generate_srv_cpp(corbslam_server
  "/home/lifu/catkin_ws/src/corbslam_server/srv/corbslam_insert.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/corbslam_server
)
_generate_srv_cpp(corbslam_server
  "/home/lifu/catkin_ws/src/corbslam_server/srv/corbslam_update.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/corbslam_server
)

### Generating Module File
_generate_module_cpp(corbslam_server
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/corbslam_server
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(corbslam_server_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(corbslam_server_generate_messages corbslam_server_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lifu/catkin_ws/src/corbslam_server/msg/corbslam_message.msg" NAME_WE)
add_dependencies(corbslam_server_generate_messages_cpp _corbslam_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lifu/catkin_ws/src/corbslam_server/srv/corbslam_insert.srv" NAME_WE)
add_dependencies(corbslam_server_generate_messages_cpp _corbslam_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lifu/catkin_ws/src/corbslam_server/srv/corbslam_update.srv" NAME_WE)
add_dependencies(corbslam_server_generate_messages_cpp _corbslam_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(corbslam_server_gencpp)
add_dependencies(corbslam_server_gencpp corbslam_server_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS corbslam_server_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(corbslam_server
  "/home/lifu/catkin_ws/src/corbslam_server/msg/corbslam_message.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/corbslam_server
)

### Generating Services
_generate_srv_lisp(corbslam_server
  "/home/lifu/catkin_ws/src/corbslam_server/srv/corbslam_insert.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/corbslam_server
)
_generate_srv_lisp(corbslam_server
  "/home/lifu/catkin_ws/src/corbslam_server/srv/corbslam_update.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/corbslam_server
)

### Generating Module File
_generate_module_lisp(corbslam_server
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/corbslam_server
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(corbslam_server_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(corbslam_server_generate_messages corbslam_server_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lifu/catkin_ws/src/corbslam_server/msg/corbslam_message.msg" NAME_WE)
add_dependencies(corbslam_server_generate_messages_lisp _corbslam_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lifu/catkin_ws/src/corbslam_server/srv/corbslam_insert.srv" NAME_WE)
add_dependencies(corbslam_server_generate_messages_lisp _corbslam_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lifu/catkin_ws/src/corbslam_server/srv/corbslam_update.srv" NAME_WE)
add_dependencies(corbslam_server_generate_messages_lisp _corbslam_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(corbslam_server_genlisp)
add_dependencies(corbslam_server_genlisp corbslam_server_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS corbslam_server_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(corbslam_server
  "/home/lifu/catkin_ws/src/corbslam_server/msg/corbslam_message.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/corbslam_server
)

### Generating Services
_generate_srv_py(corbslam_server
  "/home/lifu/catkin_ws/src/corbslam_server/srv/corbslam_insert.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/corbslam_server
)
_generate_srv_py(corbslam_server
  "/home/lifu/catkin_ws/src/corbslam_server/srv/corbslam_update.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/corbslam_server
)

### Generating Module File
_generate_module_py(corbslam_server
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/corbslam_server
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(corbslam_server_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(corbslam_server_generate_messages corbslam_server_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lifu/catkin_ws/src/corbslam_server/msg/corbslam_message.msg" NAME_WE)
add_dependencies(corbslam_server_generate_messages_py _corbslam_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lifu/catkin_ws/src/corbslam_server/srv/corbslam_insert.srv" NAME_WE)
add_dependencies(corbslam_server_generate_messages_py _corbslam_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lifu/catkin_ws/src/corbslam_server/srv/corbslam_update.srv" NAME_WE)
add_dependencies(corbslam_server_generate_messages_py _corbslam_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(corbslam_server_genpy)
add_dependencies(corbslam_server_genpy corbslam_server_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS corbslam_server_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/corbslam_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/corbslam_server
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(corbslam_server_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/corbslam_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/corbslam_server
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(corbslam_server_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/corbslam_server)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/corbslam_server\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/corbslam_server
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(corbslam_server_generate_messages_py std_msgs_generate_messages_py)
