# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ecmc: 7 messages, 0 services")

set(MSG_I_FLAGS "-Iecmc:/home/joannadiao/kaiROS/src/ecmc/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ecmc_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/can_comms_data_msg.msg" NAME_WE)
add_custom_target(_ecmc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ecmc" "/home/joannadiao/kaiROS/src/ecmc/msg/can_comms_data_msg.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/sudo_driver_input_msg.msg" NAME_WE)
add_custom_target(_ecmc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ecmc" "/home/joannadiao/kaiROS/src/ecmc/msg/sudo_driver_input_msg.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/raw_sensor_object_data_msg.msg" NAME_WE)
add_custom_target(_ecmc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ecmc" "/home/joannadiao/kaiROS/src/ecmc/msg/raw_sensor_object_data_msg.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/drive_control_input_msg.msg" NAME_WE)
add_custom_target(_ecmc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ecmc" "/home/joannadiao/kaiROS/src/ecmc/msg/drive_control_input_msg.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_flag_msg.msg" NAME_WE)
add_custom_target(_ecmc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ecmc" "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_flag_msg.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/fused_object_data_msg.msg" NAME_WE)
add_custom_target(_ecmc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ecmc" "/home/joannadiao/kaiROS/src/ecmc/msg/fused_object_data_msg.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_data_msg.msg" NAME_WE)
add_custom_target(_ecmc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ecmc" "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_data_msg.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/can_comms_data_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ecmc
)
_generate_msg_cpp(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/sudo_driver_input_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ecmc
)
_generate_msg_cpp(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/raw_sensor_object_data_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ecmc
)
_generate_msg_cpp(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/drive_control_input_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ecmc
)
_generate_msg_cpp(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_flag_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ecmc
)
_generate_msg_cpp(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/fused_object_data_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ecmc
)
_generate_msg_cpp(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_data_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ecmc
)

### Generating Services

### Generating Module File
_generate_module_cpp(ecmc
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ecmc
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ecmc_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ecmc_generate_messages ecmc_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/can_comms_data_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_cpp _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/sudo_driver_input_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_cpp _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/raw_sensor_object_data_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_cpp _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/drive_control_input_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_cpp _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_flag_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_cpp _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/fused_object_data_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_cpp _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_data_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_cpp _ecmc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ecmc_gencpp)
add_dependencies(ecmc_gencpp ecmc_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ecmc_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/can_comms_data_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ecmc
)
_generate_msg_eus(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/sudo_driver_input_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ecmc
)
_generate_msg_eus(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/raw_sensor_object_data_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ecmc
)
_generate_msg_eus(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/drive_control_input_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ecmc
)
_generate_msg_eus(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_flag_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ecmc
)
_generate_msg_eus(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/fused_object_data_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ecmc
)
_generate_msg_eus(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_data_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ecmc
)

### Generating Services

### Generating Module File
_generate_module_eus(ecmc
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ecmc
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ecmc_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ecmc_generate_messages ecmc_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/can_comms_data_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_eus _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/sudo_driver_input_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_eus _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/raw_sensor_object_data_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_eus _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/drive_control_input_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_eus _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_flag_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_eus _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/fused_object_data_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_eus _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_data_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_eus _ecmc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ecmc_geneus)
add_dependencies(ecmc_geneus ecmc_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ecmc_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/can_comms_data_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ecmc
)
_generate_msg_lisp(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/sudo_driver_input_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ecmc
)
_generate_msg_lisp(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/raw_sensor_object_data_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ecmc
)
_generate_msg_lisp(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/drive_control_input_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ecmc
)
_generate_msg_lisp(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_flag_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ecmc
)
_generate_msg_lisp(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/fused_object_data_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ecmc
)
_generate_msg_lisp(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_data_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ecmc
)

### Generating Services

### Generating Module File
_generate_module_lisp(ecmc
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ecmc
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ecmc_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ecmc_generate_messages ecmc_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/can_comms_data_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_lisp _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/sudo_driver_input_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_lisp _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/raw_sensor_object_data_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_lisp _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/drive_control_input_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_lisp _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_flag_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_lisp _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/fused_object_data_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_lisp _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_data_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_lisp _ecmc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ecmc_genlisp)
add_dependencies(ecmc_genlisp ecmc_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ecmc_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/can_comms_data_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ecmc
)
_generate_msg_nodejs(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/sudo_driver_input_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ecmc
)
_generate_msg_nodejs(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/raw_sensor_object_data_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ecmc
)
_generate_msg_nodejs(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/drive_control_input_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ecmc
)
_generate_msg_nodejs(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_flag_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ecmc
)
_generate_msg_nodejs(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/fused_object_data_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ecmc
)
_generate_msg_nodejs(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_data_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ecmc
)

### Generating Services

### Generating Module File
_generate_module_nodejs(ecmc
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ecmc
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ecmc_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ecmc_generate_messages ecmc_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/can_comms_data_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_nodejs _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/sudo_driver_input_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_nodejs _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/raw_sensor_object_data_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_nodejs _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/drive_control_input_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_nodejs _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_flag_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_nodejs _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/fused_object_data_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_nodejs _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_data_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_nodejs _ecmc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ecmc_gennodejs)
add_dependencies(ecmc_gennodejs ecmc_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ecmc_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/can_comms_data_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ecmc
)
_generate_msg_py(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/sudo_driver_input_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ecmc
)
_generate_msg_py(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/raw_sensor_object_data_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ecmc
)
_generate_msg_py(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/drive_control_input_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ecmc
)
_generate_msg_py(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_flag_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ecmc
)
_generate_msg_py(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/fused_object_data_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ecmc
)
_generate_msg_py(ecmc
  "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_data_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ecmc
)

### Generating Services

### Generating Module File
_generate_module_py(ecmc
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ecmc
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ecmc_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ecmc_generate_messages ecmc_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/can_comms_data_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_py _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/sudo_driver_input_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_py _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/raw_sensor_object_data_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_py _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/drive_control_input_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_py _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_flag_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_py _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/fused_object_data_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_py _ecmc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joannadiao/kaiROS/src/ecmc/msg/sensor_diagnostic_data_msg.msg" NAME_WE)
add_dependencies(ecmc_generate_messages_py _ecmc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ecmc_genpy)
add_dependencies(ecmc_genpy ecmc_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ecmc_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ecmc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ecmc
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ecmc_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ecmc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ecmc
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ecmc_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ecmc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ecmc
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ecmc_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ecmc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ecmc
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ecmc_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ecmc)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ecmc\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ecmc
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ecmc_generate_messages_py std_msgs_generate_messages_py)
endif()
