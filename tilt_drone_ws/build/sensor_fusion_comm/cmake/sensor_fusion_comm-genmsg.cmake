# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "sensor_fusion_comm: 5 messages, 2 services")

set(MSG_I_FLAGS "-Isensor_fusion_comm:/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(sensor_fusion_comm_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleArrayStamped.msg" NAME_WE)
add_custom_target(_sensor_fusion_comm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sensor_fusion_comm" "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleArrayStamped.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtEkf.msg" NAME_WE)
add_custom_target(_sensor_fusion_comm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sensor_fusion_comm" "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtEkf.msg" "std_msgs/Header:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtState.msg" NAME_WE)
add_custom_target(_sensor_fusion_comm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sensor_fusion_comm" "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtState.msg" "geometry_msgs/Vector3:geometry_msgs/Quaternion:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitScale.srv" NAME_WE)
add_custom_target(_sensor_fusion_comm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sensor_fusion_comm" "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitScale.srv" ""
)

get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitHeight.srv" NAME_WE)
add_custom_target(_sensor_fusion_comm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sensor_fusion_comm" "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitHeight.srv" ""
)

get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/PointWithCovarianceStamped.msg" NAME_WE)
add_custom_target(_sensor_fusion_comm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sensor_fusion_comm" "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/PointWithCovarianceStamped.msg" "std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleMatrixStamped.msg" NAME_WE)
add_custom_target(_sensor_fusion_comm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sensor_fusion_comm" "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleMatrixStamped.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sensor_fusion_comm
)
_generate_msg_cpp(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/PointWithCovarianceStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sensor_fusion_comm
)
_generate_msg_cpp(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sensor_fusion_comm
)
_generate_msg_cpp(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleMatrixStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sensor_fusion_comm
)
_generate_msg_cpp(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtEkf.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sensor_fusion_comm
)

### Generating Services
_generate_srv_cpp(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitScale.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sensor_fusion_comm
)
_generate_srv_cpp(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitHeight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sensor_fusion_comm
)

### Generating Module File
_generate_module_cpp(sensor_fusion_comm
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sensor_fusion_comm
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(sensor_fusion_comm_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(sensor_fusion_comm_generate_messages sensor_fusion_comm_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleArrayStamped.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_cpp _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtEkf.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_cpp _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtState.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_cpp _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitScale.srv" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_cpp _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitHeight.srv" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_cpp _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/PointWithCovarianceStamped.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_cpp _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleMatrixStamped.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_cpp _sensor_fusion_comm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sensor_fusion_comm_gencpp)
add_dependencies(sensor_fusion_comm_gencpp sensor_fusion_comm_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sensor_fusion_comm_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sensor_fusion_comm
)
_generate_msg_eus(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/PointWithCovarianceStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sensor_fusion_comm
)
_generate_msg_eus(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sensor_fusion_comm
)
_generate_msg_eus(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleMatrixStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sensor_fusion_comm
)
_generate_msg_eus(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtEkf.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sensor_fusion_comm
)

### Generating Services
_generate_srv_eus(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitScale.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sensor_fusion_comm
)
_generate_srv_eus(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitHeight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sensor_fusion_comm
)

### Generating Module File
_generate_module_eus(sensor_fusion_comm
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sensor_fusion_comm
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(sensor_fusion_comm_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(sensor_fusion_comm_generate_messages sensor_fusion_comm_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleArrayStamped.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_eus _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtEkf.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_eus _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtState.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_eus _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitScale.srv" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_eus _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitHeight.srv" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_eus _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/PointWithCovarianceStamped.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_eus _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleMatrixStamped.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_eus _sensor_fusion_comm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sensor_fusion_comm_geneus)
add_dependencies(sensor_fusion_comm_geneus sensor_fusion_comm_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sensor_fusion_comm_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sensor_fusion_comm
)
_generate_msg_lisp(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/PointWithCovarianceStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sensor_fusion_comm
)
_generate_msg_lisp(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sensor_fusion_comm
)
_generate_msg_lisp(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleMatrixStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sensor_fusion_comm
)
_generate_msg_lisp(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtEkf.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sensor_fusion_comm
)

### Generating Services
_generate_srv_lisp(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitScale.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sensor_fusion_comm
)
_generate_srv_lisp(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitHeight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sensor_fusion_comm
)

### Generating Module File
_generate_module_lisp(sensor_fusion_comm
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sensor_fusion_comm
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(sensor_fusion_comm_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(sensor_fusion_comm_generate_messages sensor_fusion_comm_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleArrayStamped.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_lisp _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtEkf.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_lisp _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtState.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_lisp _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitScale.srv" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_lisp _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitHeight.srv" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_lisp _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/PointWithCovarianceStamped.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_lisp _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleMatrixStamped.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_lisp _sensor_fusion_comm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sensor_fusion_comm_genlisp)
add_dependencies(sensor_fusion_comm_genlisp sensor_fusion_comm_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sensor_fusion_comm_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sensor_fusion_comm
)
_generate_msg_nodejs(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/PointWithCovarianceStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sensor_fusion_comm
)
_generate_msg_nodejs(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sensor_fusion_comm
)
_generate_msg_nodejs(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleMatrixStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sensor_fusion_comm
)
_generate_msg_nodejs(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtEkf.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sensor_fusion_comm
)

### Generating Services
_generate_srv_nodejs(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitScale.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sensor_fusion_comm
)
_generate_srv_nodejs(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitHeight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sensor_fusion_comm
)

### Generating Module File
_generate_module_nodejs(sensor_fusion_comm
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sensor_fusion_comm
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(sensor_fusion_comm_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(sensor_fusion_comm_generate_messages sensor_fusion_comm_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleArrayStamped.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_nodejs _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtEkf.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_nodejs _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtState.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_nodejs _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitScale.srv" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_nodejs _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitHeight.srv" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_nodejs _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/PointWithCovarianceStamped.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_nodejs _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleMatrixStamped.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_nodejs _sensor_fusion_comm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sensor_fusion_comm_gennodejs)
add_dependencies(sensor_fusion_comm_gennodejs sensor_fusion_comm_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sensor_fusion_comm_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sensor_fusion_comm
)
_generate_msg_py(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/PointWithCovarianceStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sensor_fusion_comm
)
_generate_msg_py(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sensor_fusion_comm
)
_generate_msg_py(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleMatrixStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sensor_fusion_comm
)
_generate_msg_py(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtEkf.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sensor_fusion_comm
)

### Generating Services
_generate_srv_py(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitScale.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sensor_fusion_comm
)
_generate_srv_py(sensor_fusion_comm
  "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitHeight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sensor_fusion_comm
)

### Generating Module File
_generate_module_py(sensor_fusion_comm
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sensor_fusion_comm
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(sensor_fusion_comm_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(sensor_fusion_comm_generate_messages sensor_fusion_comm_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleArrayStamped.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_py _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtEkf.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_py _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/ExtState.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_py _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitScale.srv" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_py _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/srv/InitHeight.srv" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_py _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/PointWithCovarianceStamped.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_py _sensor_fusion_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/radu/tiltUp3_ws/src/ethzasl_msf/sensor_fusion_comm/msg/DoubleMatrixStamped.msg" NAME_WE)
add_dependencies(sensor_fusion_comm_generate_messages_py _sensor_fusion_comm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sensor_fusion_comm_genpy)
add_dependencies(sensor_fusion_comm_genpy sensor_fusion_comm_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sensor_fusion_comm_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sensor_fusion_comm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sensor_fusion_comm
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(sensor_fusion_comm_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(sensor_fusion_comm_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sensor_fusion_comm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sensor_fusion_comm
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(sensor_fusion_comm_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(sensor_fusion_comm_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sensor_fusion_comm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sensor_fusion_comm
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(sensor_fusion_comm_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(sensor_fusion_comm_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sensor_fusion_comm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sensor_fusion_comm
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(sensor_fusion_comm_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(sensor_fusion_comm_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sensor_fusion_comm)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sensor_fusion_comm\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sensor_fusion_comm
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(sensor_fusion_comm_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(sensor_fusion_comm_generate_messages_py std_msgs_generate_messages_py)
endif()
