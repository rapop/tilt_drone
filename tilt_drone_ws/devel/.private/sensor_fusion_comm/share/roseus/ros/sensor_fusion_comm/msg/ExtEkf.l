;; Auto-generated. Do not edit!


(when (boundp 'sensor_fusion_comm::ExtEkf)
  (if (not (find-package "SENSOR_FUSION_COMM"))
    (make-package "SENSOR_FUSION_COMM"))
  (shadow 'ExtEkf (find-package "SENSOR_FUSION_COMM")))
(unless (find-package "SENSOR_FUSION_COMM::EXTEKF")
  (make-package "SENSOR_FUSION_COMM::EXTEKF"))

(in-package "ROS")
;;//! \htmlinclude ExtEkf.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*IGNORE_STATE*" (find-package "SENSOR_FUSION_COMM::EXTEKF"))
(shadow '*IGNORE_STATE* (find-package "SENSOR_FUSION_COMM::EXTEKF"))
(defconstant sensor_fusion_comm::ExtEkf::*IGNORE_STATE* 0)
(intern "*CURRENT_STATE*" (find-package "SENSOR_FUSION_COMM::EXTEKF"))
(shadow '*CURRENT_STATE* (find-package "SENSOR_FUSION_COMM::EXTEKF"))
(defconstant sensor_fusion_comm::ExtEkf::*CURRENT_STATE* 1)
(intern "*INITIALIZATION*" (find-package "SENSOR_FUSION_COMM::EXTEKF"))
(shadow '*INITIALIZATION* (find-package "SENSOR_FUSION_COMM::EXTEKF"))
(defconstant sensor_fusion_comm::ExtEkf::*INITIALIZATION* 2)
(intern "*STATE_CORRECTION*" (find-package "SENSOR_FUSION_COMM::EXTEKF"))
(shadow '*STATE_CORRECTION* (find-package "SENSOR_FUSION_COMM::EXTEKF"))
(defconstant sensor_fusion_comm::ExtEkf::*STATE_CORRECTION* 3)
(defclass sensor_fusion_comm::ExtEkf
  :super ros::object
  :slots (_header _angular_velocity _linear_acceleration _state _flag ))

(defmethod sensor_fusion_comm::ExtEkf
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:angular_velocity __angular_velocity) (instance geometry_msgs::Vector3 :init))
    ((:linear_acceleration __linear_acceleration) (instance geometry_msgs::Vector3 :init))
    ((:state __state) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:flag __flag) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _angular_velocity __angular_velocity)
   (setq _linear_acceleration __linear_acceleration)
   (setq _state __state)
   (setq _flag (round __flag))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:angular_velocity
   (&rest __angular_velocity)
   (if (keywordp (car __angular_velocity))
       (send* _angular_velocity __angular_velocity)
     (progn
       (if __angular_velocity (setq _angular_velocity (car __angular_velocity)))
       _angular_velocity)))
  (:linear_acceleration
   (&rest __linear_acceleration)
   (if (keywordp (car __linear_acceleration))
       (send* _linear_acceleration __linear_acceleration)
     (progn
       (if __linear_acceleration (setq _linear_acceleration (car __linear_acceleration)))
       _linear_acceleration)))
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:flag
   (&optional __flag)
   (if __flag (setq _flag __flag)) _flag)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Vector3 _angular_velocity
    (send _angular_velocity :serialization-length)
    ;; geometry_msgs/Vector3 _linear_acceleration
    (send _linear_acceleration :serialization-length)
    ;; float32[] _state
    (* 4    (length _state)) 4
    ;; int32 _flag
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Vector3 _angular_velocity
       (send _angular_velocity :serialize s)
     ;; geometry_msgs/Vector3 _linear_acceleration
       (send _linear_acceleration :serialize s)
     ;; float32[] _state
     (write-long (length _state) s)
     (dotimes (i (length _state))
       (sys::poke (elt _state i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int32 _flag
       (write-long _flag s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Vector3 _angular_velocity
     (send _angular_velocity :deserialize buf ptr-) (incf ptr- (send _angular_velocity :serialization-length))
   ;; geometry_msgs/Vector3 _linear_acceleration
     (send _linear_acceleration :deserialize buf ptr-) (incf ptr- (send _linear_acceleration :serialization-length))
   ;; float32[] _state
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _state (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _state i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; int32 _flag
     (setq _flag (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get sensor_fusion_comm::ExtEkf :md5sum-) "0710aec5a114bc946c3d7687c8e5a0bd")
(setf (get sensor_fusion_comm::ExtEkf :datatype-) "sensor_fusion_comm/ExtEkf")
(setf (get sensor_fusion_comm::ExtEkf :definition-)
      "Header header
geometry_msgs/Vector3       angular_velocity
geometry_msgs/Vector3       linear_acceleration
float32[]                   state
int32                       flag

uint32                      ignore_state = 0
uint32                      current_state = 1
uint32                      initialization = 2
uint32                      state_correction = 3

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :sensor_fusion_comm/ExtEkf "0710aec5a114bc946c3d7687c8e5a0bd")


