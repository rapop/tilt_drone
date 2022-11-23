;; Auto-generated. Do not edit!


(when (boundp 'sensor_fusion_comm::DoubleMatrixStamped)
  (if (not (find-package "SENSOR_FUSION_COMM"))
    (make-package "SENSOR_FUSION_COMM"))
  (shadow 'DoubleMatrixStamped (find-package "SENSOR_FUSION_COMM")))
(unless (find-package "SENSOR_FUSION_COMM::DOUBLEMATRIXSTAMPED")
  (make-package "SENSOR_FUSION_COMM::DOUBLEMATRIXSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude DoubleMatrixStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass sensor_fusion_comm::DoubleMatrixStamped
  :super ros::object
  :slots (_header _rows _cols _data ))

(defmethod sensor_fusion_comm::DoubleMatrixStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:rows __rows) 0)
    ((:cols __cols) 0)
    ((:data __data) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _rows (round __rows))
   (setq _cols (round __cols))
   (setq _data __data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:rows
   (&optional __rows)
   (if __rows (setq _rows __rows)) _rows)
  (:cols
   (&optional __cols)
   (if __cols (setq _cols __cols)) _cols)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _rows
    4
    ;; int32 _cols
    4
    ;; float64[] _data
    (* 8    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _rows
       (write-long _rows s)
     ;; int32 _cols
       (write-long _cols s)
     ;; float64[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _rows
     (setq _rows (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _cols
     (setq _cols (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get sensor_fusion_comm::DoubleMatrixStamped :md5sum-) "c58f6b7538f12163211fe0ce597bbfd8")
(setf (get sensor_fusion_comm::DoubleMatrixStamped :datatype-) "sensor_fusion_comm/DoubleMatrixStamped")
(setf (get sensor_fusion_comm::DoubleMatrixStamped :definition-)
      "Header    header
int32 rows
int32 cols
float64[] data # Data stored in row-major order.

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

")



(provide :sensor_fusion_comm/DoubleMatrixStamped "c58f6b7538f12163211fe0ce597bbfd8")


