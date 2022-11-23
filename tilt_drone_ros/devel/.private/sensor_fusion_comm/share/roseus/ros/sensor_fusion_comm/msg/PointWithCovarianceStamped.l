;; Auto-generated. Do not edit!


(when (boundp 'sensor_fusion_comm::PointWithCovarianceStamped)
  (if (not (find-package "SENSOR_FUSION_COMM"))
    (make-package "SENSOR_FUSION_COMM"))
  (shadow 'PointWithCovarianceStamped (find-package "SENSOR_FUSION_COMM")))
(unless (find-package "SENSOR_FUSION_COMM::POINTWITHCOVARIANCESTAMPED")
  (make-package "SENSOR_FUSION_COMM::POINTWITHCOVARIANCESTAMPED"))

(in-package "ROS")
;;//! \htmlinclude PointWithCovarianceStamped.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass sensor_fusion_comm::PointWithCovarianceStamped
  :super ros::object
  :slots (_header _point _covariance ))

(defmethod sensor_fusion_comm::PointWithCovarianceStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:point __point) (instance geometry_msgs::Point :init))
    ((:covariance __covariance) (make-array 9 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _point __point)
   (setq _covariance __covariance)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:covariance
   (&optional __covariance)
   (if __covariance (setq _covariance __covariance)) _covariance)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point _point
    (send _point :serialization-length)
    ;; float64[9] _covariance
    (* 8    9)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point _point
       (send _point :serialize s)
     ;; float64[9] _covariance
     (dotimes (i 9)
       (sys::poke (elt _covariance i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;; float64[9] _covariance
   (dotimes (i (length _covariance))
     (setf (elt _covariance i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get sensor_fusion_comm::PointWithCovarianceStamped :md5sum-) "0702610ecb647cea3bb121e1743a30e7")
(setf (get sensor_fusion_comm::PointWithCovarianceStamped :datatype-) "sensor_fusion_comm/PointWithCovarianceStamped")
(setf (get sensor_fusion_comm::PointWithCovarianceStamped :definition-)
      "Header header
geometry_msgs/Point point
float64[9] covariance
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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :sensor_fusion_comm/PointWithCovarianceStamped "0702610ecb647cea3bb121e1743a30e7")


