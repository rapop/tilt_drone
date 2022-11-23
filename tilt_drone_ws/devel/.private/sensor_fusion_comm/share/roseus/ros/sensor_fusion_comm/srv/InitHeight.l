;; Auto-generated. Do not edit!


(when (boundp 'sensor_fusion_comm::InitHeight)
  (if (not (find-package "SENSOR_FUSION_COMM"))
    (make-package "SENSOR_FUSION_COMM"))
  (shadow 'InitHeight (find-package "SENSOR_FUSION_COMM")))
(unless (find-package "SENSOR_FUSION_COMM::INITHEIGHT")
  (make-package "SENSOR_FUSION_COMM::INITHEIGHT"))
(unless (find-package "SENSOR_FUSION_COMM::INITHEIGHTREQUEST")
  (make-package "SENSOR_FUSION_COMM::INITHEIGHTREQUEST"))
(unless (find-package "SENSOR_FUSION_COMM::INITHEIGHTRESPONSE")
  (make-package "SENSOR_FUSION_COMM::INITHEIGHTRESPONSE"))

(in-package "ROS")





(defclass sensor_fusion_comm::InitHeightRequest
  :super ros::object
  :slots (_height ))

(defmethod sensor_fusion_comm::InitHeightRequest
  (:init
   (&key
    ((:height __height) 0.0)
    )
   (send-super :init)
   (setq _height (float __height))
   self)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:serialization-length
   ()
   (+
    ;; float32 _height
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _height
       (sys::poke _height (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _height
     (setq _height (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass sensor_fusion_comm::InitHeightResponse
  :super ros::object
  :slots (_result ))

(defmethod sensor_fusion_comm::InitHeightResponse
  (:init
   (&key
    ((:result __result) "")
    )
   (send-super :init)
   (setq _result (string __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; string _result
    4 (length _result)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _result
       (write-long (length _result) s) (princ _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _result
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _result (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass sensor_fusion_comm::InitHeight
  :super ros::object
  :slots ())

(setf (get sensor_fusion_comm::InitHeight :md5sum-) "6b19ce060384743d60708a804f1c749f")
(setf (get sensor_fusion_comm::InitHeight :datatype-) "sensor_fusion_comm/InitHeight")
(setf (get sensor_fusion_comm::InitHeight :request) sensor_fusion_comm::InitHeightRequest)
(setf (get sensor_fusion_comm::InitHeight :response) sensor_fusion_comm::InitHeightResponse)

(defmethod sensor_fusion_comm::InitHeightRequest
  (:response () (instance sensor_fusion_comm::InitHeightResponse :init)))

(setf (get sensor_fusion_comm::InitHeightRequest :md5sum-) "6b19ce060384743d60708a804f1c749f")
(setf (get sensor_fusion_comm::InitHeightRequest :datatype-) "sensor_fusion_comm/InitHeightRequest")
(setf (get sensor_fusion_comm::InitHeightRequest :definition-)
      "float32 height
---
string result
")

(setf (get sensor_fusion_comm::InitHeightResponse :md5sum-) "6b19ce060384743d60708a804f1c749f")
(setf (get sensor_fusion_comm::InitHeightResponse :datatype-) "sensor_fusion_comm/InitHeightResponse")
(setf (get sensor_fusion_comm::InitHeightResponse :definition-)
      "float32 height
---
string result
")



(provide :sensor_fusion_comm/InitHeight "6b19ce060384743d60708a804f1c749f")


