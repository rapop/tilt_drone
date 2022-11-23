;; Auto-generated. Do not edit!


(when (boundp 'sensor_fusion_comm::InitScale)
  (if (not (find-package "SENSOR_FUSION_COMM"))
    (make-package "SENSOR_FUSION_COMM"))
  (shadow 'InitScale (find-package "SENSOR_FUSION_COMM")))
(unless (find-package "SENSOR_FUSION_COMM::INITSCALE")
  (make-package "SENSOR_FUSION_COMM::INITSCALE"))
(unless (find-package "SENSOR_FUSION_COMM::INITSCALEREQUEST")
  (make-package "SENSOR_FUSION_COMM::INITSCALEREQUEST"))
(unless (find-package "SENSOR_FUSION_COMM::INITSCALERESPONSE")
  (make-package "SENSOR_FUSION_COMM::INITSCALERESPONSE"))

(in-package "ROS")





(defclass sensor_fusion_comm::InitScaleRequest
  :super ros::object
  :slots (_scale ))

(defmethod sensor_fusion_comm::InitScaleRequest
  (:init
   (&key
    ((:scale __scale) 0.0)
    )
   (send-super :init)
   (setq _scale (float __scale))
   self)
  (:scale
   (&optional __scale)
   (if __scale (setq _scale __scale)) _scale)
  (:serialization-length
   ()
   (+
    ;; float32 _scale
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _scale
       (sys::poke _scale (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _scale
     (setq _scale (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass sensor_fusion_comm::InitScaleResponse
  :super ros::object
  :slots (_result ))

(defmethod sensor_fusion_comm::InitScaleResponse
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

(defclass sensor_fusion_comm::InitScale
  :super ros::object
  :slots ())

(setf (get sensor_fusion_comm::InitScale :md5sum-) "f8ea41d14f4e256b53793fe5b7588158")
(setf (get sensor_fusion_comm::InitScale :datatype-) "sensor_fusion_comm/InitScale")
(setf (get sensor_fusion_comm::InitScale :request) sensor_fusion_comm::InitScaleRequest)
(setf (get sensor_fusion_comm::InitScale :response) sensor_fusion_comm::InitScaleResponse)

(defmethod sensor_fusion_comm::InitScaleRequest
  (:response () (instance sensor_fusion_comm::InitScaleResponse :init)))

(setf (get sensor_fusion_comm::InitScaleRequest :md5sum-) "f8ea41d14f4e256b53793fe5b7588158")
(setf (get sensor_fusion_comm::InitScaleRequest :datatype-) "sensor_fusion_comm/InitScaleRequest")
(setf (get sensor_fusion_comm::InitScaleRequest :definition-)
      "float32 scale
---
string result
")

(setf (get sensor_fusion_comm::InitScaleResponse :md5sum-) "f8ea41d14f4e256b53793fe5b7588158")
(setf (get sensor_fusion_comm::InitScaleResponse :datatype-) "sensor_fusion_comm/InitScaleResponse")
(setf (get sensor_fusion_comm::InitScaleResponse :definition-)
      "float32 scale
---
string result
")



(provide :sensor_fusion_comm/InitScale "f8ea41d14f4e256b53793fe5b7588158")


