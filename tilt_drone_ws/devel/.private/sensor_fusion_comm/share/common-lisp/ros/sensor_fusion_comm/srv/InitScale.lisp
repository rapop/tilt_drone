; Auto-generated. Do not edit!


(cl:in-package sensor_fusion_comm-srv)


;//! \htmlinclude InitScale-request.msg.html

(cl:defclass <InitScale-request> (roslisp-msg-protocol:ros-message)
  ((scale
    :reader scale
    :initarg :scale
    :type cl:float
    :initform 0.0))
)

(cl:defclass InitScale-request (<InitScale-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InitScale-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InitScale-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_fusion_comm-srv:<InitScale-request> is deprecated: use sensor_fusion_comm-srv:InitScale-request instead.")))

(cl:ensure-generic-function 'scale-val :lambda-list '(m))
(cl:defmethod scale-val ((m <InitScale-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_fusion_comm-srv:scale-val is deprecated.  Use sensor_fusion_comm-srv:scale instead.")
  (scale m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InitScale-request>) ostream)
  "Serializes a message object of type '<InitScale-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'scale))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InitScale-request>) istream)
  "Deserializes a message object of type '<InitScale-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'scale) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InitScale-request>)))
  "Returns string type for a service object of type '<InitScale-request>"
  "sensor_fusion_comm/InitScaleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitScale-request)))
  "Returns string type for a service object of type 'InitScale-request"
  "sensor_fusion_comm/InitScaleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InitScale-request>)))
  "Returns md5sum for a message object of type '<InitScale-request>"
  "f8ea41d14f4e256b53793fe5b7588158")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InitScale-request)))
  "Returns md5sum for a message object of type 'InitScale-request"
  "f8ea41d14f4e256b53793fe5b7588158")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InitScale-request>)))
  "Returns full string definition for message of type '<InitScale-request>"
  (cl:format cl:nil "float32 scale~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InitScale-request)))
  "Returns full string definition for message of type 'InitScale-request"
  (cl:format cl:nil "float32 scale~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InitScale-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InitScale-request>))
  "Converts a ROS message object to a list"
  (cl:list 'InitScale-request
    (cl:cons ':scale (scale msg))
))
;//! \htmlinclude InitScale-response.msg.html

(cl:defclass <InitScale-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform ""))
)

(cl:defclass InitScale-response (<InitScale-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InitScale-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InitScale-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_fusion_comm-srv:<InitScale-response> is deprecated: use sensor_fusion_comm-srv:InitScale-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <InitScale-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_fusion_comm-srv:result-val is deprecated.  Use sensor_fusion_comm-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InitScale-response>) ostream)
  "Serializes a message object of type '<InitScale-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InitScale-response>) istream)
  "Deserializes a message object of type '<InitScale-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'result) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InitScale-response>)))
  "Returns string type for a service object of type '<InitScale-response>"
  "sensor_fusion_comm/InitScaleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitScale-response)))
  "Returns string type for a service object of type 'InitScale-response"
  "sensor_fusion_comm/InitScaleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InitScale-response>)))
  "Returns md5sum for a message object of type '<InitScale-response>"
  "f8ea41d14f4e256b53793fe5b7588158")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InitScale-response)))
  "Returns md5sum for a message object of type 'InitScale-response"
  "f8ea41d14f4e256b53793fe5b7588158")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InitScale-response>)))
  "Returns full string definition for message of type '<InitScale-response>"
  (cl:format cl:nil "string result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InitScale-response)))
  "Returns full string definition for message of type 'InitScale-response"
  (cl:format cl:nil "string result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InitScale-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InitScale-response>))
  "Converts a ROS message object to a list"
  (cl:list 'InitScale-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'InitScale)))
  'InitScale-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'InitScale)))
  'InitScale-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitScale)))
  "Returns string type for a service object of type '<InitScale>"
  "sensor_fusion_comm/InitScale")