; Auto-generated. Do not edit!


(cl:in-package sensor_fusion_comm-srv)


;//! \htmlinclude InitHeight-request.msg.html

(cl:defclass <InitHeight-request> (roslisp-msg-protocol:ros-message)
  ((height
    :reader height
    :initarg :height
    :type cl:float
    :initform 0.0))
)

(cl:defclass InitHeight-request (<InitHeight-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InitHeight-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InitHeight-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_fusion_comm-srv:<InitHeight-request> is deprecated: use sensor_fusion_comm-srv:InitHeight-request instead.")))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <InitHeight-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_fusion_comm-srv:height-val is deprecated.  Use sensor_fusion_comm-srv:height instead.")
  (height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InitHeight-request>) ostream)
  "Serializes a message object of type '<InitHeight-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InitHeight-request>) istream)
  "Deserializes a message object of type '<InitHeight-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InitHeight-request>)))
  "Returns string type for a service object of type '<InitHeight-request>"
  "sensor_fusion_comm/InitHeightRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitHeight-request)))
  "Returns string type for a service object of type 'InitHeight-request"
  "sensor_fusion_comm/InitHeightRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InitHeight-request>)))
  "Returns md5sum for a message object of type '<InitHeight-request>"
  "6b19ce060384743d60708a804f1c749f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InitHeight-request)))
  "Returns md5sum for a message object of type 'InitHeight-request"
  "6b19ce060384743d60708a804f1c749f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InitHeight-request>)))
  "Returns full string definition for message of type '<InitHeight-request>"
  (cl:format cl:nil "float32 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InitHeight-request)))
  "Returns full string definition for message of type 'InitHeight-request"
  (cl:format cl:nil "float32 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InitHeight-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InitHeight-request>))
  "Converts a ROS message object to a list"
  (cl:list 'InitHeight-request
    (cl:cons ':height (height msg))
))
;//! \htmlinclude InitHeight-response.msg.html

(cl:defclass <InitHeight-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform ""))
)

(cl:defclass InitHeight-response (<InitHeight-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InitHeight-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InitHeight-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_fusion_comm-srv:<InitHeight-response> is deprecated: use sensor_fusion_comm-srv:InitHeight-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <InitHeight-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_fusion_comm-srv:result-val is deprecated.  Use sensor_fusion_comm-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InitHeight-response>) ostream)
  "Serializes a message object of type '<InitHeight-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InitHeight-response>) istream)
  "Deserializes a message object of type '<InitHeight-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InitHeight-response>)))
  "Returns string type for a service object of type '<InitHeight-response>"
  "sensor_fusion_comm/InitHeightResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitHeight-response)))
  "Returns string type for a service object of type 'InitHeight-response"
  "sensor_fusion_comm/InitHeightResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InitHeight-response>)))
  "Returns md5sum for a message object of type '<InitHeight-response>"
  "6b19ce060384743d60708a804f1c749f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InitHeight-response)))
  "Returns md5sum for a message object of type 'InitHeight-response"
  "6b19ce060384743d60708a804f1c749f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InitHeight-response>)))
  "Returns full string definition for message of type '<InitHeight-response>"
  (cl:format cl:nil "string result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InitHeight-response)))
  "Returns full string definition for message of type 'InitHeight-response"
  (cl:format cl:nil "string result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InitHeight-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InitHeight-response>))
  "Converts a ROS message object to a list"
  (cl:list 'InitHeight-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'InitHeight)))
  'InitHeight-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'InitHeight)))
  'InitHeight-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitHeight)))
  "Returns string type for a service object of type '<InitHeight>"
  "sensor_fusion_comm/InitHeight")