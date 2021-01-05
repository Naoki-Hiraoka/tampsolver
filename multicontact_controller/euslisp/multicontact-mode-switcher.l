#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "std_msgs")
(ros::load-ros-manifest "std_srvs")
(ros::load-ros-manifest "topic_tools")
(ros::load-ros-manifest "multicontact_controller_msgs")

(ros::roseus "multicontact_mode_switcher" :anonymous nil)

(ros::advertise "control_mode" std_msgs::String 1 t);;latch
(setq *control-mode* "IDLE")
(ros::subscribe "control_mode" std_msgs::String #'(lambda (msg) (setq *control-mode* (send msg :data))))

(if (and (ros::has-param "euslisp_file") (ros::has-param "robot"))
    (progn
      (load (ros::get-param "euslisp_file"))
      (eval (read-from-string (format nil "(~A-init)" (ros::get-param "robot")))))
    (progn
     (load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
     (setq *ri* (instance rtm-ros-robot-interface :define-all-ROSBridge-srv-methods))))

(if (ros::has-param "base_frame")
    (setq *base-frame* (ros::get-param "base_frame"))
    (setq *base-frame* "BODY"))
(if (ros::has-param "odom_frame")
    (setq *odom-frame* (ros::get-param "odom_frame"))
    (setq *odom-frame* "odom"))

(let ((req (instance topic_tools::MuxAddRequest :init)))
  (send req :topic "multicontact_odom")
  (ros::service-call "/odom_mux/add" req))

(defun end-effectors-cb (msg)
  (dolist (end-effector (set-difference *end-effectors* (send msg :strings) :test #'equal))
    (ros::unsubscribe (concatenate string end-effector "/state")))
  (dolist (end-effector (set-difference (send msg :strings) *end-effectors* :test #'equal))
    (eval `(setq ,(read-from-string (format nil "*~A-state*" end-effector)) ""))
    (ros::subscribe (concatenate string end-effector "/state") std_msgs::String (eval `#'(lambda (msg) (setq ,(read-from-string (format nil "*~A-state*" end-effector)) (send msg :data)))))
    )
  (setq *end-effectors* (send msg :strings)))

(setq *end-effectors* nil)
(ros::subscribe "end_effectors" multicontact_controller_msgs::StringArray #'end-effectors-cb)

(setq *tfl* (instance ros::transform-listener :init))

(defun enable-multicontact-mode-cb
    (req)
  (let ((res (send req :response)))
    (send res :success (if (send req :data)
                           (enable-multicontact-mode)
                           (disable-multicontact-mode)))
    res))

(defun enable-multicontact-mode
    ()
  (unless (and (null (set-difference (list "rleg" "lleg") *end-effectors* :test #'equal))
               (member *rleg-state* (list "CONTACT"
                                          "TOWARD_MAKE_CONTACT") :test #'equal)
               (member *lleg-state* (list "CONTACT"
                                          "TOWARD_MAKE_CONTACT") :test #'equal)
               (every #'(lambda (eef) (member (eval (read-from-string (format nil "*~A-state*" eef)))
                                              (list "NEAR_CONTACT"
                                                    "AIR"
                                                    "NOT_CARED") :test #'equal))
                      (set-difference *end-effectors* (list "rleg" "lleg") :test #'equal)))
    (return-from enable-multicontact-mode nil))
  (unless (send *tfl* :wait-for-transform *base-frame* *odom-frame* (ros::time) 5.0)
    (return-from enable-multicontact-mode nil))
  (unless (or (equal *control-mode* "IDLE")
              (equal *control-mode* "MULTICONTACT_MODE"))
    (return-from enable-walk-mode nil))
  (ros::publish "control_mode" (instance std_msgs::string :init :data "MULTICONTACT_MODE"))
  (setq *control-mode* "MULTICONTACT_MODE")

  (let ((req (instance std_srvs::SetBoolRequest :init)))
      (send req :data t)
      (ros::service-call "/contact_force_estimator/enable" req))
  (let ((req-base (instance multicontact_controller_msgs::SetTransformStampedRequest :init)))
      (send req-base :transform :header :stamp (ros::time-now))
      (send req-base :transform :header :frame_id *odom-frame*)
      (send req-base :transform :child_frame_id *base-frame*)
      (send req-base :transform :transform (ros::coords->tf-transform (send *tfl* :lookup-transform *odom-frame* *base-frame* (ros::time))))
      (ros::service-call "/multicontact_footcoords/set_body_transform" req-base))
  (let ((req (instance std_srvs::SetBoolRequest :init)))
      (send req :data t)
      (ros::service-call "/multicontact_footcoords/enable" req))
  (let ((req (instance topic_tools::MuxSelectRequest :init)))
    (send req :topic "multicontact_odom")
    (ros::service-call "/odom_mux/select" req))
  (let ((req (instance std_srvs::SetBoolRequest :init)))
    (send req :data t)
    (ros::service-call "/end_effector_interactive_marker_server/enable" req))
  (let ((req (instance std_srvs::SetBoolRequest :init)))
    (send req :data t)
    (ros::service-call "/contact_breakability_checker/enable" req))
  (send *ri* :set-interpolation-mode :linear)
  (let ((req (instance std_srvs::SetBoolRequest :init)))
    (send req :data t)
    (ros::service-call "/pwt_controller/enable" req))
  t)

(defun disable-multicontact-mode
    ()
  (unless (and (null (set-difference (list "rleg" "lleg") *end-effectors* :test #'equal))
               (member *rleg-state* (list "CONTACT"
                                          "TOWARD_MAKE_CONTACT") :test #'equal)
               (member *lleg-state* (list "CONTACT"
                                          "TOWARD_MAKE_CONTACT") :test #'equal)
               (every #'(lambda (eef) (member (eval (read-from-string (format nil "*~A-state*" eef)))
                                              (list "NEAR_CONTACT"
                                                    "AIR"
                                                    "NOT_CARED") :test #'equal))
                      (set-difference *end-effectors* (list "rleg" "lleg") :test #'equal)))
    (return-from disable-multicontact-mode nil))
  (unless (send *tfl* :wait-for-transform *base-frame* *odom-frame* (ros::time) 5.0)
    (return-from disable-multicontact-mode nil))
  (unless (or (equal *control-mode* "IDLE")
              (equal *control-mode* "MULTICONTACT_MODE"))
    (return-from enable-walk-mode nil))
  (ros::publish "control_mode" (instance std_msgs::string :init :data "IDLE"))
  (setq *control-mode* "IDLE")
  (let ((req (instance std_srvs::SetBoolRequest :init)))
    (send req :data nil)
    (ros::service-call "/pwt_controller/enable" req))
  (send *ri* :set-interpolation-mode :hoffarbib)
  (let ((req (instance std_srvs::SetBoolRequest :init)))
    (send req :data nil)
    (ros::service-call "/contact_breakability_checker/enable" req))
  (let ((req (instance std_srvs::SetBoolRequest :init)))
    (send req :data nil)
    (ros::service-call "/end_effector_interactive_marker_server/enable" req))
  (send *ri* :set-base-coords (send *tfl* :lookup-transform *odom-frame* *base-frame* (ros::time)) 1);;ms
  (let ((req (instance topic_tools::MuxSelectRequest :init)))
    (send req :topic "odom")
    (ros::service-call "/odom_mux/select" req))
  (let ((req (instance std_srvs::SetBoolRequest :init)))
    (send req :data nil)
    (ros::service-call "/multicontact_footcoords/enable" req))
  (let ((req (instance std_srvs::SetBoolRequest :init)))
    (send req :data nil)
    (ros::service-call "/contact_force_estimator/enable" req))
  t)

(ros::advertise-service "enable_multicontact_mode" std_srvs::SetBool #'enable-multicontact-mode-cb)

(ros::rate 10)

(while (ros::ok)
  (ros::spin-once)
  (ros::sleep))

