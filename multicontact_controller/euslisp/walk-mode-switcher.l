#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "std_srvs")
(ros::load-ros-manifest "topic_tools")
(ros::load-ros-manifest "multicontact_controller_msgs")

(ros::roseus "walk_mode_switcher" :anonymous nil)

(ros::advertise "control_mode" std_msgs::String 1 t);;latch
(setq *control-mode* "IDLE")
(ros::subscribe "control_mode" std_msgs::String #'(lambda (msg) (setq *control-mode* (send msg :data))))

(if (ros::has-param "base_frame")
    (setq *base-frame* (ros::get-param "base_frame"))
    (setq *base-frame* "BODY"))
(if (ros::has-param "odom_frame")
    (setq *odom-frame* (ros::get-param "odom_frame"))
    (setq *odom-frame* "odom"))
(if (ros::has-param "base_footprint_frame")
    (setq *base-footprint-frame* (ros::get-param "base_footprint_frame"))
    (setq *base-footprint-frame* "base_footprint"))

(if (and (ros::has-param "euslisp_file") (ros::has-param "robot"))
    (progn
      (load (ros::get-param "euslisp_file"))
      (eval (read-from-string (format nil "(~A-init)" (ros::get-param "robot")))))
    (progn
     (load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
     (setq *ri* (instance rtm-ros-robot-interface :define-all-ROSBridge-srv-methods))))

(let ((req (instance topic_tools::MuxAddRequest :init)))
  (send req :topic "odom")
  (ros::service-call "/odom_mux/add" req))

(setq *tfl* (instance ros::transform-listener :init))

(defun enable-walk-mode-cb
    (req)
  (let ((res (send req :response)))
    (send res :success (if (send req :data)
                           (enable-walk-mode)
                           (disable-walk-mode)))
    res))

(defun enable-walk-mode
    ()
  (unless (send *tfl* :wait-for-transform *base-frame* *odom-frame* (ros::time) 5.0)
    (return-from enable-walk-mode nil))
  (unless (or (equal *control-mode* "IDLE")
              (equal *control-mode* "WALK_MODE"))
    (return-from enable-walk-mode nil))
  (ros::publish "control_mode" (instance std_msgs::string :init :data "WALK_MODE"))
  (setq *control-mode* "WALK_MODE")
  (send *ri* :set-base-coords (send *tfl* :lookup-transform *odom-frame* *base-frame* (ros::time)) 1);;ms
  (let ((req (instance topic_tools::MuxSelectRequest :init)))
    (send req :topic "odom")
    (ros::service-call "/odom_mux/select" req))
  (let ((req (instance std_srvs::SetBoolRequest :init)))
    (send req :data t)
    (ros::service-call "/rtabmap/height_limit_rtabmap/enable" req))
  (let ((req (instance std_srvs::SetBoolRequest :init)))
    (send req :data t)
    (ros::service-call "/octomap/height_limit_octomap/enable" req))
  (send *ri* :start-auto-balancer-odom-safe)
  (send *ri* :start-impedance :arms)
  (send *ri* :start-st)
  t)

(defun disable-walk-mode
    ()
  (unless (or (equal *control-mode* "IDLE")
              (equal *control-mode* "WALK_MODE"))
    (return-from enable-walk-mode nil))
  (ros::publish "control_mode" (instance std_msgs::string :init :data "IDLE"))
  (setq *control-mode* "IDLE")
  (send *ri* :stop-st)
  (send *ri* :stop-impedance :arms)
  (send *ri* :stop-auto-balancer-odom-safe)
  (let ((req (instance std_srvs::SetBoolRequest :init)))
    (send req :data nil)
    (ros::service-call "/rtabmap/height_limit_rtabmap/enable" req))
  (let ((req (instance std_srvs::SetBoolRequest :init)))
    (send req :data nil)
    (ros::service-call "/octomap/height_limit_octomap/enable" req))
  (let ((req (instance topic_tools::MuxSelectRequest :init)))
    (send req :topic "odom")
    (ros::service-call "/odom_mux/select" req))
  t)

(ros::advertise-service "enable_walk_mode" std_srvs::SetBool #'enable-walk-mode-cb)

(ros::rate 10)

(while (ros::ok)
  (ros::spin-once)
  (ros::sleep))

