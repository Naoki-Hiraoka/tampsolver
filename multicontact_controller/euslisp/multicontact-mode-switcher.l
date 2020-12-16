#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "std_msgs")
(ros::load-ros-manifest "std_srvs")
(ros::load-ros-manifest "multicontact_controller_msgs")

(ros::roseus "multicontact_mode_switcher" :anonymous nil)

(if (ros::has-param "base_frame")
    (setq *base-frame* (ros::get-param "base_frame"))
    (setq *base-frame* "BODY"))
(if (ros::has-param "odom_frame")
    (setq *odom-frame* (ros::get-param "odom_frame"))
    (setq *odom-frame* "odom"))

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
  (let ((req (instance std_srvs::SetBoolRequest :init)))
      (send req :data t)
      (ros::service-call "/contact_force_estimator/enable" req))
  (let ((tm (ros::time-now)))
    (unless (send *tfl* :wait-for-transform *base-frame* *odom-frame* tm 5.0)
      (return-from enable-multicontact-mode nil))
    (let ((req-base (instance multicontact_controller_msgs::SetTransformStampedRequest :init)))
      (send req-base :transform :header :stamp (ros::time-now))
      (send req-base :transform :header :frame_id *odom-frame*)
      (send req-base :transform :child_frame_id *base-frame*)
      (send req-base :transform :transform (ros::coords->tf-transform (send *tfl* :lookup-transform *odom-frame* *base-frame* tm)))
      (ros::service-call "/multicontact_footcoords/set_body_transform" req-base))
    (let ((req (instance std_srvs::SetBoolRequest :init)))
      (send req :data nil)
      (ros::service-call "/idle_footcoords/enable" req))
    (let ((req (instance std_srvs::SetBoolRequest :init)))
      (send req :data t)
      (ros::service-call "/multicontact_footcoords/enable" req))
    ;; (send *ri* :start-st)
    t))

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
  ;; (send *ri* :stop-st)
  (let ((tm (ros::time-now)))
    (unless (send *tfl* :wait-for-transform *base-frame* *odom-frame* tm 5.0)
      (return-from disable-multicontact-mode nil))
    (let ((req-base (instance multicontact_controller_msgs::SetTransformStampedRequest :init)))
      (send req-base :transform :header :stamp (ros::time-now))
      (send req-base :transform :header :frame_id *odom-frame*)
      (send req-base :transform :child_frame_id *base-frame*)
      (send req-base :transform :transform (ros::coords->tf-transform (send *tfl* :lookup-transform *odom-frame* *base-frame* tm)))
      (ros::service-call "/idle_footcoords/set_body_transform" req-base)))
  (let ((req (instance std_srvs::SetBoolRequest :init)))
      (send req :data nil)
      (ros::service-call "/multicontact_footcoords/enable" req))
  (let ((req (instance std_srvs::SetBoolRequest :init)))
    (send req :data t)
    (ros::service-call "/idle_footcoords/enable" req))
  (let ((req (instance std_srvs::SetBoolRequest :init)))
    (send req :data nil)
    (ros::service-call "/contact_force_estimator/enable" req))
  t)

(ros::advertise-service "enable_multicontact_mode" std_srvs::SetBool #'enable-multicontact-mode-cb)

(ros::rate 10)

(while (ros::ok)
  (ros::spin-once)
  (ros::sleep))

