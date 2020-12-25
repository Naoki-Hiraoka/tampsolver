(load "package://pr2eus/speak.l")
(ros::load-ros-package "multicontact_controller_msgs")

(ros::roseus "motor_temperature_speaker")

(setq joints nil)
(ros::subscribe
 "motor_temperature_states"
 multicontact_controller_msgs::MotorTemperatureState
 #'(lambda (m)
     (unless (= (length (send m :name)) (length (send m :remaining_time)))
       (return))
     (dotimes (i (length (send m :name)))
       (if (assoc (elt (send m :name) i) joints :test #'equal)
           (setf (cadr (assoc (elt (send m :name) i) joints :test #'equal)) (elt (send m :remaining_time) i))
           (push (list (elt (send m :name) i) (elt (send m :remaining_time) i)) joints))))
 )

(setq speak-rate (ros::get-param "~speak_rate"))
(unless speak-rate (setq speak-rate 0.1))
(setq speak-limit-time (ros::get-param "~speak_limit_time"))
(unless speak-limit-time (setq speak-limit-time 600.0))

(setq rate (ros::get-param "~rate"))
(unless rate (setq rate 1))
(ros::rate rate)

(setq prev-speak-time (ros::time-now))
(while (ros::ok)
  (ros::spin-once)

  (let ((time (ros::time-now)))
    (when (> (send (ros::time- time prev-speak-time) :to-sec) (/ 1.0 speak-rate))

      (when (> (length joints) 0)
        (let ((minimum (car joints)))
          (dolist (j joints)
            (when (< (cadr j) (cadr minimum))
              (setq minimum j)))

          (when (< (cadr minimum) speak-limit-time)
            (let ((m (format nil "~A ~A second"
                             (map string #'(lambda (c) (cond ((= c (elt "_" 0)) (elt " " 0))
                                                             (t c)))
                                  (car minimum))
                             (floor (cadr minimum)))))
              (ros::ros-info m)
              (speak m)
              (setq prev-speak-time time)
            ))))))
  (ros::sleep)
  )
