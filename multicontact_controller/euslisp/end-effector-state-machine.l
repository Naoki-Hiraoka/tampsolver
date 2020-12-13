#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "std_msgs")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "multicontact_controller_msgs")

(load "package://roseus_smach/src/state-machine.l")
(load "package://roseus_smach/src/state-machine-ros.l")

(ros::roseus "end_effector_state_machine")

(defclass end-effector-state-machine
  :super state-machine
  :slots (eef-name))

(defmethod end-effector-state-machine
  (:init
   (&key ((:eef-name en)))
      (send-super* :init nil)
      (setq eef-name en)
      (send self :add-node (instance state :init :NOT_CARED `(lambda-closure nil 0 0 (&rest args) (send ,self :NOT_CARED-exec))))
      (send self :add-node (instance state :init :AIR `(lambda-closure nil 0 0 () (send ,self :AIR-exec))))
      (send self :add-node (instance state :init :NEAR_CONTACT '(lambda-closure nil 0 0  () (send self :NEAR_CONTACT-exec))))
      (send self :add-node (instance state :init :TOWARD_MAKE_CONTACT `(lambda-closure nil 0 0  () (send self :TOWARD_MAKE_CONTACT-exec))))
      (send self :add-node (instance state :init :TOWARD_BREAK_CONTACT `(lambda-closure nil 0 0  () (send self :TOWARD_BREAK_CONTACT-exec))))
      (send self :add-node (instance state :init :CONTACT `(lambda-closure nil 0 0  () (send self :CONTACT-exec))))

      (send self :start-state :NOT_CARED)

      (send self :add-transition :NOT_CARED :NOT_CARED :stay)
      (send self :add-transition :NOT_CARED :AIR :cared)
      (send self :add-transition :AIR :NOT_CARED :not-cared)
      (send self :add-transition :AIR :AIR :stay)
      (send self :add-transition :AIR :NEAR_CONTACT :near-contact)
      (send self :add-transition :NEAR_CONTACT :AIR :far-contact)
      (send self :add-transition :NEAR_CONTACT :NEAR_CONTACT :stay)
      (send self :add-transition :NEAR_CONTACT :TOWARD_MAKE_CONTACT :ref-contact)
      (send self :add-transition :TOWARD_MAKE_CONTACT :NEAR_CONTACT :ref-not-contact)
      (send self :add-transition :TOWARD_MAKE_CONTACT :TOWARD_MAKE_CONTACT :stay)
      (send self :add-transition :TOWARD_MAKE_CONTACT :CONTACT :act-contact)
      (send self :add-transition :CONTACT :TOWARD_MAKE_CONTACT :act-not-contact)
      (send self :add-transition :CONTACT :CONTACT :stay)
      (send self :add-transition :CONTACT :TOWARD_BREAK_CONTACT :ref-not-contact)
      (send self :add-transition :TOWARD_BREAK_CONTACT :CONTACT :ref-contact)
      (send self :add-transition :TOWARD_BREAK_CONTACT :TOWARD_BREAK_CONTACT :stay)
      (send self :add-transition :TOWARD_BREAK_CONTACT :NEAR_CONTACT :act-not-contact)

      (send self :reset-state)
      self)
  (:cleanup
   ()
   nil
   )
  (:NOT_CARED-exec
   (&rest args)
   (unix::sleep 2)
   :stay
   )
  (:AIR-exec
   (&rest args)
   :stay
   )
  (:NEAR_CONTACT-exec
   (&rest args)
   :stay
   )
  (:TOWARD_MAKE_CONTACT-exec
   (&rest args)
   :stay
   )
  (:CONTACT-exec
   (&rest args)
   :stay
   )
  (:TOWARD_BREAK_CONTACT-exec
   (&rest args)
   :stay
   )
  )


(defun add-end-effector-cb (req)
  (let ((res (send req :response)))
    (if (send *sm* :node (send req :name))
        (progn (send res :success t)
               (send res :message (format nil "~s is already exists" (send req :name))))
        (progn (send *sm* :add-node (instance state :init (send req :name) (instance end-effector-state-machine :init :eef-name (send req :name))))
               (send *sm* :start-state (send-all (send *sm* :nodes) :name))
               (send *sm* :active-state (send *sm* :nodes))
               (send res :success t)
               (setq *structure-changed* t)))
  res))

(defun remove-end-effector-cb (req)
  (let ((res (send req :response)))
    (if (send *sm* :node (send req :name))
        (progn
          (send (send *sm* :node (send req :name)) :cleanup)
          (send *sm* :remove-node (send req :name))
          (send *sm* :start-state (send-all (send *sm* :nodes) :name))
          (send *sm* :active-state (send *sm* :nodes))
          (send res :success t)
          (setq *structure-changed* t))
        (progn (send res :success t)
               (send res :message (format nil "~s is already removed" (send req :name)))))
    res))

(defun enable-cb (req)
  (let ((res (send req :response)))
    (setq *enabled* (send req :data))
    (send res :success t)
    res))



(setq *sm* (instance state-machine :init :parallel t))
(setq *insp* (instance state-machine-inspector :init *sm* :root-name "END_EFFECTOR_STATE_MACHINE"))
(setq *structure-changed* t)
(setq *enabled* t)

(unix::sleep 2)

(ros::advertise-service "~add_end_effector" multicontact_controller_msgs::SetString #'add-end-effector-cb)
(ros::advertise-service "~remove_end_effector" multicontact_controller_msgs::SetString #'remove-end-effector-cb)

(ros::advertise-service "~enable" std_srvs::SetBool #'enable-cb)

(ros::rate 100)
(while (ros::ok)
  (ros::spin-once)
  (when *structure-changed*
    (send *insp* :publish-structure) ;; latch
    (setq *structure-changed* nil))
  (send *insp* :publish-status)
  (send *sm* :execute nil :step -1)
  (ros::sleep))
