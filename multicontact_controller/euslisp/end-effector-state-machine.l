#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "std_msgs")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "multicontact_controller_msgs")

(load "package://roseus_smach/src/state-machine.l")
(load "package://roseus_smach/src/state-machine-ros.l")

(ros::roseus "end_effector_state_machine" :anonymous nil)

(defclass end-effector-state-machine
  :super state-machine
  :slots (eef-name
          state-topic state-msg
          force-topic act-force act-torque
          info-topic info
          set-ref-contact-serivice current-ref-contact cmd-ref-contact
          set-near-contact-serivice current-near-contact cmd-near-contact
          set-cared-serivice current-cared cmd-cared
          ))

(defmethod end-effector-state-machine
    (:init
     (&key ((:eef-name en)))
        (send-super* :init nil)
        (setq eef-name en)
        (send self :construct-structure)
        (send self :setup)
        self)
  (:construct-structure
   ()
   (send self :add-node (instance state :init :NOT_CARED `(lambda-closure nil 0 0 (&rest args) (send ,self :NOT_CARED-exec))))
   (send self :add-node (instance state :init :AIR `(lambda-closure nil 0 0 (&rest args) (send ,self :AIR-exec))))
   (send self :add-node (instance state :init :NEAR_CONTACT `(lambda-closure nil 0 0 (&rest args) (send ,self :NEAR_CONTACT-exec))))
   (send self :add-node (instance state :init :TOWARD_MAKE_CONTACT `(lambda-closure nil 0 0 (&rest args) (send ,self :TOWARD_MAKE_CONTACT-exec))))
   (send self :add-node (instance state :init :TOWARD_BREAK_CONTACT `(lambda-closure nil 0 0 (&rest args) (send ,self :TOWARD_BREAK_CONTACT-exec))))
   (send self :add-node (instance state :init :CONTACT `(lambda-closure nil 0 0 (&rest args) (send ,self :CONTACT-exec))))

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
   )
  (:setup
   ()
   (setq state-topic (concatenate string eef-name "/state"))
   (setq state-msg (instance std_msgs::String :init))
   (ros::advertise state-topic std_msgs::String 1)

   (setq force-topic (concatenate string eef-name "/force"))
   (setq act-force (float-vector 0 0 0))
   (setq act-torque (float-vector 0 0 0))
   (ros::subscribe force-topic geometry_msgs::WrenchStamped #'send self :force-cb)

   (setq info-topic (concatenate string eef-name "/info"))
   (setq info (instance multicontact_controller_msgs::EndEffectorInfo :init))
   (ros::subscribe info-topic multicontact_controller_msgs::EndEffectorInfo #'send self :info-cb)

   (ros::wait-for-service "end_effector_info_publisher/add_end_effector")
   (let ((req (instance multicontact_controller_msgs::SetStringRequest :init)))
     (send req :name eef-name)
     (ros::service-call "end_effector_info_publisher/add_end_effector" req))

   (setq set-ref-contact-serivice (concatenate string eef-name "/set_ref_contact"))
   (ros::advertise-service set-ref-contact-serivice std_srvs::SetBool #'send self :set-ref-contact-cb)

   (setq set-near-contact-serivice (concatenate string eef-name "/set_near_contact"))
   (ros::advertise-service set-near-contact-serivice std_srvs::SetBool #'send self :set-near-contact-cb)

   (setq set-cared-serivice (concatenate string eef-name "/set_cared"))
   (ros::advertise-service set-cared-serivice std_srvs::SetBool #'send self :set-cared-cb)

   t)
  (:cleanup
   ()
   (ros::unadvertise state-topic)
   (ros::unsubscribe force-topic)
   (ros::unsubscribe info-topic)
   (ros::wait-for-service "end_effector_info_publisher/remove_end_effector")
   (let ((req (instance multicontact_controller_msgs::SetStringRequest :init)))
     (send req :name eef-name)
     (ros::service-call "end_effector_info_publisher/remove_end_effector" req))
   (ros::unadvertise-service set-ref-contact-serivice)
   (ros::unadvertise-service set-near-contact-serivice)
   (ros::unadvertise-service set-cared-serivice)
   t)
  (:force-cb
   (msg)
   (setelt act-force 0 (send (send (send msg :wrench) :force) :x))
   (setelt act-force 1 (send (send (send msg :wrench) :force) :y))
   (setelt act-force 2 (send (send (send msg :wrench) :force) :z))
   (setelt act-torque 0 (send (send (send msg :wrench) :torque) :x))
   (setelt act-torque 1 (send (send (send msg :wrench) :torque) :y))
   (setelt act-torque 2 (send (send (send msg :wrench) :torque) :z))
   )
  (:info-cb
   (msg)
   (setq info msg)
   )
  (:set-ref-contact-cb
   (req)
   (let ((res (send req :response))
         (org-cmd-ref-contact cmd-ref-contact))
     (setq cmd-ref-contact (send req :data))
     (send self :execute nil :step 0)
     (send self :execute nil :step 0)
     (if (equal (send req :data) current-ref-contact)
         (send res :success t)
         (progn (send res :success nil)
                (setq cmd-ref-contact org-cmd-ref-contact)))
     res))
  (:set-near-contact-cb
   (req)
   (let ((res (send req :response))
         (org-cmd-near-contact cmd-near-contact))
     (setq cmd-near-contact (send req :data))
     (send self :execute nil :step 0)
     (send self :execute nil :step 0)
     (if (equal (send req :data) current-near-contact)
         (send res :success t)
         (progn (send res :success nil)
                (setq cmd-near-contact org-cmd-near-contact)))
     res))
  (:set-cared-cb
   (req)
   (let ((res (send req :response))
         (org-cmd-cared cmd-cared))
     (setq cmd-cared (send req :data))
     (send self :execute nil :step 0)
     (send self :execute nil :step 0)
     (if (equal (send req :data) current-cared)
         (send res :success t)
         (progn (send res :success nil)
                (setq cmd-cared org-cmd-cared)))
     res))
  (:publish-state
   ()
   (send state-msg :data (string (send (send self :active-state) :name)))
   (ros::publish state-topic state-msg)
   t)
  (:act-contact
   (high)
   (cond
     ((equal "SURFACE" (send info :contact :type))
      (if high
          (< (send info :contact :contact_decision_threshold2) (elt act-force 2))
          (< (send info :contact :contact_decision_threshold1) (elt act-force 2))))
     (t nil)))
  (:NOT_CARED-exec
   (&rest args)
   (setq current-cared nil)
   (setq current-near-contact nil)
   (setq current-ref-contact nil)
   (send self :publish-state)
   (cond (cmd-cared :cared)
         (t :stay))
   )
  (:AIR-exec
   (&rest args)
   (setq current-cared t)
   (setq current-near-contact nil)
   (setq current-ref-contact nil)
   (send self :publish-state)
   (cond ((null cmd-cared) :not-cared)
         (cmd-near-contact :near-contact)
         (t :stay))
   )
  (:NEAR_CONTACT-exec
   (&rest args)
   (setq current-cared t)
   (setq current-near-contact t)
   (setq current-ref-contact nil)
   (send self :publish-state)
   (cond ((null cmd-near-contact) :far-contact)
         (cmd-ref-contact :ref-contact)
         (t :stay))
   )
  (:TOWARD_MAKE_CONTACT-exec
   (&rest args)
   (setq current-cared t)
   (setq current-near-contact t)
   (setq current-ref-contact t)
   (send self :publish-state)
   (cond
     ((send self :act-contact t) :act-contact)
     ((null cmd-ref-contact) :ref-not-contact)
     (t :stay))
   )
  (:CONTACT-exec
   (&rest args)
   (setq current-cared t)
   (setq current-near-contact t)
   (setq current-ref-contact t)
   (send self :publish-state)
   (cond
     ((null cmd-ref-contact) :ref-not-contact)
     ((null (send self :act-contact nil)) :act-not-contact)
     (t :stay))
   )
  (:TOWARD_BREAK_CONTACT-exec
   (&rest args)
   (setq current-cared t)
   (setq current-near-contact t)
   (setq current-ref-contact nil)
   (send self :publish-state)
   (cond
     ((null (send self :act-contact nil)) :act-not-contact)
     (cmd-ref-contact :ref-contact)
     (t :stay))
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
        (if (equal (send (send (send (send *sm* :node (send req :name)) :submachine) :active-state) :name) :NOT_CARED)
            (progn
              (send (send (send *sm* :node (send req :name)) :submachine) :cleanup)
              (send *sm* :remove-node (send *sm* :node (send req :name)))
              (send *sm* :start-state (send-all (send *sm* :nodes) :name))
              (send *sm* :active-state (send *sm* :nodes))
              (send res :success t)
              (setq *structure-changed* t))
            (progn (send res :success nil)
                   (send res :message (format nil "~s is not NOT_CARED (~s)" (send req :name) (send (send (send *sm* :node (send req :name)) :active-state) :name)))))
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

(dolist (eef-name (ros::get-param "end_effectors"))
  (add-end-effector-cb (instance multicontact_controller_msgs::setstringrequest :init :name eef-name)))

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
