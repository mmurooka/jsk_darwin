(require "package://jsk_darwin/euslisp/robotis_op2.l")
(require "package://jsk_darwin/euslisp/robotis_op2-utils.l")
(require "package://pr2eus/robot-interface.l")


(defclass robotis_op2-interface
  :super robot-interface
  :slots ()
  )

(defmethod robotis_op2-interface
  (:init (&rest args)
   (prog1 (send-super* :init :robot robotis_op2-robot :base-controller-action-name nil args)
     (send self :add-controller :fullbody-controller)
     ))
  (:default-controller ()
   (append
    (send self :fullbody-controller)))
  (:fullbody-controller ()
   (list
    (list
     (cons :controller-action "/fullbody_controller/follow_joint_trajectory_action")
     (cons :controller-state "/fullbody_controller/follow_joint_trajectory_action/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (send-all (send robot :joint-list) :name)))))
  )

(defun robotis_op2-init (&optional (create-viewer))
  (unless (boundp '*robotis_op2*) (robotis_op2))
  (unless (ros::ok) (ros::roseus "robotis_op2_eus_interface"))
  (unless (boundp '*ri*) (setq *ri* (instance robotis_op2-interface :init)))

  (ros::spin-once)
  (send *ri* :spin-once)

  (when create-viewer (objects (list *robotis_op2*)))
  )
