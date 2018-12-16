(ros::roseus "robotis_op2")

(require "package://jsk_darwin/euslisp/robotis_op2.l")
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
     (cons :joint-names (list "l_hip_yaw" "l_hip_roll" "l_hip_pitch" "l_knee" "l_ank_pitch" "l_ank_roll" "r_hip_yaw" "r_hip_roll" "r_hip_pitch" "r_knee" "r_ank_pitch" "r_ank_roll" "r_sho_pitch" "r_sho_roll" "r_el" "l_sho_pitch" "l_sho_roll" "l_el" "head_pan" "head_tilt")))))
  )

(defun robotis_op2-init (&optional (create-viewer))
  (unless (boundp '*robotis_op2*) (robotis_op2))
  (unless (ros::ok) (ros::roseus "robotis_op2_eus_interface"))
  (unless (boundp '*ri*) (setq *ri* (instance robotis_op2-interface :init)))

  (ros::spin-once)
  (send *ri* :spin-once)

  (send *robotis_op2* :angle-vector (send *ri* :state :potentio-vector))
  (when create-viewer (objects (list *robotis_op2*)))
  )
