(load "./robotis_op2.l")

(ros::roseus-add-msgs "robotis_controller_msgs")


(defun set-min-max-joint-angle
    ()
  ;; head
  (send (send *robot* :joint "head_pan") :max-angle 120)
  (send (send *robot* :joint "head_pan") :min-angle -120)

  (send (send *robot* :joint "head_tilt") :max-angle 45)
  (send (send *robot* :joint "head_tilt") :min-angle -30)

  ;; arms
  (send (send *robot* :joint "r_sho_pitch") :max-angle 150)
  (send (send *robot* :joint "r_sho_pitch") :min-angle -150)
  (send (send *robot* :joint "l_sho_pitch") :max-angle 150)
  (send (send *robot* :joint "l_sho_pitch") :min-angle -150)

  (send (send *robot* :joint "r_sho_roll") :max-angle 135)
  (send (send *robot* :joint "r_sho_roll") :min-angle -40)
  (send (send *robot* :joint "l_sho_roll") :max-angle 40)
  (send (send *robot* :joint "l_sho_roll") :min-angle -135)

  (send (send *robot* :joint "r_el") :max-angle 70)
  (send (send *robot* :joint "r_el") :min-angle -90)
  (send (send *robot* :joint "l_el") :max-angle 90)
  (send (send *robot* :joint "l_el") :min-angle -70)

  ;; legs
  (send (send *robot* :joint "r_hip_yaw") :max-angle 40)
  (send (send *robot* :joint "r_hip_yaw") :min-angle -60)
  (send (send *robot* :joint "l_hip_yaw") :max-angle 60)
  (send (send *robot* :joint "l_hip_yaw") :min-angle -40)

  (send (send *robot* :joint "r_hip_roll") :max-angle 90)
  (send (send *robot* :joint "r_hip_roll") :min-angle -60)
  (send (send *robot* :joint "l_hip_roll") :max-angle 60)
  (send (send *robot* :joint "l_hip_roll") :min-angle -90)

  (send (send *robot* :joint "r_hip_pitch") :max-angle 90)
  (send (send *robot* :joint "r_hip_pitch") :min-angle -90)
  (send (send *robot* :joint "l_hip_pitch") :max-angle 90)
  (send (send *robot* :joint "l_hip_pitch") :min-angle -90)

  (send (send *robot* :joint "r_knee") :max-angle 85)
  (send (send *robot* :joint "r_knee") :min-angle 0)
  (send (send *robot* :joint "l_knee") :max-angle 0)
  (send (send *robot* :joint "l_knee") :min-angle -85)

  (send (send *robot* :joint "r_ank_pitch") :max-angle 70)
  (send (send *robot* :joint "r_ank_pitch") :min-angle -20)
  (send (send *robot* :joint "l_ank_pitch") :max-angle 20)
  (send (send *robot* :joint "l_ank_pitch") :min-angle -70)

  (send (send *robot* :joint "r_ank_roll") :max-angle 30)
  (send (send *robot* :joint "r_ank_roll") :min-angle -20)
  (send (send *robot* :joint "l_ank_roll") :max-angle 20)
  (send (send *robot* :joint "l_ank_roll") :min-angle -30)
  )

(defun visualize-joint-angle-range
    (&key (joint-list (send *robot* :joint-list)))
  (setq *joint-min-max-list* nil)
  (dolist (jnt joint-list)
    (send *robot* :init-pose)
    (warning-message 3 "=== target joint: ~a ===~%" (send jnt :name))
    (do-until-key
     (send jnt :joint-angle (send jnt :min-angle))
     (while (< (send jnt :joint-angle) (send jnt :max-angle))
       (x::window-main-one)
       (send jnt :joint-angle 1 :relative t)
       (send *irtviewer* :draw-objects)
       (unix::usleep (* 10 1000)))
     )
    )
  )


(defun joint-servo-off
    ()
  (ros::advertise "/robotis/set_control_mode" std_msgs::String 1)
  (ros::advertise "/robotis/sync_write_item" robotis_controller_msgs::SyncWriteItem 1)

  (let* ((string-msg (instance std_msgs::String :init :data "DirectControlMode"))
         (write-msg (instance robotis_controller_msgs::SyncWriteItem :init))
         )
    (ros::publish "/robotis/set_control_mode" string-msg)
    (send write-msg :item_name "torque_enable")
    (send write-msg :joint_name (send-all (send *robot* :joint-list) :name))
    (send write-msg :value (make-list (length (send write-msg :joint_name)) :initial-element 0))
    (ros::publish "/robotis/sync_write_item" write-msg)
    )
  )

(defun measure-joint-angle-range-with-real-robot
    (&key (joint-list (send *robot* :joint-list)))
  (setq *joint-min-max-list* nil)
  (dolist (jnt joint-list)
    (send *robot* :init-pose)
    (warning-message 3 "=== target joint: ~a ===~%" (send jnt :name))
    (warning-message 2 "press Enter to stop animation~%")
    (do-until-key
     (send jnt :joint-angle (send jnt :min-angle))
     (while (< (send jnt :joint-angle) (send jnt :max-angle))
       (x::window-main-one)
       (send jnt :joint-angle 3 :relative t)
       (send *irtviewer* :draw-objects))
     )
    (let* ((min-angle)
           (max-angle)
           )
      (warning-message 2 "move joint to min angle and press Enter~%")
      (read-line)
      (send *robot* :angle-vector (send *ri* :state :potentio-vector))
      (send *irtviewer* :draw-objects)
      (setq min-angle (send jnt :joint-angle))
      (warning-message 2 "move joint to max angle and press Enter~%")
      (read-line)
      (send *robot* :angle-vector (send *ri* :state :potentio-vector))
      (send *irtviewer* :draw-objects)
      (setq max-angle (send jnt :joint-angle))
      (push (list (send jnt :name) min-angle max-angle) *joint-min-max-list*)
      )
    )
  )
