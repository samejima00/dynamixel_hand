(defun callback(msg)
  (setq *result-data* (send msg :data))
  )

(ros::roseus "sensor")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(load "package://dynamixel_hand/euslisp/dxl-hand-interface.l")
(ros::subscribe "/touch_result" std_msgs::Float32MultiArray #'callback)

(setq *result-data* (float-vector 0 0 0 0))

(hironxjsk-init)
(setq new-endcoords (make-cascoords :coords (send (send *hironxjsk* :larm :end-coords :copy-worldcoords) :translate #f(0 0 -300) :local)))
(send (send *hironxjsk* :larm :end-coords :parent) :assoc new-endcoords)

(send *hironxjsk* :larm :inverse-kinematics 
      (make-coords :pos #f(500 0 300) :rpy (float-vector (deg2rad -90) 0 (deg2rad 90))))
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)
(do-until-key (unix::sleep 1))

(hand_model5)
(setq *hand* (instance rhp3hand_l-interface :init))
(send *hand* :angle-vector (send *hand_model5* :angle-vector #f(120 10 40 0 -40 -20 180)) :fast :default-controller 0 :min-time 0.05)

(setq flag 0)
(ros::spin-once)
(ros::sleep)
(while (= flag 0)
    (send *hironxjsk* :larm :inverse-kinematics 
	  (make-coords :pos #f(590 100 0) :rpy (float-vector (deg2rad -110) (deg2rad -180) (deg2rad 110))) :move-target new-endcoords)
    (send *ri* :angle-vector (send *hironxjsk* :angle-vector))
    (send *ri* :wait-interpolation)
    (do-until-key (unix::sleep 1))

    (send *hironxjsk* :larm :inverse-kinematics
	  (make-coords :pos #f(590 100 -65) :rpy (float-vector (deg2rad -110) (deg2rad -180) (deg2rad 110))) :move-target new-endcoords)
    (send *ri* :angle-vector (send *hironxjsk* :angle-vector))
    (send *ri* :wait-interpolation)
    (send *hand* :angle-vector (send *hand_model5* :angle-vector #f(120 -30 120 0 -130 -50 290)) :fast :default-controller 0 :min-time 0.05)
  (unix::sleep 2)  
  (ros::spin-once)
  (ros::sleep)
   (if (< (elt *result-data* 0) 80)
        (progn (setq flag 1) (print *result-data*))
    (send *hand* :angle-vector (send *hand_model5* :angle-vector #f(120 10 40 0 -40 -20 180)) :fast :default-controller 0 :min-time 0.05)
  )
)

(send *hironxjsk* :larm :inverse-kinematics
      (make-coords :pos #f(590 100 70) :rpy (float-vector (deg2rad -110) (deg2rad -180) (deg2rad 110))) :move-target new-endcoords)
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)

(send *hironxjsk* :larm :inverse-kinematics (make-coords :pos #f(590 80 160) :rpy (float-vector (deg2rad -100) (deg2rad -180) (deg2rad 100))) :move-target new-endcoords)
(send *hironxjsk* :rarm :inverse-kinematics (make-coords :pos #f(520 30 110) :rpy (float-vector (deg2rad -60) (deg2rad 0) (deg2rad 110))))
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)
(send *hand* :angle-vector (send *hand_model5* :angle-vector #f(120 10 40 0 -40 -20 180)) :fast :default-controller 0 :min-time 0.05)

(setq flag 0)
(while (= flag 0)
  (send *hironxjsk* :rarm :move-end-pos (float-vector -5 10 0))
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector))
  (send *ri* :wait-interpolation)
  (ros::spin-once)
  (unix::sleep 1)
  (when (or (< (elt *result-data* 1) 70) (< (elt *result-data* 3) 70))
  (return nil)
  )
 
  (send *hironxjsk* :rarm :move-end-pos (float-vector 0 0 5))
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector))
  (send *ri* :wait-interpolation)
  (ros::spin-once)
  (unix::sleep 1)
  (when (or (< (elt *result-data* 1) 70) (< (elt *result-data* 3) 70))
  (return nil)
  )

  (send *hironxjsk* :rarm :move-end-pos (float-vector 0 0 -10))
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector))
  (send *ri* :wait-interpolation)
  (ros::spin-once)
  (unix::sleep 1)
  (when (or (< (elt *result-data* 1) 70) (< (elt *result-data* 3) 70))
  (return nil)
  )

  (send *hironxjsk* :rarm :move-end-pos (float-vector 0 0 5))
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector))
  (send *ri* :wait-interpolation)
  
)



;;引き出し開け閉め

(send *hand* :angle-vector (send *hand_model5* :angle-vector #f(40 10 40 0 -40 -20 290)) :fast :default-controller 0 :min-time 0.05)

(send *hironxjsk* :larm :inverse-kinematics 
      (make-coords :pos #f(300 0 250) :rpy (float-vector (deg2rad -90) (deg2rad 0) (deg2rad 40))))
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)



(send *hironxjsk* :larm :inverse-kinematics 
      (make-coords :pos #f(500 0 115) :rpy (float-vector (deg2rad -90) (deg2rad 0) (deg2rad 40))))
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)

(send *hironxjsk* :larm :inverse-kinematics
      (make-coords :pos #f(520 0 115) :rpy (float-vector (deg2rad -90) (deg2rad 0) (deg2rad 40))))
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)

(send *hironxjsk* :larm :inverse-kinematics
      (make-coords :pos #f(520 0 100) :rpy (float-vector (deg2rad -90) (deg2rad 0) (deg2rad 45))))
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)

(send *hand* :angle-vector (send *hand_model5* :angle-vector #f(40 -20 90 0 -100 -10 290)) :fast :default-controller 0 :min-time 0.05)

(send *hironxjsk* :larm :inverse-kinematics
      (make-coords :pos #f(450 0 100) :rpy (float-vector (deg2rad -90) (deg2rad 0) (deg2rad 45))))
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)

