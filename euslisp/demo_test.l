(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(load "package://dynamixel_hand/euslisp/dxl-hand-interface.l")
(ros::subscribe "/touch_data" std_msgs::Float32MultiArray #'callback)

(defun callback(msg)
  (setq *result-data* (send msg :data))
  )

(hironxjsk-init)
(setq new-endcoords (make-cascoords :coords (send (send *hironxjsk* :larm :end-coords :copy-worldcoords) :translate #f(0 0 -300) :local)))
(send (send *hironxjsk* :larm :end-coords :parent) :assoc new-endcoords)

(hand-model5)
(setq *hand* (instance rhp3hand_l-interface :init))
(send *hand* :angle-vector (send *hand_model5* :angle-vector #f(120 10 40 0 -40 -20 180)) :fast :default-controller 0 :min-time 0.05)

(send *hironxjsk* :larm :inverse-kinematics 
      (make-coords :pos #f(500 0 300) :rpy (float-vector (deg2rad -90) 0 (deg2rad 90))))
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)
(do-until-key (unix::sleep 1))

(send *hironxjsk* :larm :inverse-kinematics 
      (make-coords :pos #f(590 100 40) :rpy (float-vector (deg2rad -110) (deg2rad -180) (deg2rad 110))) :move-target new-endcoords)
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)
(do-until-key (unix::sleep 1))

(while ((and (> (elt *result-data* 0) 80) (> (elt *result-data* 1) 80)))
       (send *hironxjsk* new-endcoords :move-end-pos (float-vector 0 0 -10))
       (send *ri* :angle-vector (send *hironxjsk* :angle-vector))
       (send *ri* :wait-interpolation)
       )
 

