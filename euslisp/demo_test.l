(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(load "package://dynamixel_hand/euslisp/dxl-hand-interface.l")

(hironxjsk-init)
(setq new-endcoords (make-cascoords :coords (send (send *hironxjsk* :larm :end-coords :copy-worldcoords) :translate #f(0 0 -300) :local)))
(send (send *hironxjsk* :larm :end-coords :parent) :assoc new-endcoords)


(hand-model5)
(setq *hand* (instance rhp3hand_l-interface :init))

(send *hironxjsk* :larm :inverse-kinematics (make-coords :pos #f(500 0 300) :rpy (float-vector (deg2rad -90) 0 (deg2rad 90))))
(doi-until-key (unix::sleep 1))

(send *hironxjsk* :larm :inverse-kinematics (make-coords :pos #f(590 100 40) :rpy (float-vector (deg2rad -110) (deg2rad -180) (deg2rad 110))) :move-target new-endcoords)
