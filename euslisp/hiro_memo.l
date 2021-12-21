(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
hironxjsk-init
objects (list *hironxjsk* (send *hironxjsk* :head-end-coords))
setq new-endcoords (make-cascoords :coords (send (send *hironxjsk* :larm :end-coords :copy-worldcoords) :translate #f(0 0 -300) :local))
send (send *hironxjsk* :larm :end-coords :parent) :assoc new-endcoords


send *hironxjsk* :larm :inverse-kinematics (make-coords :pos #f(590 100 70) :rpy (float-vector (deg2rad -110) (deg2rad -180) (deg2rad 110))) :move-target new-endcoords

send *hironxjsk* :larm :inverse-kinematics (make-coords :pos #f(590 100 -70) :rpy (float-vector (deg2rad -110) (deg2rad -180) (deg2rad 110))) :move-target new-endcoords

send *hironxjsk* :larm :inverse-kinematics (make-coords :pos #f(590 80 160) :rpy (float-vector (deg2rad -100) (deg2rad -180) (deg2rad 100))) :move-target new-endcoords

send *hironxjsk* :rarm :inverse-kinematics (make-coords :pos #f(520 30 110) :rpy (float-vector (deg2rad -60) (deg2rad 0) (deg2rad 110)))

send *ri* :start-grasp :rarm



send *hironxjsk* :rarm :inverse-kinematics (make-coords :pos #f(530 -20 130) :rpy (float-vector (deg2rad -60) (deg2rad 0) (deg2rad 110)))


