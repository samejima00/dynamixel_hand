(defun callback(msg)
  (setq *result-data* (send msg :data))
  )

;;(defun box-callback(msg)
;;    (unless (send msg :boxes) (return-from box-callback nil))
;;    (setq bb (elt (send msg :boxes) 0))
;;    (setq *co1* (send *hironxjsk* :copy-coords))
;;    (let ((cube (bounding-box->cube bb))
;;	(send (send (send *hironxjsk* :head-end-coords) :copy-coords) :transform *co1*))
;;    (send cube :newcoords (send cam-coords :transform cube))
;;    (setq *cube-coords* cube)))
  
(defun tf-pose->coords (frame_id pose)
  (send (send *tfl* :lookup-transform "WAIST" frame_id (ros::time 0))
        :transform (ros::tf-pose->coords pose)))

(defun bbox->cube (bbox)
  (let* ((dims (ros::tf-point->pos (send bbox :dimensions)))
         (bx (make-cube (elt dims 0) (elt dims 1) (elt dims 2))))
    (send bx :newcoords
          (tf-pose->coords (send bbox :header :frame_id) (send bbox :pose)))
    bx))

(defun box-callback(msg)
  (unless (send msg :boxes) (return-from box-callback nil))
  (setq *target-bbox* (elt (send msg :boxes) 0))
  (setq *target-cube* (bbox->cube *target-bbox*))
  )

(ros::roseus "sensor")
(ros::load-ros-manifest "jsk_recognition_msgs")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(load "package://dynamixel_hand/euslisp/dxl-hand-interface.l")
;;(load "package://jsk_pcl_ros/euslisp/bounding-box-to-cube.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(ros::subscribe "/touch_result" std_msgs::Float32MultiArray #'callback)
(ros::subscribe "/boxes" jsk_recognition_msgs::BoundingBoxArray #'box-callback)


(do-until-key
  (ros::spin-once)
  (print (send *target-cube* :pos))
  (unix::sleep 2))

(setq *result-data* (float-vector 0 0 0 0))

(setq pre-pose #f(120 10 40 0 -40 -20 100))
(setq grasp-pose #f(120 -40 130 0 -120 -40 100))

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
(send *hand* :angle-vector (send *hand_model5* :angle-vector pre-pose) :fast :default-controller 0 :min-time 0.05)

(send *hironxjsk* :larm :inverse-kinematics (make-coords :pos #f(590 200 100) :rpy (float-vector (deg2rad -110) (deg2rad -180) (deg2rad 110))) :move-target new-endcoords)
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)
(while t
    (ros::spin-once)
    (when (send *target-cube* :pos) (return nil))
    (ros::sleep)
    )
(setq x-pos (+ (elt (send *target-cube* :pos) 0) 50))
(setq y-pos (elt (send *target-cube* :pos) 1))

(while t
    (send *hironxjsk* :larm :inverse-kinematics 
	  (make-coords :pos (float-vector x-pos y-pos 0) :rpy (float-vector (deg2rad -110) (deg2rad -180) (deg2rad 110))) :move-target new-endcoords)
    (send *ri* :angle-vector (send *hironxjsk* :angle-vector))
    (send *ri* :wait-interpolation)
    (do-until-key (unix::sleep 1))

    (send *hironxjsk* :larm :inverse-kinematics
	  (make-coords :pos (float-vector x-pos y-pos -65) :rpy (float-vector (deg2rad -110) (deg2rad -180) (deg2rad 110))) :move-target new-endcoords)
    (send *ri* :angle-vector (send *hironxjsk* :angle-vector))
    (send *ri* :wait-interpolation)
    (send *hand* :angle-vector (send *hand_model5* :angle-vector grasp-pose) :fast :default-controller 0 :min-time 0.05)
  (unix::sleep 2)  
  (ros::spin-once)
  (ros::sleep)
   (if  (< (elt *result-data* 0) 90) 
        (progn (print *result-data*) (return nil))
    (send *hand* :angle-vector (send *hand_model5* :angle-vector pre-pose) :fast :default-controller 0 :min-time 0.05)
  )
)

(send *hironxjsk* :larm :inverse-kinematics
      (make-coords :pos #f(590 80 160) :rpy (float-vector (deg2rad -110) (deg2rad -180) (deg2rad 110))) :move-target new-endcoords)
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)

(send *hironxjsk* :larm :inverse-kinematics (make-coords :pos #f(590 80 160) :rpy (float-vector (deg2rad -100) (deg2rad -180) (deg2rad 100))) :move-target new-endcoords)
(send *hironxjsk* :rarm :inverse-kinematics (make-coords :pos #f(500 30 110) :rpy (float-vector (deg2rad -60) (deg2rad 0) (deg2rad 110))))
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)
(send *ri* :wait-interpolation)
(unix::sleep 1)
(send *hand* :angle-vector (send *hand_model5* :angle-vector pre-pose) :fast :default-controller 0 :min-time 0.05)

(setq flag 0)
(send *hironxjsk* :rarm :move-end-rot 20 :x)

(while (or (= flag 0))
  (send *hironxjsk* :rarm :move-end-pos (float-vector -5 10 0))
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector))
  (send *ri* :wait-interpolation)
  (ros::spin-once)
;;  (when (or (< (elt *result-data* 1) 70) (< (elt *result-data* 3) 70))
  (when (< (elt *result-data* 1) 50)
  (return nil)
) 
  (unix::sleep 1)
  (send *hironxjsk* :rarm :move-end-pos (float-vector 0 0 5))
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector))
  (send *ri* :wait-interpolation)
  (ros::spin-once)
;;  (when (or (< (elt *result-data* 1) 70) (< (elt *result-data* 3) 70))
  (when (< (elt *result-data* 1) 50) 
  (return nil)
  )
  (unix::sleep 1)

)

(send *hand* :angle-vector (send *hand_model5* :angle-vector grasp-pose) :fast :default-controller 0 :min-time 0.05)
(send *hironxjsk* :rarm :inverse-kinematics (make-coords :pos #f(320 -80 110) :rpy (float-vector (deg2rad -60) (deg2rad 0) (deg2rad 110))))
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)

(do-until-key
 (send *hand* :angle-vector (send *hand_model5* :angle-vector #f(60 -30 120 -70 -120 -40 100)) :fast :default-controller 0 :min-time 0.05)
 (unix::sleep 2)
 (send *hand* :angle-vector (send *hand_model5* :angle-vector #f(120 -30 120 -70 -120 -40 100)) :fast :default-controller 0 :min-time 0.05)
 (unix::sleep 2)
 )
;;引き出し開け閉め

(setq pre-pose #f(40 10 40 0 -60 -20 100))
(send *hand* :angle-vector (send *hand_model5* :angle-vector pre-pose) :fast :default-controller 0 :min-time 0.05)

(send *hironxjsk* :larm :inverse-kinematics 
      (make-coords :pos #f(300 0 250) :rpy (float-vector (deg2rad -90) (deg2rad 0) (deg2rad 40))))
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)

(send *hironxjsk* :larm :inverse-kinematics 
      (make-coords :pos #f(560 0 110) :rpy (float-vector (deg2rad -90) (deg2rad 0) (deg2rad 40))))
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)

(send *hironxjsk* :larm :inverse-kinematics
      (make-coords :pos #f(580 0 110) :rpy (float-vector (deg2rad -90) (deg2rad 0) (deg2rad 40))))
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)

(send *hironxjsk* :larm :inverse-kinematics
      (make-coords :pos #f(580 0 90) :rpy (float-vector (deg2rad -90) (deg2rad 0) (deg2rad 45))))
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)

(send *hand* :angle-vector (send *hand_model5* :angle-vector #f(40 -20 90 0 -100 -10 290)) :fast :default-controller 0 :min-time 0.05)

(dotimes (i 9)
    (setq pos (- 580 (* 30 i)))
    (send *hironxjsk* :larm :inverse-kinematics
      (make-coords :pos (float-vector pos 0 90) :rpy (float-vector (deg2rad -90) (deg2rad 0) (deg2rad 45))))
    (send *ri* :angle-vector (send *hironxjsk* :angle-vector))
    (send *ri* :wait-interpolation)
    )
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)

;;ペン持ち上げ

(while t
    (send *hironxjsk* :larm :inverse-kinematics
          (make-coords :pos (float-vector x-pos y-pos 0) :rpy (float-vector (deg2rad -110) (deg2rad -180) (deg2rad 100))) :move-target new-endcoords)
    (send *ri* :angle-vector (send *hironxjsk* :angle-vector))
    (send *ri* :wait-interpolation)
    (do-until-key (unix::sleep 1))

    (send *hironxjsk* :larm :inverse-kinematics
          (make-coords :pos (float-vector x-pos y-pos -48) :rpy (float-vector (deg2rad -110) (deg2rad -180) (deg2rad 100))) :move-target new-endcoords)
    (send *ri* :angle-vector (send *hironxjsk* :angle-vector))
    (send *ri* :wait-interpolation)
    (send *hand* :angle-vector (send *hand_model5* :angle-vector grasp-pose) :fast :default-controller 0 :min-time 0.05)
  (unix::sleep 2)
  (ros::spin-once)
  (ros::sleep)
   (if  (< (elt *result-data* 0) 90)
        (progn (print *result-data*) (return nil))
    (send *hand* :angle-vector (send *hand_model5* :angle-vector pre-pose) :fast :default-controller 0 :min-time 0.05)
  )
)

(send *hironxjsk* :larm :inverse-kinematics
      (make-coords :pos #f(590 80 160) :rpy (float-vector (deg2rad -110) (deg2rad -180) (deg2rad 110))) :move-target new-endcoords)
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)



;;引き出し without skin

(setq pre-pose #f(40 10 40 0 -60 -20 100))
(send *hand* :angle-vector (send *hand_model5* :angle-vector pre-pose) :fast :default-controller 0 :min-time 0.05)

(send *hironxjsk* :larm :inverse-kinematics
      (make-coords :pos #f(300 0 250) :rpy (float-vector (deg2rad -90) (deg2rad 0) (deg2rad 40))))
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)



(send *hironxjsk* :larm :inverse-kinematics
      (make-coords :pos #f(560 0 115) :rpy (float-vector (deg2rad -90) (deg2rad 0) (deg2rad 40))))
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)

(send *hironxjsk* :larm :inverse-kinematics
      (make-coords :pos #f(580 0 115) :rpy (float-vector (deg2rad -90) (deg2rad 0) (deg2rad 40))))
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)

(send *hironxjsk* :larm :inverse-kinematics
      (make-coords :pos #f(580 0 90) :rpy (float-vector (deg2rad -90) (deg2rad 0) (deg2rad 45))))
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)

(send *hand* :angle-vector (send *hand_model5* :angle-vector #f(40 -20 90 0 -100 -10 290)) :fast :default-controller 0 :min-time 0.05)

(dotimes (i 5)
    (setq pos (- 580 (* 40 i)))
    (send *hironxjsk* :larm :inverse-kinematics
      (make-coords :pos (float-vector pos 0 82) :rpy (float-vector (deg2rad -90) (deg2rad 0) (deg2rad 50))))
    (send *ri* :angle-vector (send *hironxjsk* :angle-vector))
    (send *ri* :wait-interpolation)
    )
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)


;;ハサミタスク without 表皮
(setq pre-pose #f(120 -30 40 0 -40 -20 100))

(send *hironxjsk* :larm :inverse-kinematics
      (make-coords :pos #f(590 100 0) :rpy (float-vector (deg2rad -110) (deg2rad -180) (deg2rad 110))) :move-target new-endcoords)
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)
(do-until-key (unix::sleep 1))

(send *hironxjsk* :larm :inverse-kinematics
      (make-coords :pos #f(590 100 -70) :rpy (float-vector (deg2rad -110) (deg2rad -180) (deg2rad 110))) :move-target new-endcoords)
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)
(send *hand* :angle-vector (send *hand_model5* :angle-vector grasp-pose) :fast :default-controller 0 :min-time 0.05)

(send *hironxjsk* :larm :inverse-kinematics
      (make-coords :pos #f(590 100 70) :rpy (float-vector (deg2rad -110) (deg2rad -180) (deg2rad 110))) :move-target new-endcoords)
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)

(send *hironxjsk* :larm :inverse-kinematics (make-coords :pos #f(590 80 160) :rpy (float-vector (deg2rad -100) (deg2rad -180) (deg2rad 100))) :move-target new-endcoords)
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)
(send *hironxjsk* :rarm :inverse-kinematics (make-coords :pos #f(500 30 110) :rpy (float-vector (deg2rad -60) (deg2rad 0) (deg2rad 110))))
(send *ri* :angle-vector (send *hironxjsk* :angle-vector))
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)
(send *ri* :wait-interpolation)
(unix::sleep 1)
(send *hand* :angle-vector (send *hand_model5* :angle-vector pre-pose) :fast :default-controller 0 :min-time 0.05)

(send *hironxjsk* :rarm :move-end-rot 20 :x)


