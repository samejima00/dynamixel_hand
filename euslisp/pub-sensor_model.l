#!/usr/bin/env roseus

(ros::roseus "test_pub_marker")
(ros::roseus-add-msgs "std_msgs")

(defvar *pub-cube-array-topic* "/cube_array_topic")
(defvar *pub-cube-topic* "/cube_topic")
(defvar *sub-sensor-data* "/touch_data")

(defun callback (msg)
  ;;(print (list 'a (send msg :data)))
  (setq *result-msg* (send msg :data)))

(ros::advertise *pub-cube-array-topic* visualization_msgs::MarkerArray 5)
(ros::advertise *pub-cube-topic* visualization_msgs::Marker 5)
(ros::subscribe "/touch_data" std_msgs::Float32MultiArray #'callback)

(unix:usleep (* 1000 1000))

(setq *result-msg* (float-vector 0 0 0 0))


(defun color (max min val)
  (cond
    ((and (< min val) (< val max))
	  (return-from color (float-vector 0 0 1)))
    (t (float-vector 0 1 0)))
  )

(setq sensor-data (make-matrix 4 2
      '((550 680)
      (610 755)
      (470 670)
      (650 760))))

;; (defun test-pub ()
;;   (let* ((cube-array-msg (instance visualization_msgs::MarkerArray :init))
;; 	 (cube1-1 (make-cube 10 100 10))
;; 	 (cube1-2 (make-cube 10 100 10))
;; 	 (cube1-3 (make-cube 10 100 10))		  
;; 	 (cube1-4 (make-cube 10 100 10))
;; 	 (cube1-5 (make-cube 10 100 10)) 
;;          (cube1-6 (make-cube 10 100 10))
;; 	 (cube2-1 (make-cube 10 100 10)) 
;;          (cube2-2 (make-cube 10 100 10))
;; 	 (cube2-3 (make-cube 10 100 10))
;; 	 (cube2-4 (make-cube 10 100 10))
;; 	 (cube2-5 (make-cube 10 100 10))
;;          (cube2-6 (make-cube 10 100 10))
;; 	 (header1 (instance std_msgs::header :init
;; 			    :stamp (ros::time-now)
;; 			    :frame_id "1st_Link"))
;; 	 (header2 (instance std_msgs::header :init
;; 			    :stamp (ros::time-now)
;; 			    :frame_id "1st_Link2"))
;; 	 (header3 (instance std_msgs::header :init
;; 			    :stamp (ros::time-now)
;; 			    :frame_id "2nd_Link"))
;; 	 (header4 (instance std_msgs::header :init
;; 			    :stamp (ros::time-now)
;; 			    :frame_id "2nd_Link2"))
;;          cube1-1msg cube1-2msg cube1-3msg cube1-4msg cube1-5msg cube1-6msg
;;          cube2-1msg cube2-2msg cube2-3msg cube2-4msg cube2-5msg cube2-6msg)
;;     (send cube1-1 :translate #f(10 0 10))
;;     (setq cube1-1msg (cube->marker-msg cube1-1 header1
;; 				  :color (float-vector 0 1 0)))
;;     (setq cube1-2msg (cube->marker-msg cube1-2 header1
;; 				  :color (float-vector 0 1 0)))
;;     (setq cube1-3msg (cube->marker-msg cube1-3 header1
;; 				  :color (float-vector 0 1 0)))
;;     (setq cube1-4msg (cube->marker-msg cube1-4 header2
;; 				  :color (float-vector 0 1 0)))
;;     (setq cube1-5msg (cube->marker-msg cube1-5 header2
;; 				  :color (float-vector 0 1 0)))
;;     (setq cube1-6msg (cube->marker-msg cube1-6 header2
;; 				  :color (float-vector 0 1 0)))
;;     (setq cube2-1msg (cube->marker-msg cube2-1 header3
;; 				  :color (float-vector 0 1 0)))
;;     (setq cube2-2msg (cube->marker-msg cube2-2 header3
;; 				  :color (float-vector 0 1 0)))
;;     (setq cube2-3msg (cube->marker-msg cube2-3 header3
;; 				  :color (float-vector 0 1 0)))
;;     (setq cube2-4msg (cube->marker-msg cube2-4 header4
;; 				  :color (float-vector 0 1 0)))
;;     (setq cube2-5msg (cube->marker-msg cube2-6 header4
;; 				  :color (float-vector 0 1 0)))
;;     (setq cube2-6msg (cube->marker-msg cube2-6 header4
;; 				  :color (float-vector 0 1 0)))
;;     ;;
;;     (setq cube-array-msg (instance visualization_msgs::MarkerArray :init))
;;     (send cube-array-msg :markers (list cube1-1msg cube1-2msg))
;;    ;; (ros::publish *pub-cube-topic* cube1-1msg)
;;     (ros::publish *pub-cube-array-topic* cube-array-msg)
;;     )

(defun test-pub ()
 (do-until-key
  (let* ((cube-array-msg (instance visualization_msgs::MarkerArray :init))
	 (link-names (list "1st_Link" "1st_Link2" "2nd_fing_Link" "2nd_fing_Link2"))
	 (header-table (make-hash-table :test #'equal))
	 (cube-table (make-hash-table :test #'equal))
	 (color-table (make-hash-table :test #'equal))
	 (sensor-table (make-hash-table :test #'equal))
	 cube-list)
    ;; Set header and cube for each link
    (dolist (link-name link-names)
      (sethash link-name header-table
	       (instance std_msgs::header :init
			 :stamp (ros::time-now)
			 :frame_id link-name))
      (sethash link-name cube-table
	       (list (make-cube 10 1 10) (make-cube 10 1 10) (make-cube 10 1 10) (make-cube 10 1 10)))
      (sethash link-name color-table
	       (list (float-vector 0 1 0) (float-vector 0 1 0) (float-vector 0 1 0) (float-vector 0 1 0))))
   ;; Move cube
    (send (elt (gethash "1st_Link" cube-table) 0)
	  :translate #f(-8 -5 20))
    (send (elt (gethash "1st_Link" cube-table) 1)
	  :translate #f(-8 -5 10))
    (send (elt (gethash "1st_Link" cube-table) 2)
	  :translate #f(-8 5 20))
    (send (elt (gethash "1st_Link" cube-table) 3)
	  :translate #f(-8 5 10))
    (send (elt (gethash "1st_Link2" cube-table) 0)
	  :translate #f(-8 -3 30))
    (send (elt (gethash "1st_Link2" cube-table) 1)
	  :translate #f(-8 -3 10))
    (send (elt (gethash "1st_Link2" cube-table) 2)
	  :translate #f(-8 7 30))
    (send (elt (gethash "1st_Link2" cube-table) 3)
	  :translate #f(-8 7 10))

    (send (elt (gethash "2nd_fing_Link" cube-table) 0)
	  :rotate -0.5 :x)
    (send (elt (gethash "2nd_fing_Link" cube-table) 1)
	  :rotate -0.5 :x)
    (send (elt (gethash "2nd_fing_Link" cube-table) 2)
	  :rotate -0.5 :x)
    (send (elt (gethash "2nd_fing_Link" cube-table) 3)
	  :rotate -0.5 :x)
 
    (send (elt (gethash "2nd_fing_Link" cube-table) 0)
	  :translate #f(-8 -3 20))
    (send (elt (gethash "2nd_fing_Link" cube-table) 1)
	  :translate #f(-8 -3 10))
    (send (elt (gethash "2nd_fing_Link" cube-table) 2)
	  :translate #f(-8 7 20))
    (send (elt (gethash "2nd_fing_Link" cube-table) 3)
	  :translate #f(-8 7 10))
    (send (elt (gethash "2nd_fing_Link2" cube-table) 0)
	  :translate #f(-8 -3 30))
    (send (elt (gethash "2nd_fing_Link2" cube-table) 1)
	  :translate #f(-8 -3 10))
    (send (elt (gethash "2nd_fing_Link2" cube-table) 2)
	  :translate #f(-8 7 30))
    (send (elt (gethash "2nd_fing_Link2" cube-table) 3)
	  :translate #f(-8 7 10))

    (print *result-msg*)

    ;; Set color-table based on sensor value
    ;; TODO (sethash link-name color-table (float-vector ...))
    ;; Append each cube into list
    (setq link-sets (list (list "1st_Link2" "1st_Link" ) (list "2nd_fing_Link2" "2nd_fing_Link" )))
    (dotimes (i 2)
      (setq id (* i 2))
      (setq minval_front (aref sensor-data id 0))
      (setq maxval_front (aref sensor-data id 1))
      (setq diff_front (/ (- maxval_front minval_front) 4))
      (setq minval_back (aref sensor-data (+ id 1) 0))
      (setq maxval_back (aref sensor-data (+ id 1) 1))
      (setq diff_back (/ (- maxval_back minval_back) 4))
      (sethash (elt (elt link-sets i) 0) color-table (list (color maxval_front (- maxval_front diff_front) (elt *result-msg* id)) 
					     (color (- maxval_front diff_front) (- maxval_front (* 2 diff_front)) (elt *result-msg* id)) 
					     (color maxval_back (- maxval_back diff_back) (elt *result-msg* (+ id 1))) 
                                             (color (- maxval_back diff_back) (- maxval_back (* 2 diff_back)) (elt *result-msg* (+ id 1))))) 

      (sethash (elt (elt link-sets i) 1) color-table (list (color (+ minval_front (* 2 diff_front)) (+ minval_front diff_front) (elt *result-msg* id)) 
					     (color (+ minval_front diff_front) 0 (elt *result-msg* id))
					     (color (+ minval_back (* 2 diff_back)) (+ minval_back diff_back) (elt *result-msg* (+ id 1))) 
					     (color (+ minval_back diff_back) 0 (elt *result-msg* (+ id 1)))
					     ))
      )

    (print (gethash "1st_Link" color-table))
    (print (gethash "1st_Link2" color-table))
    (dolist (link-name link-names)
      (dotimes (i 4)
	(setq cube-list
	      (append cube-list (list (cube->marker-msg (elt (gethash link-name cube-table) i)
							(gethash link-name header-table)
							;;:color (float-vector 0 1 0)
							:color (elt (gethash link-name color-table) i)
     ))))))
    (setq cube-array-msg (instance visualization_msgs::MarkerArray :init))
    ;; Publish
    ;;(send cube-array-msg :markers cube-list)
    (dotimes (i 16)
      (send (elt cube-list i) :id i)
      )
    (send cube-array-msg :markers cube-list)
    (ros::publish *pub-cube-array-topic* cube-array-msg)
    (x::window-main-one)
    (ros::spin-once)
    (ros::sleep)
    )))

(test-pub)

