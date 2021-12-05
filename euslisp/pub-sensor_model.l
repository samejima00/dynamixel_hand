#!/usr/bin/env roseus

(ros::roseus "test_pub_marker")

(defvar *pub-cube-array-topic* "/cube_array_topic")
(defvar *pub-cube-topic* "/cube_topic")

(ros::advertise *pub-cube-array-topic* visualization_msgs::MarkerArray 5)
(ros::advertise *pub-cube-topic* visualization_msgs::Marker 5)
(unix:usleep (* 1000 1000))

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
;;     (send cube-array-msg :markers (list cube1-1msg cube1-2msg))
;;    ;; (ros::publish *pub-cube-topic* cube1-1msg)
;;     (ros::publish *pub-cube-array-topic* cube-array-msg)
;;     ))

(defun test-pub ()
  (let* ((cube-array-msg (instance visualization_msgs::MarkerArray :init))
	 (link-names (list "1st_Link" "1st_Link2" "2nd_Link" "2nd_Link2"))
	 (header-table (make-hash-table :test #'equal))
	 (cube-table (make-hash-table :test #'equal))
	 (color-table (make-hash-table :test #'equal))
	 cube-list)
    ;; Set header and cube for each link
    (dolist (link-name link-names)
      (sethash link-name header-table
	       (instance std_msgs::header :init
			 :stamp (ros::time-now)
			 :frame_id link-name))
      (sethash link-name cube-table
	       (list (make-cube 10 100 10) (make-cube 10 100 10) (make-cube 10 100 10)))
      (sethash link-name color-table
	       (list (float-vector 0 1 0) (float-vector 0 1 0) (float-vector 0 1 0))))
    ;; Move cube
    (send (elt (gethash "1st_Link" cube-table) 0)
	  :translate #f(10 0 10))
    ;; Set color-table based on sensor value
    ;; TODO (sethash link-name color-table (float-vector ...))
    ;; Append each cube into list
    (dolist (link-name link-names)
      (dotimes (i 3)
	(setq cube-list
	      (append cube-list (list (cube->marker-msg (elt (gethash link-name cube-table) i)
							(gethash link-name header-table)
							:color (float-vector 0 1 0)
							;; : color (elt (gethash link-name color-table) i)
							))))))
    ;; Publish
    (send cube-array-msg :markers cube-list)
    (ros::publish *pub-cube-array-topic* cube-array-msg)
    ))

(test-pub)

