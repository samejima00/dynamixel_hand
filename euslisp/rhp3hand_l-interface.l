;;(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(load "package://pr2eus/robot-interface.l")
(require :rhp3hand_l "package://dynamixel_hand/euslisp/hand_model5.l")
(ros::load-ros-manifest "control_msgs")

(defclass rhp3hand_l-interface
  :super robot-interface;;rtm-ros-robot-interface
  :slots ())
(defmethod rhp3hand_l-interface
  (:init
    (&rest args);; (&rest args &key ((:controller-timeout ct) 5))
    (prog1
      (send-super* :init :robot hand_model5-robot args)
      (send self :add-controller :default-controller)
    )
    )
  (:default-controller
    ()
    (list
      (list
        (cons :controller-action "dynamixel_hand_controller/follow_joint_trajectory_action")
        (cons :controller-state "dynamixel_hand_controller/state")
        (cons :action-type control_msgs::FollowJointTrajectoryAction)
        (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n)) (send-all (send robot :joint-list) :name)))
        )
      )
    )
  )

(defun rhp3hand_l-init ()
  (if (not (boundp '*ri*))
    (setq *ri* (instance* rhp3hand_l-interface :init)))
  (if (not (boundp '*rhp3hand_l*))
    (setq *rhp3hand_l* (instance rhp3hand_l-robot :init)))
  )
