;;(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(load "package://pr2eus/robot-interface.l")
(require :rhp3hand_l "package://rhp_dynamixel_hand/models/rhp3hand_l.l")
(ros::load-ros-manifest "control_msgs")

(defclass rhp3hand_l-interface
  :super robot-interface;;rtm-ros-robot-interface
  :slots ())
(defmethod rhp3hand_l-interface
  (:init
    (&rest args &key ((:controller-timeout ct) 5))
    (prog1
      (send-super* :init :robot rhp3hand_l-robot :controller-timeout ct args)
      )
    )
  (:default-controller
    ()
    (list
      (list
        (cons :controller-action "left_hand_controller/follow_joint_trajectory_action")
        (cons :controller-state "left_hand_controller/state")
        (cons :action-type control_msgs::FollowJointTrajectoryAction)
        (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n)) (send-all (send robot :joint-list) :name)))
        )
      )
    )
  )

(defun rhp3hand_l-init (&rest args)
  (if (not (boundp '*ri*))
    (setq *ri* (instance* rhp3hand_l-interface :init args)))
  (if (not (boundp '*rhp3hand_l*))
    (setq *rhp3hand_l* (instance rhp3hand_l-robot :init)))
  )
