(load "rhp3hand_l-interface.l")
(load "rhp3hand_r-interface.l")

(defun linit ()
  (rhp3hand_l-init)
  (setq *robot* *rhp3hand_l*)
  (objects (list *robot*))
  )

(defun rinit()
  (rhp3hand_r-init)
  (setq *robot* *rhp3hand_r*)
  (objects (list *robot*))
  )
