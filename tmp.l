(defun color (min max val)
  (cond
    ((and (< min val) (< val max))
          (return-from color (float-vector 0 0 1)))
    (t float-vector 0 1 0))
  )

