# dynamixel_hand

'''rlwrap roseus euslisp/dxl-hand-interface.l
hand_model
setq *hand* (instance rhp3hand_l-interface :init)
send *hand* :angle-vector (send *hand_model* :angle-vector #f(0 0 0 0 0 0)) :fast :default-controller 0 :min-time 0.05
'''
