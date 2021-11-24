# dynamixel_hand

このレポジトリの上のディレクトリで
```
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```
```
cd dynamixel_hand
rlwrap roseus euslisp/dxl-hand-interface.l
hand_model
setq *hand* (instance rhp3hand_l-interface :init)
send *hand* :angle-vector (send *hand_model* :angle-vector #f(0 0 0 0 0 0)) :fast :default-controller
```
