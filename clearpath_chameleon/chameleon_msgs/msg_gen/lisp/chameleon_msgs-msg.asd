
(cl:in-package :asdf)

(defsystem "chameleon_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ChameleonCmdSpeed" :depends-on ("_package_ChameleonCmdSpeed"))
    (:file "_package_ChameleonCmdSpeed" :depends-on ("_package"))
    (:file "ChameleonSense" :depends-on ("_package_ChameleonSense"))
    (:file "_package_ChameleonSense" :depends-on ("_package"))
    (:file "ChameleonCmdPWM" :depends-on ("_package_ChameleonCmdPWM"))
    (:file "_package_ChameleonCmdPWM" :depends-on ("_package"))
  ))