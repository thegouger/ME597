
(cl:in-package :asdf)

(defsystem "indoor_pos-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ips_msg" :depends-on ("_package_ips_msg"))
    (:file "_package_ips_msg" :depends-on ("_package"))
  ))