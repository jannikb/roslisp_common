;;;; -*- Mode: LISP -*-

(defsystem "actionlib-test"
  :depends-on ("roslisp" "roslisp-utils" "lisp-unit" "actionlib")
  :components
  ((:module "test"
            :components
            ((:file "package")
             (:file "state-machine-test" :depends-on ("package"))))))
  