(in-package :actionlib)

(defparameter *states* 
  ;;              State   Signal     Target-State
  (make-states '((:done (:send-goal :waiting-for-goal-ack))
                 (:waiting-for-goal-ack (:cancel-goal :waiting-for-cancel-ack
                                         :pending :pending
                                         :active :active
                                         :recalling :recalling
                                         :preempting :preempting
                                         :rejected :waiting-for-result
                                         :recalled :waiting-for-result
                                         :preempted :waiting-for-result
                                         :succeeded :waiting-for-result
                                         :aborted :waiting-for-result
                                         :receive :done))
                 (:pending (:cancel-goal :waiting-for-cancel-ack
                            :active :active
                            :recalling :recalling
                            :preempting :preempting
                            :rejected :waiting-for-result
                            :recalled :waiting-for-result
                            :preempted :waiting-for-result
                            :succeeded :waiting-for-result
                            :aborted :waiting-for-result
                            :receive :done))
                 (:active (:cancel-goal :waiting-for-cancel-ack
                           :preempting :preempting
                           :preempted :waiting-for-result
                           :succeeded :waiting-for-result
                           :aborted :waiting-for-result
                           :receive :done))
                 (:waiting-for-cancel-ack (:recalling :recalling
                                           :preempting :preempting
                                           :rejected :waiting-for-result
                                           :recalled :waiting-for-result
                                           :preempted :waiting-for-result
                                           :succeeded :waiting-for-result
                                           :aborted :waiting-for-result
                                           :receive :done))
                 (:recalling (:preempting :preempting
                              :rejected :waiting-for-result
                              :recalled :waiting-for-result
                              :preempted :waiting-for-result
                              :succeeded :waiting-for-result
                              :aborted :waiting-for-result
                              :receive :done))
                 (:preempting (:preempted :waiting-for-result
                               :succeeded :waiting-for-result
                               :aborted :waiting-for-result
                               :receive :done))
                 (:waiting-for-result (:receive :done)))))

(defclass comm-state-machine (state-machine)
  ((current-state :initform (getf *states* :waiting-for-goal-ack))
   (states :initform *states*)
   (goal-id :initarg :goal-id
            :reader goal-id)
   (transition-cb :initarg :transition-cb
                  :initform nil
                  :reader transition-cb)
   (feedback-cb :initarg :feedback-cb
                :initform nil
                :reader :feedback-cb)
   (send-goal-fn :initarg :send-goal-fn
                 :reader send-goal-fn)
   (send-cancel-fn :initarg :send-cancel-fn
                   :reader send-cancel-fn)
   (latest-goal-status :initform :waiting-for-goal-ack
                       :accessor latest-goal-status)
   (latest-result :initform nil
                  :accessor latest-result)))

(defgeneric transition-to (csm goal-handle signal))

(defgeneric update-status (csm goal-handle state-name))

(defgeneric update-result (csm goal-handle action-result))

(defgeneric update-feedback (csm goal-handle action-feedback))


;;;Implementation

(defmethod transition-to ((csm comm-state-machine) goal-handle signal)
  (if (process-signal csm signal)
      (if transition-cb
          (funcall transition-cb goal-handle))))

(defmethod update-status ((csm comm-state-machine) goal-handle state-name)
  nil)

(defmethod update-result ((csm comm-state-machine) goal-handle action-result)
  nil)

(defmethod update-feedback ((csm comm-state-machine) goal-handle action-feedback)
  nil)
