(in-package :cl-urdf)

(defun shallow-copy (original slots &optional (init nil))
  (let* ((class (class-of original))
         (copy (if init
                   (make-instance class)
                   (allocate-instance class))))
    (dolist (slot slots)
      (when (slot-boundp original slot)
        (setf (slot-value copy slot)
              (slot-value original slot))))
    copy))

(defun deep-copy (original shallow-slots deep-slots &optional (init nil))
  (let ((copy (shallow-copy original shallow-slots init)))
    (dolist (slot deep-slots)
      (when (slot-boundp original slot)
        (if (slot-value original slot)
            (setf (slot-value copy slot)
                  (copy-object (slot-value original slot)))
            (setf (slot-value copy slot) nil))))
    copy))
        
(defgeneric copy-object (original))

(defmethod copy-object (original)
  (warn "copy not implemented for class ~a." (class-of original))
  original)

(defmethod copy-object ((original cons))
  (mapcar (lambda (elem) (copy-object elem)) original))

(defmethod copy-object ((original string))
  (copy-seq original))

;;; Classes from robot

(defmethod copy-object ((original robot))
  (let ((copy (deep-copy original '(name) nil t)))
    (maphash (lambda (key value) (setf (gethash key (links copy)) (copy-object value)))
             (links original))
    (maphash (lambda (key value) (setf (gethash key (joints copy)) (copy-object value)))
             (joints original))
    (maphash (lambda (key value)
               (declare (ignore key))
               (let ((parent (gethash (parent-name value) (links copy)))
                     (child (gethash (child-name value) (links copy))))
                 (setf (parent value) parent)
                 (setf (child value) child)
                 (setf (from-joint child) value)
                 (setf (to-joints parent) (cons value (to-joints parent)))))
             (joints copy))                 
    (setf (root-link copy) (gethash (name (root-link original)) (links copy)))
    copy))
    

;;; Classes from link

(defmethod copy-object ((original link))
  (deep-copy original '(name) '(visual collision) t))

(defmethod copy-object ((original inertial))
  (deep-copy original '(mass inertia) '(origin)))

(defmethod copy-object ((original collision))
  (deep-copy original nil '(origin geometry)))

(defmethod copy-object ((original visual))
  (deep-copy original nil '(origin geometry material)))

(defmethod copy-object ((original box))
  (deep-copy original '(size) nil))

(defmethod copy-object ((original cylinder))
  (deep-copy original '(radius length) nil))

(defmethod copy-object ((original sphere))
  (deep-copy original '(radius) nil))

(defmethod copy-object ((original mesh))
  (deep-copy original '(filename scale size) nil))

(defmethod copy-object ((original material))
  (deep-copy original '(name color texture) nil))

;;; Classes from joint

(defmethod copy-object ((original joint))
  (deep-copy original
             '(name parent-name child-name type)
             '(origin axis limits)))

(defmethod copy-object ((original limits))
  (deep-copy original '(upper lower effort velocity) nil))

(defmethod copy-object ((original cl-transforms:transform))
  (deep-copy original nil '(cl-transforms:translation cl-transforms:rotation)))

(defmethod copy-object ((original cl-transforms:3d-vector))
  (deep-copy original '(cl-transforms:x cl-transforms:y cl-transforms:z) nil))

(defmethod copy-object ((original cl-transforms:quaternion))
  (deep-copy original
             '(cl-transforms:x cl-transforms:y cl-transforms:z cl-transforms:w)
             nil))


