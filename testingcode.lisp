(defclass world ()
  ((obstacles :initform (cons 14 14) :accessor obstacle-loc)
   (dimensions :initform (cons 25 25) :accessor dimensions)
   (nodes :initform '(0 0) :accessor nodes-loc)
   (agent-location :initform (cons 0 0) :accessor agent-coords)
   (agent-dir :initform 'N :accessor facing))) ;allows dimensions to be read

(defparameter *world*
  (make-instance 'world))

(defmethod make-nodes-list ((self world))
      (with-slots (nodes) self
         (setq nodes 
           (loop for n from 0 to (1- (car (dimensions *world*)))
               append (loop for m from 0 to (1- (cdr (dimensions *world*)))
                      collect (cons n m))))))

(print (make-nodes-list *world*))