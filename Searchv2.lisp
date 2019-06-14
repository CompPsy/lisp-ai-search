;;>>> My comments look like this

;;>>> In general, please keep lines <=80 characters.

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;
;;;;; This file contains the code for creating a virtual simulation where agents
;;;;; can perform different search methods, including basic reflex and 
;;;;; model-based reflex agents, as well as agents for hill-climbing, 
;;;;; breadth-first and A* searches 
;;;;;
;;;;; Created: 3/1/2019
;;;;; Author: Elizabeth Thompson
;;;;; Modifications:
;;;;;       - 6/14/2019 - edited comments based on feedback; altered/improved
;;;;;           existing code to streamline project/provide clarity
;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;
;;; Class: world
;;; Slots: 
;;;    - obstacles: a list that holds xy coord of all obstacles in the world
;;;    - parameters: an xy pair denoting the height and width of the search grid
;;;    - agent-location: an xy pair that denotes the current location of the 
;;;             agent in the world
;;; Description: 
;;;     This class creates the search world that the agent will act in
;;; Author: Elizabeth Thompson
;;; Created 3/2/2019
;;; Modications: none
;;;
(defclass World ()
  ((obstacles :initform (cons 14 14) :accessor obstacle-loc)
   (parameters :initform (cons 25 25) :accessor parameters)
   (nodes :initform (cons 0 0) :accessor nodes-loc)
   (agent-location :initform (cons 0 0) :accessor agent-coords)
   (agent-dir :initform 'N :accessor facing))) ;allows dimensions to be read


;;;
;;; Variable: *world*
;;; Description:
;;;     creates a default global variable of the world
;;; Modified:
;;;     - 6/14/2019 streamlined code based on feedback from Dr. Turner
(defparameter *world*
  (make-instance 'world :parameters (cons 26 26)))


;;;
;;; Method: makes-nodes-list
;;; Arguments:
;;;     -world: the world the search occurs in
;;; Returns: a list of available nodes as x.y cons cells
;;; Description: 
;;;     Uses the parameters of the world to create a list of xy cons cells
;;; Author: Elizabeth Thompson
;;; Created 3/5/2019
;;; Modications:
;;;     - 6/14/19 - changed from function to method and altered code to
;;;           create a list of available nodes stored in the 'node' slot        
(defmethod make-nodes-list ((self world))
      (with-slots (nodes) self
         (setq nodes 
           (loop for n from 0 to (1- (car (dimensions *world*)))
               append (loop for m from 0 to (1- (cdr (dimensions *world*)))
                      collect (cons n m))))))


;;;
;;; Class: simulator
;;; Slots:
;;;     -rules: the heuristic used by the current search agent
;;;     -moves-counter: tracks the number of moves made
;;; Description: 
;;;     This is the simulator that controls movement of the agent
;;;     during the simulation
;;; Author: Elizabeth Thompson
;;; Created 3/5/2019
;;; Modications: none
(defclass simulator ()
  ((rules :initargs :rules-list :initform "Please add rules"
        :accessor rules-list)
   (moves-counter :initform 0 :accessor moves-counter)))


;;;creates an instance of the simulator
(defparameter *simulator*
  (make-instance 'simulator))


;;;-
;;; Class: agent
;;; Slots: 
;;;     - agent-x: x coord of the agent
;;;     - agent-y: y coord of the agent
;;; Description: 
;;;     The basic agent class
;;; Author: Elizabeth Thompson
;;; Created 3/2/2019
;;; Modications:
;;;     - 6/14/2019 removed out of date information re: facing direction
(defclass agent ()
    ((agent-x :initargs :start-x :initform 0 :accessor agent-x)  
     (agent-y :initargs :start-y :initform 0 :accessor agent-y)))


;;;creates an instance of the agent class
(defparameter *agent*
  (make-instance 'agent))
-

;;;
;;; Class: reflex-agent
;;; Slots: 
;;;     - sensors: sensors that the agent uses to detect the world around it
;;;     - corner-found: flag for if the agent has found a corner
;;; Description: 
;;;     This class inherits from agent class and includes necessary sensors 
;;;     and checks for a reflex agent including that a reflex agent has no 
;;;     knowledge of the world outside of its sensors.
;;; Author: Elizabeth Thompson
;;; Created 3/3/2019
;;; Modications: none
(defclass reflex-agent (agent) (
    (front-sensor :initform nil :accessor front-sensor)
    (front-bump :initform nil :accessor front-bump)
    (left-bump :initform nil :accessor left-bump)
    (back-bump :initform nil :accessor back-bump)
    (right-bump :initform nil :accessor right-bump)
    (corner-found :initform nil :accessor corner-found)))

;;; Global parameter for a reflex agent
(defparameter *reflex-agent*
  (make-instance 'reflex-agent))

;;;
;;; Class: model-reflex-agent
;;; Slots: 
;;;     - sensors: sensors that the agent uses to detect the world around it
;;;     - mapped: list for storing visited locations
;;;     - corner-found: flag for when/if the agent has found a corner
;;; Description:
;;;     This class inherits from agent class and includes necessary sensors and
;;;     checks for a model-reflex agent including the ability to store visited 
;;;     locations data.
;;; Author: Elizabeth Thompson
;;; Created 3/4/2019
;;; Modications: none
(defclass model-agent (agent) (
    (front-sensor :initform nil :accessor front-sensor)
    (front-bump :initform nil :accessor front-bump)
    (left-bump :initform nil :accessor left-bump)
    (back-bump :initform nil :accessor back-bump)
    (right-bump :initform nil :accessor right-bump)
    (mapped :initform (cons 0 0) :accessor mapped)
    (corner-found :initform nil :accessor corner-found)))


;;;global parameter for a model-agent
(defparameter *model-agent*
    (make-instance 'model-agent))


;;;				       
;;; Class: hill-climber-agent
;;; Slots: 
;;;     - sensors: sensors that the agent uses to detect the world around it
;;;     - mapped: list for storing visited locations
;;;     - goal: xy coordinates of the goal location on the map
;;;     - goal-found: flag to detect if the goal has been reached.
;;; Description: 
;;;     This class inherits from agent class and includes necessary sensors and
;;;     checks for an hill climbing agent including the ability to store visited 
;;;     locations data, and calculate the distance from the current node to
;;;     the goal node.
;;; Author: Elizabeth Thompson
;;; Created 3/5/2019
;;; Modications: none
(defclass hill-climbing-agent (agent) (
    (front-sensor :initform nil :accessor front-sensor)
    (front-bump :initform nil :accessor front-bump)
    (left-bump :initform nil :accessor left-bump)
    (back-bump :initform nil :accessor back-bump)
    (right-bump :initform nil :accessor right-bump)
    (mapped :initform nil :accessor mapped)               ;;stores the list of visited nodes
    (goal :initform (cons 24 24) :accessor goal-coords)   ;;goal coordinates
    (goal-found :initform nil :accessor goal-found)))     ;;flag for it goal was found

;;;global parameter for a hill-climbing-agent
(defparameter *hill-climbing*
    (make-instance 'hill-climbing-agent))




;;;				       
;;; Class: breadth-first-agent
;;; Slots: 
;;;     - search-nodes: a list of nodes to search through
;;;     - goal: xy coordinates of the goal location on the map
;;;     - goal-found: flag to detect if the goal has been reached.
;;; Description: 
;;;     This class inherits from agent class and includes a goal
;;;     corrdinate slot, a boolean for if the goal has been found and
;;;     a list of nodes to search through for the goal. 
;;; Author: Elizabeth Thompson
;;; Created 3/8/2019
;;; Modications: none
(defclass breadth-first-agent (agent)
  (   (goal :initform (cons 24 24) :accessor goal-coords)
      (goal-found :initform nil :accessor goal-found)
      (search-nodes :initform nil :accessor search-list)))

;;; global parameter of a breadth-first-agent
(defparameter *breadth-agent*
  (make-instance 'breadth-first-agent))


;;;A* opens up all nodes on the frontier with the lowest cost as
;;;determined by the h weight, frontier is contastly updated.
(defclass A*-agent (agent) (
    (mapped :initform nil :accessor mapped) 
    (goal :initform (cons 24 24) :accessor goal-coords)
    (goal-found :initform nil :accessor goal-found)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; this is 80 characters to measure

;;;
;;; Function: update-precept
;;; Arguments: 
;;;     - agent: the agent running in the search
;;;     - precept: a set of precept instructions corresponding to the
;;;       different sensors of the agent.
;;; Returns: updated list of precepts, otherwise nil
;;; Description: 
;;;    This function updates the precept list such that it can take a
;;;    full set of precepts as a list and update the corresponding
;;;    sensors on the agent. 
;;; Author: Elizabeth Thompson
;;; Created 3/6/2019
;;; Modications:
;;;     - 6/14/2019 - updated code per feedback from Dr. Turner to improve
;;;             clarity and quality of code/performance
(defmethod update-precept ((self agent) precept)
    (with-slots (front-sensor front-bump left-bump right-bump back-bump) self
        (multiple-value-setq (front-sensor
                front-bump
                left-bump
                right-bump
                back-bump)
            (values-list percept))))+


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; this is 80 characters to measure

;;;
;;; Function: can-move
;;; Arguments: 
;;;     - agent: the agent running in the search
;;; Returns: move function if a move is possible, otherwise nil
;;; Description: 
;;;    This function accesses an agents sensors and checks for possible move
;;;    options, then adds those that return true to a list. The agent then moves
;;;    in the direction of the first valid option in the list, if any. 
;;; Author: Elizabeth Thompson
;;; Created 3/3/2019
;;; Modications:
;;;     - 6/14/19 - updated/streamlined code based on feedback from Dr. Turner
(defmethod can-move ((self agent)) 
    (with-slots (front-sensor left-bump right-bump back-bump) self
        (let ((dir (member t (list (cons front-sensor f)
                    (cons left-bump l)
    		        (cons right-bump r)
			        (cons back-bump b)))))
            (when dir
                (move self (cdr dir))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;TAKE OUT AT END, LEAVE IN NOW FOR REFERENCE;;;;;;;;;
;;>>>     ;; Cooler way:
;;>>>     (defmethod can-move ((self agent)) 
;;>>>       (with-slots (front-sensor left-bump right-bump back-bump) self
;;>>>         (let ((pos (search '(t) (list front-sensor left-bump right-bump back-bump))))
;;>>>           (when pos
;;>>>     	(move self (elt '(f l r b) pos))))))
;;>>>     
;;>>>     ;;>>> Want to be really opaque here?  You could do:
;;>>>     
;;>>>     (defmethod can-move ((self agent)) 
;;>>>       (with-slots (front-sensor left-bump right-bump back-bump) self
;;>>>         (and (or front-sensor (move self 'f))
;;>>>     	 (or left-bump (move self 'l))
;;>>>     	 (or right-bump (move self 'r))
;;>>>     	 (or back-bump (move self 'b))
;;>>>     	 nil)))				; but function always returns nil now, so...
;;>>>
;;>>> 
;;>>> Okay, I'm done playing with Lisp now...


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; this is 80 characters to measure

;;;
;;; Function: can-move-dir
;;; Arguments: 
;;;     - agent: the agent running in the search
;;;     - letter: the direction to check for movement in
;;; Returns: move function if a move is possible, otherwise nil
;;; Description: 
;;;    This function checks all of an agent's possible moves in the direction
;;;    pass into it and calls move if any options are valid, otherwise it
;;;    returns nil when no moves are possible. 
;;; Author: Elizabeth Thompson
;;; Created 3/5/2019
;;; Modications: none
(defmethod can-move-dir (agent letter)
    (case letter
        (F (if (not (front-sensor agent))
                (move agent 'F)
                (progn (turn agent 'R) (reset-sensors agent) 
                    (setf (left-bump agent) t))))
        (L (if (not (left-bump agent))
                (move agent 'L)
                (progn (turn agent 'R) (reset-sensors agent) 
                    (setf (left-bump agent) t))))
        (R (if (not (right-bump agent))
                (move agent 'R)
                (progn (turn agent 'R) (reset-sensors agent) 
                    (setf (left-bump agent) t))))
        (B (if (not (back-bump agent))
                (move agent 'B)
                (progn (turn agent 'R) (reset-sensors agent) 
                    (setf (left-bump agent) t))))))

;;;
;;; Function: turn
;;; Arguments: 
;;;     - agent: the agent running in the search
;;;     - letter: the direction to turn in
;;; Returns: nil
;;; Description: 
;;;     This function turns the agent in the desired direction and updates
;;;     the facing variable to reflect their new direction.
;;; Author: Elizabeth Thompson
;;; Created 3/3/2019
;;; Modications: none
(defmethod turn (agent letter)
  (case letter
    (R (case (facing *world*) (N (setf (facing *world*) 'E)
				 (reset-sensors agent))
	     (E (setf (facing *world*) 'S)
		(reset-sensors agent))
	     (S (setf (facing *world*) 'W)
		(reset-sensors agent))
	     (W (setf (facing *world*) 'N)
		(reset-sensors agent))))
    (L (case (facing *world*) (N (setf (facing *world*) 'W)
				 (reset-sensors agent))
	     (E (setf (facing *world*) 'N)
		(reset-sensors agent))
	     (S (setf (facing *world*) 'E)
		(reset-sensors agent))
	     (W (setf (facing *world*) 'S)
		(reset-sensors agent))))
	(otherwise nil)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; this is 80 characters to measure

;;;
;;; Function: move
;;; Arguments: 
;;;     - agent: the agent running in the search
;;;     - letter: the direction the agent tries to move in
;;; Returns: move function if a move is possible, otherwise nil
;;; Description: 
;;;    This function takes in an agent and a direction to move, checks if the
;;;    move is valid, and either updates the agent's xy coordinates if it is, 
;;;    or updates the agent's bump sensor if it encounters an obstacle.
;;; Author: Elizabeth Thompson
;;; Created 3/3/2019
;;; Modications: none
(defmethod move (agent letter)
    (case letter
        (F (case (facing *world*) 
            (N  (if (< (+ (agent-y agent) 1) 25) ;;fix parens on everything!!!
                    (progn (setf (agent-y agent) (+ (agent-y agent) 1))
                        (reset-sensors agent))
                        (setf (front-bump agent) t)))
            (E  (if (< (+ (agent-x agent) 1) 25)
                    (progn (setf (agent-x agent) (+ (agent-x agent) 1))
                        (reset-sensors agent)) 
                        (setf (front-bump agent) t)))
            (S  (if (> (- (agent-y agent) 1) -1)
                    (progn (setf (agent-x agent) (- (agent-y agent) 1))
                        (reset-sensors agent))
                            (setf (front-bump agent) t)))
            (W  (if (> (- (agent-x agent) 1) -1)
                    (progn (setf (agent-x agent) (- (agent-x agent) 1))
                        (reset-sensors agent))
                        (setf (front-bump agent) t)))))
        (L (case (facing *world*)
            (N  (if (> (- (agent-x agent) 1) -1)
                    (progn (setf (agent-x agent) (- (agent-x agent) 1))
                        (reset-sensors agent))
                        (setf (left-bump agent) t)))
            (E  (if (> (+ (agent-y agent) 1) 25)
                    (progn (setf (agent-y agent) (+ (agent-y agent) 1))
                        (reset-sensors agent))
                        (setf (left-bump agent) t)))
            (S  (if (< (+ (agent-x agent) 1) 25)
                    (progn (setf (agent-x agent) (+ (agent-x agent) 1))
                        (reset-sensors agent))
                        (setf (left-bump agent) t)))
            (W  (if (< (- (agent-y agent) 1) -1)
                    (progn (setf (agent-y agent) (- (agent-y agent) 1))
                        (reset-sensors agent))
                        (setf (left-bump agent) t)))))
	 
	(R (case (facing *world*) 
            (N  (if (< (+ (agent-x agent) 1) 25)
                    (progn (setf (agent-x agent) (+ (agent-x agent) 1))
                        (reset-sensors agent))
                    (setf (right-bump agent) t)))    
            (E  (if (> (- (agent-y agent) 1) -1)
                    (progn (setf (agent-y agent) (- (agent-y agent) 1))
                        (reset-sensors agent))
                    (setf (right-bump agent) t)))
            (S  (if (> (- (agent-x agent) 1) -1)
                    (progn (setf (agent-x agent) (- (agent-x agent) 1))
                        (reset-sensors agent))
                    (setf (right-bump agent) t)))
            (W  (if (< (+ (agent-y agent) 1) 25)
                    (progn (setf (agent-y agent) (+ (agent-y agent) 1))
                        (reset-sensors agent))
                    (setf (right-bump agent) t)))))

	(B (case (facing *world*)
            (N  (if (> (- (agent-y agent) 1) -1)
                    (progn (setf (agent-y agent) (- (agent-y agent) 1))
                        (reset-sensors agent))
                    (setf (back-bump agent) t)))
            (E  (if (> (- (agent-x agent) 1) -1)
                    (progn (setf (agent-x agent) (- (agent-x agent) 1))
                        (reset-sensors agent))
                    (setf (back-bump agent) t)))
            (S  (if (< (+ (agent-y agent) 1) 25)
                    (progn (setf (agent-y agent) (+ (agent-y agent) 1))
                        (reset-sensors agent))
                    (setf (back-bump agent) t)))
            (W  (if (< (+ (agent-x agent) 1) 25)
                    (progn (setf (agent-x agent) (+ (agent-x agent) 1))
                        (reset-sensors agent))
                    (setf (back-bump agent) t)))))
	    (t nil)))


;;;
;;; Function: check-corner
;;; Arguments: 
;;;     - agent: the agent running in the search
;;; Returns: true if the agent is in a corner, otherwise nil
;;; Description: 
;;;    This function takes in an agent and attempts to move 
;;;    all the way left until it hits a wall, or right if that 
;;;    fails, to find a corner, and returns true if it succeeds.
;;; Author: Elizabeth Thompson
;;; Created 3/5/2019
;;; Modications: none
(defmethod check-corner (agent)
  (case (facing *world*)
    (N (loop while (not (left-bump agent))
          do (move agent 'L)) ;;fix up parenthsis
       (if (not (and (left-bump agent) (front-bump agent)))
           (loop while (not (right-bump agent)) 
              do (move agent 'R))))
    (S (loop while (not (left-bump agent)) 
          do (move agent 'L))
       (if (not (and (left-bump agent) (front-bump agent)))
           (loop while (not (right-bump agent)) 
              do (move agent 'R))))
    (E (loop while (not (left-bump agent)) 
          do (move agent 'L))
       (if (not (and (left-bump agent)(front-bump agent)))
	   (loop while (not (right-bump agent)) 
              do (move agent 'R))))
    (W (loop while (not (left-bump agent)) 
          do (move agent 'L))
       (if (not (and (left-bump agent) (front-bump agent)))          
	   (loop while (not (right-bump agent)) 
              do (move agent 'R)))))                      
  (cond ((and (left-bump agent) (front-bump agent)) 
                (setf (corner-found agent) t))
	((and (left-bump agent) (back-bump agent)) 
                (setf (corner-found agent) t))
	((and (right-bump agent) (front-bump agent)) 
                (setf (corner-found agent) t))
	((and (right-bump agent) (back-bump agent)) 
                (setf (corner-found agent) t))
        (t nil)))

;;;
;;; Function: reset-sensors
;;; Arguments: none
;;; Returns: nil
;;; Description: 
;;;    This function resets all of the sensors on an agent.
;;; Author: Elizabeth Thompson
;;; Created 3/4/2019
;;; Modications: none
(defmethod reset-sensors (agent)
    (setf (front-bump agent) nil)
    (setf (left-bump agent) nil)
    (setf (right-bump agent) nil)
    (setf (back-bump agent) nil))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; this is 80 characters to measure
;;;
;;; Function: rules
;;; Arguments:
;;;     - type: matches type of heurstic based on type of agent
;;; Returns: nil
;;; Description: 
;;;    This function gives the simulator the correct heuristic for the type
;;;    of agent being used.
;;; Author: Elizabeth Thompson
;;; Created 3/5/2019
;;; Modications: none
(defun rules (agent)
  (case agent
    ((reflex-agent) (reflex-rules agent)) ;no real heuristic for this search
    ((model-agent) (model-rules agent))
    ((hill-climbing-agent) (hill-climbing-rules agent))
    ((breadth-first-agent) (breadth-first-rules agent *world*))))
;        ((A*-agent) (A*-rules))
	


;;;
;;; Function: reflex-rules
;;; Arguments:
;;;     -agent: the agent in the search.
;;; Returns: nil
;;; Description: 
;;;     This is the set of rules that the agent-program for the reflex agent
;;;     functions like.
;;; Author: Elizabeth Thompson
;;; Created 3/5/2019
;;; Modications: none
(defun reflex-rules (agent)
  (loop while (not (corner-found agent))
     do (can-move agent)
       (check-corner agent)))


;;;
;;; Function: model-rules
;;; Arguments:
;;;     -agent: the agent in the search.
;;; Returns: nil
;;; Description: 
;;;     This is the set of rules that the agent-program for the model-reflex 
;;;     agent functions like. This agent can backtrack by tracking the 
;;;     locations it has been at.
;;; Author: Elizabeth Thompson
;;; Created 3/5/2019
;;; Modications: none
(defun model-rules (agent)
  (loop while (not (corner-found agent))
     do (can-move agent)
       (append (mapped agent) (cons (agent-x agent) (agent-y agent)))
       (check-corner agent))) 
;;;
;;; Function: distance
;;; Arguments:
;;;     -agent: the agent in the search.
;;; Returns: the distance of the agent from the goal
;;; Description: 
;;;     Calculates the xy distance from the agent to the goal
;;; Author: Elizabeth Thompson
;;; Created 3/5/2019
;;; Modications: none
(defun distance (agent)
  (let (x y v w)
        (setf x (car (goal-coords agent)))
        (setf y (cdr (goal-coords agent)))
        (setf v (abs (- (agent-x agent) x)))
        (setf w (abs (- (agent-y agent) y)))
        (+ w v)))

;;;
;;; Function: distance-static
;;; Arguments:
;;;     -agent: the agent in the search.
;;;     -coords: a coordinate pair location in the search
;;; Returns: the distance of the coordinates from the goal
;;; Description: 
;;;     Calculates the xy distance from the coordinate pair to the goal
;;; Author: Elizabeth Thompson
;;; Created 3/8/2019
;;; Modications: none
(defun distance-static (coords agent)
  (let (p s x y v w)
        (setf x (car (goal-coords agent)))
        (setf y (cdr (goal-coords agent)))
	(setf p (car coords))
	(setf s (cdr coords))
	(setf v (abs (- p x)))
        (setf w (abs (- s y)))
        (+ w v)))

;;;
;;; Function: goal-check
;;; Arguments:
;;;     -agent: the agent in the search
;;;     -coords: the coordinates of a location in the search
;;; Returns: t if the coordinates match, else nil
;;; Description: 
;;;     checks if the goal is at the given coordinates
;;; Author: Elizabeth Thompson
;;; Created 3/5/2019
;;; Modications: none
(defun goal-check (coords agent)
  (if (and (eq (car coords) (car (goal-coords agent))) (eq (cdr
							    coords) (cdr (goal-coords agent))))
      (setf (goal-found agent) t)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; this is 80 characters to measure

;;;
;;; Function: hill-climbing-rules
;;; Arguments:
;;;     -agent: the agent in the search.
;;; Returns: nil
;;; Description: 
;;;     This is the set of rules that the agent-program for the 
;;;     hill-climbing-agent functions like. It calculates the distance from
;;;     itself to the goal and constantly moves towards the goal in the
;;;     shortest way it can see.
;;; Author: Elizabeth Thompson
;;; Created 3/5/2019
;;; Modications: none
(defun hill-climbing-rules (agent)
  (let (x y z q)
    (loop while (not (goal-found agent)) do
         (setf x (distance agent))
         (can-move-dir agent 'L)
         (setf y (distance agent))
         (move agent 'R) (can-move-dir agent 'R)
         (setf z (distance agent))
         (move agent 'L) (can-move-dir agent 'F)
         (setf q (distance agent))
         (move 'B)    ;should be in original position now
         (cond   ((< y x) (move 'L))
                 ((< z x) (move 'R))
                 ((< q x) (move 'F))
                 (t (move 'B)))
         (if (and (equal (agent-x agent) (car (goal-coords agent)))
		  (equal (agent-y agent) (cdr (goal-coords agent)))) 
             (setf (goal-found agent) t)
             (+ (moves-counter *simulator*) 1)))))

(defmethod breadth-first-rules (world agent)
  (let ((i (moves-counter *simulator*)) (j (search-list agent)))   
      (loop unless (or (eq j nil) (goal-found agent))
	 do (if (eq (distance-static j agent) i) 
		(goal-check (pop j) agent)
		(pop j)))))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; this is 80 characters to measure

;;;creates an search agent instance, can be altered for desired agent
;;;type
(defparameter *search-agent*
  (make-instance 'model-agent))

;;;creates a global variable to be used as a counter variable.
(defparameter *turn-number* 30)

;;;
;;; Function: search-reflex
;;; Returns: nil
;;; Description:
;;;     This search is designed for the two types of reflex agents 
;;;     Utilizes the global variables to match the simulation to the current
;;;     rule set/agent program such that it matches the agent type being used.
;;;     The method then calls the rule sets functional code to find a corner.
;;; Author: Elizabeth Thompson
;;; Created 3/5/2019
;;; Modications: none
(defmethod search-reflex ()
  (setf (rules-list *simulator*) (rules *search-agent*))
  (if (not (corner-found *search-agent*))
      (print "corner-found false")
      (print "corner-found true"))
  (loop repeat *turn-number*
     do (can-move *search-agent*)
     when (corner-found *search-agent*)
     return 'victory)
  (if (not (corner-found *search-agent*))
      (print "search failed")))


;;;
;;; Function: search-hill
;;; Returns: nil
;;; Description:
;;;     This search is designed for hill-climbing agent.
;;;     This method utilizes a series of calculation to determin which
;;;     move available to the agent will most immediately reduce his
;;;     distance to the goal.
;;; Author: Elizabeth Thompson
;;; Created 3/7/2019
;;; Modications: none
(defmethod search-hill ()
  (setf (rules-list *simulator*) (rules *hill-climbing*))
  (hill-climbing-rules *hill-climbing*))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; this is 80 characters to measure

;;;
;;; Function: search-breadth
;;; Returns: nil
;;; Description:
;;;     This search is designed for a breadth-first agent.
;;;     This method utilizes a list of xy nodes of the search-space
;;;     for the agent to search though. The agent will search all
;;;     nodes 1 distance away before searching nodes that are 2 moves
;;;     away, and so forth until either the goal is found or the agent has
;;;     searched the entire map.
;;; Author: Elizabeth Thompson
;;; Created 3/8/2019
;;; Modications: none
(defmethod search-breadth ()
  (setf (rules-list *simulator*) (rules *breadth-agent*))
  (let ((a (moves-counter *simulator*)) (b (moves-counter *simulator*)))
    (loop while (not (goal-found *breadth-agent*))
       do (loop for n from 0 to a
	     do (loop for m from 0 to b
		   do (if (eq (distance-static (cons n m) *breadth-agent*)
			      (moves-counter *simulator*))
			  (goal-check (cons n m) *breadth-agent*))))
	 (+ (moves-counter *simulator*) 1))))
