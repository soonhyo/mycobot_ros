#!/usr/bin/env roseus
(load "package://mycobot_280/euslisp/mycobot-interface.l")
(load "models/arrow-object.l")

(setq *arrow-mycobot-rarm* (arrow))
(setq *arrow-mycobot-end* (arrow))
(setq *arrow-mycobot-target* (arrow))

(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "std_msgs")

(mycobot-init)

(setq *count* 0)
;; start grasp comb
(setq *depth* 40)

;; (send *ri* :start-grasp :rarm)
;; (unix::sleep 3)
(setq *default-end-z* 20)
(setq *gripper-x* (- 80 *default-end-z*)
      )
(setq *gripper-z* -10)
(setq *gripper-y* 0)

(setq *comb-gripper* -10)

(setq *comb-head-length* 80)
(setq *comb-head-width* 70)
(setq *comb-head-thickness* 25)

(setq *comb-head-x* 0)
(setq *comb-head-y* 0)
(setq *comb-head-z* (+ (* -1 (/ *comb-head-thickness* 2))
			  *comb-gripper*)
      )

(setq *comb-attach-thickness* 15)

(setq *comb-attach-x* 0)
(setq *comb-attach-y* 0)
(setq *comb-attach-z* (+ (* -1 (/ *comb-attach-thickness* 2))
			 (* -1 (/ *comb-head-thickness* 2))
			 )
      )

(setq *endeffect-x* (+ *gripper-x* *comb-head-x* *comb-attach-x*))
(setq *endeffect-y* (+ *gripper-y* *comb-head-y* *comb-attach-y*))
(setq *endeffect-z* (+ *gripper-z* *comb-head-z* *comb-attach-z*))
(setq *endeffect* (make-cube 10 10 10))
(send *endeffect* :set-color :yellow)
(send *endeffect* :newcoords (send (send *robot* :rarm :end-coords) :copy-worldcoords))
(send *endeffect* :locate (float-vector *endeffect-x* *endeffect-y* *endeffect-z*))
(send *endeffect* :rotate pi :y)
(send *endeffect* :rotate (/ -pi 2) :z)
 
(setq *gripper* (make-cube 20 20 20))
(send *gripper* :set-color :yellow)
(send *gripper* :newcoords (send (send *robot* :rarm :end-coords) :copy-worldcoords))
(send *gripper* :locate (float-vector *gripper-x* *gripper-y* *gripper-z*))
(send *robot* :rarm :end-coords :assoc *gripper*)
;; (* 1 (+ (/ *comb-head-length* 2) *comb-gripper-length*))

(setq *comb-head* (make-cube *comb-head-length* *comb-head-width* *comb-head-thickness*))
(send *comb-head* :set-color :blue)
(send *comb-head* :newcoords (send *gripper* :copy-worldcoords))
(send *comb-head* :locate (float-vector *comb-head-x* *comb-head-y* *comb-head-z*) :local)

(setq *comb-attach* (make-cube *comb-head-length* *comb-head-width* *comb-attach-thickness*))
(send *comb-attach* :set-color :red)
(send *comb-attach* :newcoords (send *comb-head* :copy-worldcoords))
(send *comb-attach* :locate (float-vector *comb-attach-x* *comb-attach-y* *comb-attach-z*) :local)

(setq *offset-x* (* -1 *endeffect-x* ))
(setq *offset-y* (* -1 *endeffect-y* ))
(setq *offset-z* (* -1 *endeffect-z* ))

(send *arrow-mycobot-rarm* :newcoords (send (send *robot* :rarm :end-coords) :copy-worldcoords))
(send *robot* :rarm :end-coords :assoc *arrow-mycobot-rarm*)

(send *arrow-mycobot-end* :newcoords (send *endeffect* :copy-worldcoords))

(objects (list *robot* *arrow-mycobot-rarm* *gripper* *comb-attach* *comb-head* *endeffect* *arrow-mycobot-end* *arrow-mycobot-target*))

(defun pose2coords (pose frame-id)
  (let
      (coordlist angles coords base->camera (stamp (ros::time 0)))
    (unless (boundp '*tfl*)
      (setq *tfl* (instance ros::transform-listener :init)))
    
    (send *tfl* :wait-for-transform
          "base" frame-id stamp 10.0)

    (setq base->camera
          (send *tfl* :lookup-transform
                "base" frame-id stamp))
    
    (setq coords
          (send (send base->camera :copy-worldcoords)
                :transform (ros::tf-pose->coords pose)))
    
    (send coords :translate (float-vector -70 0 -80) :local)
    

    ;; inverse-kinematics
    (send *baxter* :larm :inverse-kinematics coords
          :rotation-axis t)
    (setq angles (send *baxter* :larm :angle-vector))

    (setq coordlist (append coordlist (list angles))) 
    
    )
  )

(defun comb-from-pose (pose frame-id)
  (let
      (offset length-to-endeffect comb-path-coords camera->base (stamp (ros::time 0)) )
    (print (send pose :position :x))
    
    (unless (boundp '*tfl*)
      (setq *tfl* (instance ros::transform-listener :init)))

    (send *tfl* :wait-for-transform
          "link1" frame-id stamp 10.0)

    (setq camera->base
          (send *tfl* :lookup-transform
                "link1" frame-id stamp))
    
    (setq comb-path-coords
          (send (send camera->base :copy-worldcoords)
                :transform (ros::tf-pose->coords pose)))

    ;; (send comb-path-coords :rotate (/ pi 2) :z)
    ;; (send comb-path-coords :rotate -pi :y)
    ;; (send comb-path-coords :translate (float-vector *offset-x* *offset-y* *offset-z*) :local)
    (setq comb-path-coords (ros::tf-pose->coords pose))
    
    (send *arrow-mycobot-target* :newcoords (send comb-path-coords :copy-worldcoords))

    (objects (list *robot* *arrow-mycobot-rarm* *gripper* *comb-attach* *comb-head* *endeffect* *arrow-mycobot-end* *arrow-mycobot-target*))

    ;; inverse-kinematics
    (send *robot* :rarm :inverse-kinematics comb-path-coords
          :rotation-axis nil)
    (send *irtviewer* :draw-objects)
    ;; send to real robot                       
    
    (send *ri* :angle-vector (send *robot* :angle-vector) 500 )
    (send *ri* :wait-interpolation)
    

    (send comb-path-coords :draw-on :flush t)
    (send *irtviewer* :draw-objects)
    )
  )
;; get pose array from topic
(defun pose-array-cb (pose-array-msg)
  (print pose-array-msg)
  (let*
      ((poses (send pose-array-msg :poses)))
    ;; (pose (if poses (car poses))))
    (if poses
        (progn
          (print "pose")
          (dolist (pose poses)
            (comb-from-pose pose (send pose-array-msg :header :frame_id))
            )
          )
        )
    )
  )
;; bbox-msg callback function
(defun comb-cb (bbox-msg)
  (let*
      ((boxes (send bbox-msg :boxes))
       (box (if boxes (car boxes))))
    (if box
        (comb-from-box box)

      )
    )
  )


(ros::roseus "combloop")
(ros::advertise "signal_loop" std_msgs::int8 1)

(do-until-key
  (setq path-once (one-shot-subscribe "/trajectory_for_comb/output/pose_array"
                                      geometry_msgs::PoseArray
                                      :timeout (* 10 1000)))
  (pose-array-cb path-once)
)


