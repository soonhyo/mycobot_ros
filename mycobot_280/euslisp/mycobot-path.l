#!/usr/bin/env roseus
(load "package://mycobot_280/euslisp/mycobot-interface.l")
(load "models/arrow-object.l")

(setq *arrow-mycobot-rarm* (arrow))
(setq *arrow-mycobot-end* (arrow))
(setq *arrow-mycobot-target* (arrow))

(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "std_msgs")

(setq *signal-loop* (instance std_msgs::int8 :init))
(send *signal-loop* :data 1)

(mycobot-init)

(setq *seq-before* 0)
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
;; (send *robot* :rarm :end-coords :assoc *gripper*)
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
(send *robot* :rarm :end-coords :assoc *comb-head*)
(send *robot* :rarm :end-coords :assoc *comb-attach*)

(send *arrow-mycobot-end* :newcoords (send *endeffect* :copy-worldcoords))

(objects (list *robot* *arrow-mycobot-rarm* *gripper* *comb-attach* *comb-head* *endeffect* *arrow-mycobot-end* *arrow-mycobot-target*))

(setq *anglelist* 0)
(defun reset-pose ()
  (send *robot* :reset-pose)
  ;; move real robot
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )
(defun ready-static ()
  ;; inverse-kinematics
  (send *robot* :rarm :angle-vector #f(-31.81 -129.99 153.36 -22.58 -123.13 150.02))
  ;;#f(34.1895 -29.5752 -56.228 109.138 -95.8228 39.3311 75.3882)
  ;; #f(22.8296 -26.7187 -44.978 123.574 -132.495 70.9058 96.2622)
  (send *irtviewer* :draw-objects)
  ;; send to real robot
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun poses2angles (poses frame-id)
  (let
      (count anglelist angle ret coords base->camera (stamp (ros::time 0)))
    (unless (boundp '*tfl*)
      (setq *tfl* (instance ros::transform-listener :init)))
    
    (send *tfl* :wait-for-transform "link1" frame-id stamp 10.0)
    
    (setq base->camera
          (send *tfl* :lookup-transform
                "link1" frame-id stamp))
    (setq count 0)
    (dolist (pose poses)
      (setq coords
            (send (send base->camera :copy-worldcoords)
                  :transform (ros::tf-pose->coords pose)))
      ;; inverse-kinematics
      (setq ret (send *robot* :rarm :inverse-kinematics coords
            :rotation-axis nil ))
      (if ret
	  (progn
	    (setq angle (send *robot* :rarm :angle-vector))
      
	    (if (eq count 0)
		(progn
		  (setq anglelist (list angle))
		  (setq count (+ count 1))
		  )
		(setq anglelist (append anglelist (list angle)))
		)
	    )
	  )
      )
    (setq *anglelist* anglelist)
    )
  )

(defun poses->coords (poses frame-id)
  (let (coordslist coords base->camera (stamp (ros::time 0)) )
    (if poses
        (progn
          (setq count 0)
          ((dolist (pose poses)
             (unless (boundp '*tfl*)
               (setq *tfl* (instance ros::transform-listener :init)))
             
             (send *tfl* :wait-for-transform
                   "link1" frame-id stamp 10.0)
             
             (setq base->camera
                   (send *tfl* :lookup-transform
                         "link1" frame-id stamp))
  
             (setq coords
                   (send (send base->camera :copy-worldcoords)
                         :transform (ros::tf-pose->coords pose)))
             
             (if (eq count 0)
                 (setq coordslist (list coords))
                 (setq coordslist (append coordslist coords))
                 )
             (setq (+ count 1))
             
             )
           )
          )
        )
    (print coordslist)
    coordslist
    )
  )
;; get pose array from topic
(defun pose->angle-array-cb (pose-array-msg)
  (print pose-array-msg)
  (unless pose-array-msg
    (progn
      (print "stand-by")
      (ready-static)
      )
    )
  (let* (poses angleslist)
    (if pose-array-msg
        (setq poses (send pose-array-msg :poses)))
    ;; (pose (if poses (car poses))))
    (if poses
        (progn
          (print "approach")
          ;; (approach (car poses) (send pose-array-msg :header :frame_id))

          (print "comb")
          (setq angleslist (coords->angles (poses->coords poses (send pose-array-msg :header :frame_id))))
          (print angleslist)

          (print "stand-by")
          (ready-static)

          ;; (ready (nth (- (list-length poses) 1) poses) (send pose-array-msg :header :frame_id))
          )
        )
    )
  )
(defun coords->angles (coordslist)
  (let (angleslist angle count)
    (if coordslist
        (progn
          (setq count 0)
          ((dolist (coords coordslist)
             (send *robot* :rarm :inverse-kinematics coords)
             (setq angle (send *robot* :rarm :angle-vector))
             (if (eq count 0)
                 (setq angleslist (list angle))
                 (setq angleslist (append angleslist angle))
                 )
             (setq (+ count 1))
             )
           )
          )
        )
    (print angleslist)
    angleslist
   )

  )
(defun get-header-of-mask(msg)
  (print "rec")
  (unless msg
    (ready-static)
    (print "no msg")
    (ros::publish "signal_loop" *signal-loop*)
    )

  (let (seq hdr)
    (if msg
        (progn
          (setq hdr (send msg :header))
          (setq seq (send hdr :seq))
          (ros::ros-info "sequence ~A" seq)
          (if (< *seq-before* seq)
              (progn
                (setq path-once (one-shot-subscribe "/trajectory_for_comb_node/output/pose_array"
                                                    geometry_msgs::PoseArray
                                                    :timeout (* 10 1000)))
                (pose-array-cb path-once)
                ;; (pose->angle-array-cb path-once)

                ;; (comb-cb (one-shot-subscribe "/segmentation_decomposer_hair/boxes"
                ;;                              jsk_recognition_msgs::BoundingBoxArray
                ;;                              :timeout (* 10 1000)))
                (setq *seq-before* seq)
                (unix::sleep 1)

                )
              (progn
                (ready-static)
                (exit)
              )

            )
          )
      )
    )
  )

;; get pose array from topic
(defun pose-array-cb (pose-array-msg)
  (print pose-array-msg)
  (unless pose-array-msg
    (progn
      (print "stand-by")
      (ready-static)
      )
    )
  (let* (poses)
    (if pose-array-msg
        (setq poses (send pose-array-msg :poses)))
    ;; (pose (if poses (car poses))))
    (if poses
        (progn
          (print "approach")
          ;; (approach (car poses) (send pose-array-msg :header :frame_id))

          (print "comb")
          ;; (dolist (pose poses) (comb-from-pose pose (send pose-array-msg :header :frame_id)))
          (comb-st-from-pose (car poses) (send pose-array-msg :header :frame_id))


          (print "stand-by")
          (ready-static)

          ;; (ready (nth (- (list-length poses) 1) poses) (send pose-array-msg :header :frame_id))
          )
        )
    )
  )
(defun comb-from-pose (pose frame-id)
  (let
      (offset length-to-endeffect comb-path-coords camera->base (stamp (ros::time 0)) )
    ;; (print (send pose :position :x))
    
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
    ;; (setq comb-path-coords (ros::tf-pose->coords pose))
    
    (send *arrow-mycobot-target* :newcoords (send comb-path-coords :copy-worldcoords))

    (objects (list *robot* *arrow-mycobot-rarm* *gripper* *comb-attach* *comb-head* *endeffect* *arrow-mycobot-end* *arrow-mycobot-target*))

    ;; inverse-kinematics
    (send *robot* :rarm :inverse-kinematics comb-path-coords)
          :rotation-axis nil :translation-axis nil)
    (send *irtviewer* :draw-objects)
    ;; send to real robot                       
    
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    

    (send comb-path-coords :draw-on :flush t)
    (send *irtviewer* :draw-objects)
    )
  )

(defun comb-st-from-pose (pose frame-id)
  (let
      (target-cube comb-target-cube target-arrow offset length-to-endeffect target-coords comb-coords base->camera (stamp (ros::time 0)) )
    (print (send pose :position :x))
    
    (unless (boundp '*tfl*)
      (setq *tfl* (instance ros::transform-listener :init)))

    (send *tfl* :wait-for-transform
          "link1" frame-id stamp 10.0)

    (setq base->camera
          (send *tfl* :lookup-transform
                "link1" frame-id stamp))
    
    (setq target-coords
          (send (send base->camera :copy-worldcoords)
                :transform (ros::tf-pose->coords pose)))
    
    ;; for viz of coords
    (setq target-cube (make-cube 10 10 10))
    (send target-cube :newcoords (send target-coords :copy-worldcoords))
    

    (setq comb-coords (send target-coords :copy-worldcoords))
    (send comb-coords :rotate (/ -pi 2) :x)
    ;; (send comb-coords :rotate (/ -pi 2) :y)
    ;; (send comb-coords :rotate (/ -pi 4) :y)

    ;; (send comb-coords :translate (float-vector *offset-x* *offset-y* *offset-z*) :local)

    (send comb-coords :translate (float-vector -50 0 0) :local)
    ;; (send comb-coords :translate (float-vector 0 0 0) :local)

    (setq target-arrow (arrow))
    (send target-arrow :newcoords (send comb-coords :copy-worldcoords))

    ;; (send comb-coords :translate (float-vector 0 0 30) :local)
    ;; (send comb-coords :translate (float-vector 0 0 20) :world)

    ;; (send comb-coords :translate (float-vector *depth* 0 0) :local)

    (setq comb-target-cube (make-cube 10 10 10))
    (send comb-target-cube :set-color :yellow)
    (send comb-target-cube :newcoords (send comb-coords :copy-worldcoords))

    ;; (objects (list *baxter* *table* *comb* *arrow-baxter-larm* target-coords target-arrow target-cube comb-target-cube))
;; target-arrow comb-target-cube comb-coords
    (objects (list *robot* *arrow-mycobot-rarm* *comb-attach* *comb-head* comb-target-cube target-arrow comb-coords))
    ;; *arrow-mycobot-target*
    ;; *arrow-mycobot-end*
    ;; inverse-kinematics
    (send *robot* :rarm :inverse-kinematics comb-coords
          :rotation-axis nil)
    (send *irtviewer* :draw-objects)
    ;; send to real robot                       
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    
    ;; (print "touch")
    ;; (touch)
  
    ;; move down
    (print "down")
    (setq down-coords (send comb-coords :copy-worldcoords))
    ;; (send *baxter* :larm :move-end-pos (float-vector 0 *dst-length* 0) :local))
    
    (send down-coords :translate #f(0 0 -150) :world)
    
    (send *robot* :rarm :inverse-kinematics down-coords :rotation-axis nil)
    (send *irtviewer* :draw-objects)

    ;; send to real robot                       
    
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    )
  )

;; get pose array from topic
;; (defun pose-array-cb (pose-array-msg)
;;   (print pose-array-msg)
;;   (let*
;;       ((poses (send pose-array-msg :poses)))
;;     ;; (pose (if poses (car poses))))
;;     (if poses
;;         (progn
;;           (print "pose")
;;           (dolist (pose poses)
;;             (comb-from-pose pose (send pose-array-msg :header :frame_id))
;;             )
;;           )
;;         )
;;     )
;;   )
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

(ready-static)

(setq *before-y* 0 )
(setq *count* 0)

(ros::roseus "combloop")
(ros::advertise "signal_loop" std_msgs::int8 1)

;; (setq path-once (one-shot-subscribe "/trajectory_for_comb_node/output/pose_array"
;;                                     geometry_msgs::PoseArray
;;                                     :timeout (* 10 1000)))
;; (poses2angles (send path-once :poses) (send path-once :header :frame_id))
;; (print *anglelist*)

(do-until-key
  (print "loop")
  (ros::publish "signal_loop" *signal-loop*)
  (setq mask-once (one-shot-subscribe "/hair_orientation/output/mask"
                                      sensor_msgs::Image
                                      :timeout (* 10 5000)))
  (get-header-of-mask mask-once)

  ;; (setq path-once (one-shot-subscribe "/trajectory_for_comb/output/pose_array"
  ;;                                     geometry_msgs::PoseArray
  ;;                                     :timeout (* 10 1000)))
  ;; (pose-array-cb path-once)
)

;; (do-until-key
;;   (setq path-once (one-shot-subscribe "/trajectory_for_comb_node/output/pose_array"
;;                                       geometry_msgs::PoseArray
;;                                       :timeout (* 10 1000)))
;;   (pose-array-cb path-once)
;; )


