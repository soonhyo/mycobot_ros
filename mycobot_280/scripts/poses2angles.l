(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus "testnode")
(setq poselist (one-shot-subscribe "/trajectory_for_comb_node/output/pose_array" geometry_msgs::PoseArray :timeout (* 10 1000)))

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

(pose2coords (car poselist) (send poselist :)
	     
