;; Auto-generated. Do not edit!


(when (boundp 'detection::obstacle_bound)
  (if (not (find-package "DETECTION"))
    (make-package "DETECTION"))
  (shadow 'obstacle_bound (find-package "DETECTION")))
(unless (find-package "DETECTION::OBSTACLE_BOUND")
  (make-package "DETECTION::OBSTACLE_BOUND"))

(in-package "ROS")
;;//! \htmlinclude obstacle_bound.msg.html


(defclass detection::obstacle_bound
  :super ros::object
  :slots (_top_left_x _top_left_y _bottom_right_x _bottom_right_y _distance ))

(defmethod detection::obstacle_bound
  (:init
   (&key
    ((:top_left_x __top_left_x) 0)
    ((:top_left_y __top_left_y) 0)
    ((:bottom_right_x __bottom_right_x) 0)
    ((:bottom_right_y __bottom_right_y) 0)
    ((:distance __distance) 0.0)
    )
   (send-super :init)
   (setq _top_left_x (round __top_left_x))
   (setq _top_left_y (round __top_left_y))
   (setq _bottom_right_x (round __bottom_right_x))
   (setq _bottom_right_y (round __bottom_right_y))
   (setq _distance (float __distance))
   self)
  (:top_left_x
   (&optional __top_left_x)
   (if __top_left_x (setq _top_left_x __top_left_x)) _top_left_x)
  (:top_left_y
   (&optional __top_left_y)
   (if __top_left_y (setq _top_left_y __top_left_y)) _top_left_y)
  (:bottom_right_x
   (&optional __bottom_right_x)
   (if __bottom_right_x (setq _bottom_right_x __bottom_right_x)) _bottom_right_x)
  (:bottom_right_y
   (&optional __bottom_right_y)
   (if __bottom_right_y (setq _bottom_right_y __bottom_right_y)) _bottom_right_y)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; int16 _top_left_x
    2
    ;; int16 _top_left_y
    2
    ;; int16 _bottom_right_x
    2
    ;; int16 _bottom_right_y
    2
    ;; float32 _distance
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _top_left_x
       (write-word _top_left_x s)
     ;; int16 _top_left_y
       (write-word _top_left_y s)
     ;; int16 _bottom_right_x
       (write-word _bottom_right_x s)
     ;; int16 _bottom_right_y
       (write-word _bottom_right_y s)
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _top_left_x
     (setq _top_left_x (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _top_left_y
     (setq _top_left_y (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _bottom_right_x
     (setq _bottom_right_x (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _bottom_right_y
     (setq _bottom_right_y (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get detection::obstacle_bound :md5sum-) "41b15b11d68bbfaa7472618894c283f6")
(setf (get detection::obstacle_bound :datatype-) "detection/obstacle_bound")
(setf (get detection::obstacle_bound :definition-)
      "int16 top_left_x
int16 top_left_y
int16 bottom_right_x
int16 bottom_right_y
float32 distance

")



(provide :detection/obstacle_bound "41b15b11d68bbfaa7472618894c283f6")


