(define (problem imv-problem-2-final)
  (:domain imv-multi-agent)

  (:objects
    curator - robot
    drone1 - drone
    
    entrance tunnel hall_a hall_b cryo stasis pods_room - location
    
    art_alpha art_beta mars_core - artifact
    pod1 pod2 - pod
    
    n0 n1 n2 - count
  )

  (:init
    (connected entrance tunnel) (connected tunnel entrance)
    (connected tunnel hall_a)   (connected hall_a tunnel)
    (connected tunnel hall_b)   (connected hall_b tunnel)
    (connected tunnel cryo)     (connected cryo tunnel)
    (connected tunnel stasis)   (connected stasis tunnel)
    (connected tunnel pods_room)(connected pods_room tunnel)

    (at curator entrance) (load curator n0)
    (at drone1 entrance)  (load drone1 n0)

    (capacity-ok drone1 n0) (capacity-ok drone1 n1) (capacity-ok drone1 n2)
    (capacity-ok curator n0) (capacity-ok curator n1) (capacity-ok curator n2)
    
    (next n0 n1) (next n1 n2)

    (at art_alpha hall_a)
    (at art_beta hall_b)
    (at mars_core cryo)
    (at pod1 pods_room) (empty pod1)
    (at pod2 pods_room) (empty pod2)

    (transportable pod1)
    (transportable pod2)
    (transportable art_alpha)
    (transportable mars_core)

    (needs-cooling art_alpha)

    (seismic-active hall_b) 
  )

  (:goal
    (and
      (at art_beta stasis)
      (at pod1 pods_room)
      (at pod2 pods_room)
      (at art_alpha cryo)
      (at mars_core stasis)
      (at curator entrance)
      (at drone1 entrance)
    )
  )
)