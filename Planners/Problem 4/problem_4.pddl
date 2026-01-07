(define (problem IMV-PROBLEM-4-TEMPORAL)
(:domain imv_temporal)

(:objects
    drone1 - rover
    curator - rover
    
    entrance tunnel hall_a hall_b cryo stasis pods_room - location
    
    art_alpha art_beta mars_core - artifact
    pod1 pod2 - pod
    
    n0 n1 n2 - count
)

(:init
    (connected entrance tunnel) (connected tunnel entrance)
    (connected tunnel hall_a) (connected hall_a tunnel)
    (connected tunnel hall_b) (connected hall_b tunnel)
    (connected tunnel cryo) (connected cryo tunnel)
    (connected tunnel stasis) (connected stasis tunnel)
    (connected tunnel pods_room) (connected pods_room tunnel)
    
    (at drone1 entrance)
    (at curator entrance)
    
    (at art_alpha hall_a)
    (at mars_core cryo)
    (at art_beta hall_b)
    
    (at pod1 pods_room)
    (at pod2 pods_room)
    
    (load drone1 n0)
    (load curator n0)
    
    (next n0 n1)
    (next n1 n2)
    
    (capacity-ok drone1 n0)
    (capacity-ok drone1 n1)
    (capacity-ok drone1 n2)
    (capacity-ok curator n0)
    (capacity-ok curator n1)
    
    (fragile art_beta)
    (needs-cooling art_alpha)
    
    (transportable mars_core)
    (transportable art_alpha)
    (transportable pod1)
    (transportable pod2)
    
    (empty pod1)
    (empty pod2)
    
    (ready drone1)
    (ready curator)
    (movement-allowed drone1)
    (movement-allowed curator)
    
    (has-cooling drone1)
)

(:goal (and
    (cooled art_alpha) ; CORRETTO: Goal Positivo
    (at art_beta stasis)
    (at art_alpha cryo)
    (at mars_core stasis)
))

(:metric minimize (total-time))
)