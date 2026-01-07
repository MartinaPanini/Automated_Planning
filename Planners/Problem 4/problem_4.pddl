(define (problem IMV-PROBLEM-4-TEMPORAL)
(:domain IMV-TEMPORAL)

(:objects
    drone1 - rover
    curator - rover
    
    entrance tunnel hall_a hall_b cryo stasis pods_room - location
    
    art_alpha art_beta mars_core - artifact
    pod1 pod2 - pod
    
    n0 n1 n2 - count
)

(:init
    ;; Topologia
    (connected entrance tunnel) (connected tunnel entrance)
    (connected tunnel hall_a) (connected hall_a tunnel)
    (connected tunnel hall_b) (connected hall_b tunnel)
    (connected tunnel cryo) (connected cryo tunnel)
    (connected tunnel stasis) (connected stasis tunnel)
    (connected tunnel pods_room) (connected pods_room tunnel)
    
    ;; Posizioni Iniziali
    (at drone1 entrance)
    (at curator entrance)
    
    ;; Oggetti
    (at art_alpha hall_a)
    (at mars_core cryo)
    
    (at pod1 pods_room)
    (at pod2 pods_room)
    (at art_beta pods_room)
    
    ;; Capacità
    (capacity drone1 n0)
    (capacity curator n0)
    (next n0 n1)
    (next n1 n2)
    
    ;; Proprietà
    (fragile art_beta)
)

(:goal (and
    (cooled art_alpha)
    (at art_beta stasis)
    (at art_alpha cryo)
    (at mars_core stasis)
))

(:metric minimize (total-time))
)