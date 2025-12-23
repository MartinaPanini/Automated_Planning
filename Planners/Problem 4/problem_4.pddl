(define (problem imv-problem-4-temporal)
  (:domain imv-temporal)

  (:objects
    curator - robot
    drone1 - drone
    entrance tunnel hall_a hall_b cryo stasis pods_room - location
    art_alpha art_beta mars_core - artifact
    pod1 pod2 - pod
    n0 n1 n2 - count ;; n0 è un oggetto normale ora
  )

  (:init
    (connected entrance tunnel) (connected tunnel entrance)
    (connected tunnel hall_a)   (connected hall_a tunnel)
    (connected tunnel hall_b)   (connected hall_b tunnel)
    (connected tunnel cryo)     (connected cryo tunnel)
    (connected tunnel stasis)   (connected stasis tunnel)
    (connected tunnel pods_room)(connected pods_room tunnel)

    ;; Stato Agenti (INIZIALMENTE POSSONO MUOVERSI)
    (at curator entrance) (load curator n0) (can-move curator)
    (at drone1 entrance)  (load drone1 n0)  (can-move drone1)

    (capacity-ok drone1 n0) (capacity-ok drone1 n1) (capacity-ok drone1 n2)
    (capacity-ok curator n0) (capacity-ok curator n1) (capacity-ok curator n2)
    
    (next n0 n1) (next n1 n2)

    ;; Oggetti
    (at art_alpha hall_a)
    (at art_beta hall_b)
    (at mars_core cryo)
    (at pod1 pods_room) (empty pod1)
    (at pod2 pods_room) (empty pod2)

    ;; DEFINIZIONE TIPI DI OGGETTI (Positivi)
    (transportable pod1)
    (transportable pod2)
    (transportable art_alpha)
    (transportable mars_core)
    
    (fragile art_beta) ;; Art Beta è fragile!

    ;; LUOGHI SICURI (Tutti tranne se volessimo simulare sisma)
    (safe-location entrance)
    (safe-location tunnel)
    (safe-location hall_a)
    (safe-location hall_b)
    (safe-location cryo)
    (safe-location stasis)
    (safe-location pods_room)
  )

  (:goal
    (and
      (at art_beta stasis)
      (at pod1 pods_room)
      (at art_alpha cryo)
      (at mars_core stasis)
    )
  )

  (:metric minimize (total-time))
)