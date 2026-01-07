(define (problem imv-problem-1)
  (:domain imv-simple)

  (:objects
    curator - robot
    entrance tunnel hall_a hall_b cryo stasis pods_room - location
    art_alpha art_beta mars_core - item
    pod1 pod2 - item
  )

  (:init
    (connected entrance tunnel) (connected tunnel entrance)
    (connected tunnel hall_a)   (connected hall_a tunnel)
    (connected tunnel hall_b)   (connected hall_b tunnel)
    (connected tunnel cryo)     (connected cryo tunnel)
    (connected tunnel stasis)   (connected stasis tunnel)
    (connected tunnel pods_room)(connected pods_room tunnel)

    ;; Stato Agente
    (at curator entrance)
    (handempty curator)

    ;; Oggetti
    (at art_alpha hall_a) (is-artifact art_alpha)
    (at art_beta hall_b)  (is-artifact art_beta)
    (at mars_core cryo)   (is-artifact mars_core)

    (at pod1 pods_room) (is-pod pod1) (empty pod1)
    (at pod2 pods_room) (is-pod pod2) (empty pod2)

    ;; Regole
    (transportable pod1)
    (transportable pod2)
    (transportable art_alpha)
    (transportable mars_core)
    ;; art_beta NON è transportable (fragile)
    
    ;; Temperatura
    (needs-cooling art_alpha)
  )

  (:goal (and
      (at art_alpha cryo)
      (at art_beta stasis)
      (at mars_core stasis)
      
      ;; Ritorno alla base (come richiesto dal prof)
      (at curator entrance)
  ))
)