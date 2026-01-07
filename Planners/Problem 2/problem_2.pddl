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
    ;; Topologia
    (connected entrance tunnel) (connected tunnel entrance)
    (connected tunnel hall_a)   (connected hall_a tunnel)
    (connected tunnel hall_b)   (connected hall_b tunnel)
    (connected tunnel cryo)     (connected cryo tunnel)
    (connected tunnel stasis)   (connected stasis tunnel)
    (connected tunnel pods_room)(connected pods_room tunnel)

    ;; Stato Agenti
    (at curator entrance) (load curator n0)
    (at drone1 entrance)  (load drone1 n0)

    ;; Capacità: Il Drone e il Curator possono portare fino a 2 oggetti
    (capacity-ok drone1 n0) (capacity-ok drone1 n1) (capacity-ok drone1 n2)
    (capacity-ok curator n0) (capacity-ok curator n1) (capacity-ok curator n2)
    
    ;; Numeri
    (next n0 n1) (next n1 n2)

    ;; Oggetti e posizioni iniziali
    (at art_alpha hall_a)
    (at art_beta hall_b)
    (at mars_core cryo)
    (at pod1 pods_room) (empty pod1)
    (at pod2 pods_room) (empty pod2)

    ;; --- REGOLE DI TRASPORTABILITÀ ---
    (transportable pod1)
    (transportable pod2)
    (transportable art_alpha)
    (transportable mars_core)
    ;; NOTA: art_beta NON è presente qui -> fragile -> causa (blocked).

    ;; --- TEMPERATURA ---
    ;; Fix Critico: Ora art_alpha richiede raffreddamento prima del trasporto
    (needs-cooling art_alpha)

    ;; --- SISMICITÀ ---
    ;; (seismic-active hall_b) 
  )

  (:goal
    (and
      ;; 1. Art Beta portato allo Stasis Lab
      (at art_beta stasis)
      
      ;; 2. I Pod devono tornare al loro posto
      (at pod1 pods_room)
      (at pod2 pods_room)

      ;; 3. Art Alpha alla Cryo
      (at art_alpha cryo)

      ;; 4. Core Sample allo Stasis
      (at mars_core stasis)

      ;; 5. Ritorno alla base
      (at curator entrance)
      (at drone1 entrance)
    )
  )
)