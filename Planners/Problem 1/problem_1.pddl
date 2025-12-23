(define (problem imv-problem-1)
  (:domain imv-simple)

  (:objects
    curator - robot
    entrance tunnel hall_a hall_b cryo stasis pods_room - location
    art_alpha art_beta mars_core - item
    pod1 pod2 - item
  )

  (:init
    ;; --- TOPOLOGIA ---
    (connected entrance tunnel) (connected tunnel entrance)
    (connected tunnel hall_a)   (connected hall_a tunnel)
    (connected tunnel hall_b)   (connected hall_b tunnel)
    (connected tunnel cryo)     (connected cryo tunnel)
    (connected tunnel stasis)   (connected stasis tunnel)
    (connected tunnel pods_room)(connected pods_room tunnel)

    ;; --- STATO INIZIALE AGENTE ---
    (at curator entrance)
    (handempty curator)

    ;; --- POSIZIONAMENTO OGGETTI ---
    (at art_alpha hall_a) (is-artifact art_alpha)
    (at art_beta hall_b)  (is-artifact art_beta)
    (at mars_core cryo)   (is-artifact mars_core)

    (at pod1 pods_room) (is-pod pod1) (empty pod1)
    (at pod2 pods_room) (is-pod pod2) (empty pod2)

    ;; --- REGOLE DI TRASPORTABILITÀ ---
    (transportable pod1)
    (transportable pod2)
    (transportable art_alpha)
    (transportable mars_core)
    ;; NOTA: art_beta NON è transportable (simula la fragilità)
    ;; Questo significa che il robot non può uscire dalla stanza con art_beta in mano.

    ;; --- STATO SISMICO ---
    ;; (Nessun terremoto attivo al momento)
  )

  (:goal
    (and
      ;; 1. ARTEFATTO BETA: Deve essere "a terra" allo Stasis Lab
      ;; Questo implica implicitamente: Load -> Transport -> Unload -> Drop
      (at art_beta stasis)

      ;; 2. POD: Deve essere riportato alla base
      (at pod1 pods_room)
      (at pod2 pods_room)

      ;; 3. ALTRI OBIETTIVI (Standard)
      (at art_alpha cryo)
      (at mars_core stasis)
    )
  )
)