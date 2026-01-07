(define (problem IMV-PROBLEM-SEISMIC)
(:domain imv_seismic)

(:objects
    drone1 - rover
    curator - rover
    
    entrance tunnel hall_a hall_b cryo stasis pods_room - location
    
    art_alpha art_beta mars_core - artifact
    pod1 pod2 - pod
    
    n0 n1 n2 - count
)

(:init
    ;; --- TIMED INITIAL LITERALS (Sisma) ---
    (safe entrance) (safe tunnel) 
    (safe hall_a) (safe hall_b) 
    (safe cryo) (safe stasis) (safe pods_room)

    ;; Hall B diventa pericolosa tra T=20 e T=60
    (at 20 (not (safe hall_b)))
    (at 60 (safe hall_b))
    
    ;; Topologia
    (connected entrance tunnel) (connected tunnel entrance)
    (connected tunnel hall_a) (connected hall_a tunnel)
    (connected tunnel hall_b) (connected hall_b tunnel)
    (connected tunnel cryo) (connected cryo tunnel)
    (connected tunnel stasis) (connected stasis tunnel)
    (connected tunnel pods_room) (connected pods_room tunnel)
    
    ;; Posizioni
    (at drone1 entrance)
    (at curator entrance)
    
    (at art_alpha hall_a)
    (at mars_core cryo)
    (at art_beta hall_b) 
    
    (at pod1 pods_room)
    (at pod2 pods_room)
    
    ;; Capacità e Stato Iniziale Robot
    (load drone1 n0)
    (load curator n0)
    
    (next n0 n1)
    (next n1 n2)

    ;; --- FIX CRUCIALE: DEFINIRE TUTTI I LIVELLI DI CAPACITÀ ---
    (capacity-ok drone1 n1)  ;; Il drone può portare 1 oggetto
    
    (capacity-ok curator n1) ;; Il curatore può portare 1 oggetto...
    (capacity-ok curator n2) ;; ...e anche 2 oggetti!
    
    ;; Stato Pod
    (empty pod1)
    (empty pod2)
    
    ;; Proprietà Oggetti
    (transportable art_alpha)
    (transportable mars_core)
    (transportable art_beta) ;; FIX: Rendiamo art_beta prendibile
    (transportable pod1)
    (transportable pod2)
    
    ;; Necessità
    (needs-cooling art_alpha)
    (has-cooling curator)
)

(:goal (and
    (cooled art_alpha)
    (at art_beta stasis)
    (at art_alpha cryo)
    (at mars_core stasis)
    (at curator entrance)
))

(:metric minimize (total-time))
)