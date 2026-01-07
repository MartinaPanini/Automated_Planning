(define (domain imv_seismic)
  (:requirements :strips :typing :durative-actions :timed-initial-literals :negative-preconditions)

  ;; DEFINIZIONE GERARCHIA TIPI (Cruciale per evitare Type Error)
  (:types 
    location count - object
    rover artifact pod - object 
  )

  (:predicates
    (at ?x - object ?l - location)
    (connected ?l1 ?l2 - location)
    
    (holding ?r - rover ?x - object)
    
    ;; Gestione Capacità Robot
    (load ?r - rover ?c - count)
    (capacity-ok ?r - rover ?c - count)
    (next ?c1 ?c2 - count)
    
    ;; Gestione Pod
    (inside ?a - artifact ?p - pod)
    (empty ?p - pod) ;; 'empty' si applica SOLO ai pod
    
    ;; Proprietà Oggetti
    (transportable ?i - object)
    (fragile ?i - object)
    (has-cooling ?r - rover)
    (needs-cooling ?i - artifact)
    (cooled ?i - artifact)
    
    ;; Sicurezza (Logica Positiva per OPTIC)
    (safe ?l - location)
  )

  (:durative-action move
    :parameters (?r - rover ?from - location ?to - location)
    :duration (= ?duration 10)
    :condition (and
        (at start (at ?r ?from))
        (over all (connected ?from ?to))
        (over all (safe ?to))
    )
    :effect (and
        (at start (not (at ?r ?from)))
        (at end (at ?r ?to))
    )
  )

  (:durative-action pick_up
    :parameters (?r - rover ?i - object ?l - location ?c_pre - count ?c_post - count)
    :duration (= ?duration 5)
    :condition (and
        (over all (at ?r ?l))
        (at start (at ?i ?l))
        (over all (transportable ?i))
        (at start (load ?r ?c_pre))
        (over all (next ?c_pre ?c_post))
        (over all (capacity-ok ?r ?c_post))
        (over all (safe ?l))
    )
    :effect (and
        (at start (not (at ?i ?l)))
        (at start (not (load ?r ?c_pre)))
        (at end (load ?r ?c_post))
        (at end (holding ?r ?i))
    )
  )

  (:durative-action drop
    :parameters (?r - rover ?i - object ?l - location ?c_pre - count ?c_post - count)
    :duration (= ?duration 5)
    :condition (and
        (over all (at ?r ?l))
        (at start (holding ?r ?i))
        (at start (load ?r ?c_pre))
        (over all (next ?c_post ?c_pre))
        (over all (safe ?l))
    )
    :effect (and
        (at start (not (holding ?r ?i)))
        (at start (not (load ?r ?c_pre)))
        (at end (load ?r ?c_post))
        (at end (at ?i ?l))
    )
  )

  (:durative-action load_pod
    :parameters (?r - rover ?art - artifact ?pod - pod ?l - location ?c_pre - count ?c_post - count)
    :duration (= ?duration 8)
    :condition (and
        (over all (at ?r ?l))
        (at start (holding ?r ?art))
        (over all (holding ?r ?pod))
        (at start (empty ?pod))
        (at start (load ?r ?c_pre))
        (over all (next ?c_post ?c_pre))
        (over all (safe ?l))
    )
    :effect (and
        (at start (not (holding ?r ?art)))
        (at start (not (empty ?pod)))
        (at start (not (load ?r ?c_pre)))
        (at end (load ?r ?c_post))
        (at end (inside ?art ?pod))
    )
  )

  (:durative-action unload_pod
    :parameters (?r - rover ?art - artifact ?pod - pod ?l - location ?c_pre - count ?c_post - count)
    :duration (= ?duration 8)
    :condition (and
        (over all (at ?r ?l))
        (over all (holding ?r ?pod))
        (at start (inside ?art ?pod))
        (at start (load ?r ?c_pre))
        (over all (next ?c_pre ?c_post))
        (over all (capacity-ok ?r ?c_post))
        (over all (safe ?l))
    )
    :effect (and
        (at start (not (inside ?art ?pod)))
        (at end (empty ?pod))
        (at end (holding ?r ?art))
        (at start (not (load ?r ?c_pre)))
        (at end (load ?r ?c_post))
    )
  )

  (:durative-action cool_down
    :parameters (?r - rover ?art - artifact ?l - location)
    :duration (= ?duration 5)
    :condition (and
        (over all (at ?r ?l))
        (over all (holding ?r ?art))
        (over all (has-cooling ?r))
        (at start (needs-cooling ?art))
        (over all (safe ?l))
    )
    :effect (and
        (at end (not (needs-cooling ?art)))
        (at end (cooled ?art))
    )
  )
)