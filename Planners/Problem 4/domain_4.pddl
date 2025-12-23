(define (domain imv-temporal)
  (:requirements :strips :typing :durative-actions)

  (:types
    location item agent count - object
    robot drone - agent
    artifact pod - item
  )

  (:predicates
    (at ?o - object ?l - location)
    (connected ?l1 ?l2 - location)
    (holding ?a - agent ?i - item)
    (load ?a - agent ?c - count)
    (next ?c1 ?c2 - count)
    (capacity-ok ?a - agent ?c - count)
    (inside ?art - item ?p - item)
    (empty ?p - item)
    
    ;; PREDICATI POSITIVI (Niente 'not' nelle precondizioni)
    (transportable ?i - item)    ;; Oggetto sicuro
    (fragile ?i - item)          ;; Oggetto fragile (sostituisce 'not transportable')
    (can-move ?a - agent)        ;; Agente libero (sostituisce 'not blocked')
    (safe-location ?l - location);; Luogo sicuro (sostituisce 'not seismic')
  )

  ;; --- MOVIMENTO UNIFICATO ---
  ;; Ora c'è una sola mossa. Se (can-move) è vero, ti muovi.
  ;; Se hai in mano roba fragile, (can-move) sarà falso e non potrai muoverti.
  
  (:durative-action move
    :parameters (?a - agent ?from ?to - location)
    :duration (= ?duration 10)
    :condition (and 
        (at start (at ?a ?from))
        (over all (connected ?from ?to))
        (at start (can-move ?a)) ;; Deve essere sbloccato
    )
    :effect (and 
        (at start (not (at ?a ?from)))
        (at end (at ?a ?to))
    )
  )

  ;; --- PICK UP (SPLIT) ---

  ;; 1. Pick Up Sicuro (Non blocca)
  (:durative-action pick-up-secure
    :parameters (?a - agent ?i - item ?l - location ?c_pre ?c_post - count)
    :duration (= ?duration 2)
    :condition (and 
        (at start (at ?a ?l))
        (over all (at ?a ?l))
        (at start (at ?i ?l))
        (at start (load ?a ?c_pre))
        (over all (next ?c_pre ?c_post))
        (over all (capacity-ok ?a ?c_post))
        (at start (safe-location ?l))
        (over all (transportable ?i)) ;; Solo per oggetti sicuri
    )
    :effect (and 
        (at start (not (at ?i ?l)))
        (at end (holding ?a ?i))
        (at start (not (load ?a ?c_pre)))
        (at end (load ?a ?c_post))
    )
  )

  ;; 2. Pick Up Fragile (Blocca + Richiede Pod)
  (:durative-action pick-up-fragile
    :parameters (?a - agent ?i - item ?p - pod ?l - location ?c_pre ?c_post - count)
    :duration (= ?duration 2)
    :condition (and 
        (at start (at ?a ?l))
        (over all (at ?a ?l))
        (at start (at ?i ?l))
        (at start (load ?a ?c_pre))
        (over all (next ?c_pre ?c_post))
        (over all (capacity-ok ?a ?c_post))
        (at start (safe-location ?l))
        
        (over all (fragile ?i))        ;; Solo per fragili
        (over all (holding ?a ?p))     ;; DEVE AVERE IL POD IN MANO
        (at start (empty ?p))
    )
    :effect (and 
        (at start (not (at ?i ?l)))
        (at end (holding ?a ?i))
        (at start (not (load ?a ?c_pre)))
        (at end (load ?a ?c_post))
        (at end (not (can-move ?a)))   ;; BLOCCA IL ROBOT (Delete effect è ok)
    )
  )

  ;; --- DROP (SPLIT) ---

  (:durative-action drop-secure
    :parameters (?a - agent ?i - item ?l - location ?c_pre ?c_post - count)
    :duration (= ?duration 2)
    :condition (and 
        (at start (at ?a ?l))
        (over all (at ?a ?l))
        (at start (holding ?a ?i))
        (at start (load ?a ?c_pre))
        (over all (next ?c_post ?c_pre))
        (over all (transportable ?i))
    )
    :effect (and 
        (at end (at ?i ?l))
        (at start (not (holding ?a ?i)))
        (at start (not (load ?a ?c_pre)))
        (at end (load ?a ?c_post))
    )
  )

  (:durative-action drop-fragile
    :parameters (?a - agent ?i - item ?l - location ?c_pre ?c_post - count)
    :duration (= ?duration 2)
    :condition (and 
        (at start (at ?a ?l))
        (over all (at ?a ?l))
        (at start (holding ?a ?i))
        (at start (load ?a ?c_pre))
        (over all (next ?c_post ?c_pre))
        (over all (fragile ?i))
    )
    :effect (and 
        (at end (at ?i ?l))
        (at start (not (holding ?a ?i)))
        (at start (not (load ?a ?c_pre)))
        (at end (load ?a ?c_post))
        (at end (can-move ?a))         ;; SBLOCCA IL ROBOT (Add effect è ok)
    )
  )

  ;; --- GESTIONE POD (Unificata) ---
  ;; Caricare nel pod rende tutto sicuro, quindi sblocca sempre.

  (:durative-action load-pod
    :parameters (?a - agent ?art - item ?pod - item ?l - location ?c_pre ?c_post - count)
    :duration (= ?duration 5)
    :condition (and 
        (at start (at ?a ?l))
        (over all (at ?a ?l))
        (at start (at ?pod ?l))
        (at start (holding ?a ?art))
        (at start (empty ?pod))
        (at start (load ?a ?c_pre))
        (over all (next ?c_post ?c_pre))
        (at start (safe-location ?l))
    )
    :effect (and 
        (at start (not (holding ?a ?art)))
        (at start (not (empty ?pod)))
        (at end (inside ?art ?pod))
        (at start (not (load ?a ?c_pre)))
        (at end (load ?a ?c_post))
        (at end (can-move ?a))         ;; SBLOCCA SEMPRE
    )
  )

  ;; --- UNLOAD (SPLIT) ---

  (:durative-action unload-pod-secure
    :parameters (?a - agent ?art - item ?pod - item ?l - location ?c_pre ?c_post - count)
    :duration (= ?duration 5)
    :condition (and 
        (at start (at ?a ?l))
        (over all (at ?a ?l))
        (at start (at ?pod ?l))
        (at start (inside ?art ?pod))
        (at start (load ?a ?c_pre))
        (over all (next ?c_pre ?c_post))
        (over all (capacity-ok ?a ?c_post))
        (at start (safe-location ?l))
        (over all (transportable ?art))
    )
    :effect (and 
        (at end (holding ?a ?art))
        (at end (empty ?pod))
        (at start (not (inside ?art ?pod)))
        (at start (not (load ?a ?c_pre)))
        (at end (load ?a ?c_post))
    )
  )

  (:durative-action unload-pod-fragile
    :parameters (?a - agent ?art - item ?pod - item ?l - location ?c_pre ?c_post - count)
    :duration (= ?duration 5)
    :condition (and 
        (at start (at ?a ?l))
        (over all (at ?a ?l))
        (at start (at ?pod ?l))
        (at start (inside ?art ?pod))
        (at start (load ?a ?c_pre))
        (over all (next ?c_pre ?c_post))
        (over all (capacity-ok ?a ?c_post))
        (at start (safe-location ?l))
        (over all (fragile ?art))
    )
    :effect (and 
        (at end (holding ?a ?art))
        (at end (empty ?pod))
        (at start (not (inside ?art ?pod)))
        (at start (not (load ?a ?c_pre)))
        (at end (load ?a ?c_post))
        (at end (not (can-move ?a)))   ;; BLOCCA
    )
  )
)