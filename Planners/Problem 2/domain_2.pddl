(define (domain imv-multi-agent)
  (:requirements :strips :typing :negation :conditional-effects :existential-preconditions)

  (:types
    location item agent count - object
    robot drone - agent
    artifact pod - item
  )

  (:predicates
    (at ?o - object ?l - location)
    (connected ?l1 ?l2 - location)
    
    ;; Gestione Inventario
    (holding ?a - agent ?i - item)
    (load ?a - agent ?c - count)         
    (next ?c1 ?c2 - count)               
    (capacity-ok ?a - agent ?c - count)  
    
    ;; Proprietà Oggetti
    (inside ?art - item ?p - item)
    (empty ?p - item)
    (transportable ?i - item)      
    
    ;; Ambiente
    (seismic-active ?l - location) 
    
    ;; Stato di Blocco
    (blocked ?a - agent) 
  )

  ;; --- MOVIMENTO ---
  
  (:action move-freely
    :parameters (?a - agent ?from ?to - location)
    :precondition (and 
        (at ?a ?from) 
        (connected ?from ?to)
        (load ?a n0) 
    )
    :effect (and (not (at ?a ?from)) (at ?a ?to))
  )

  (:action move-carrying
    :parameters (?a - agent ?from ?to - location)
    :precondition (and 
        (at ?a ?from) 
        (connected ?from ?to)
        (not (load ?a n0)) 
        (not (blocked ?a)) 
    )
    :effect (and (not (at ?a ?from)) (at ?a ?to))
  )

  ;; --- RACCOLTA (MODIFICATA) ---
  
  (:action pick-up
    :parameters (?a - agent ?i - item ?l - location ?c_pre ?c_post - count)
    :precondition (and 
        (at ?a ?l) (at ?i ?l)
        (load ?a ?c_pre) (next ?c_pre ?c_post) (capacity-ok ?a ?c_post)
        (not (seismic-active ?l)) 
        
        ;; --- NUOVA CONDIZIONE ---
        ;; Se l'oggetto è transportable (sicuro), OK.
        ;; Se NON lo è (fragile), devi GIA' avere in mano un pod vuoto.
        (or (transportable ?i)
            (exists (?p - pod) (and (holding ?a ?p) (empty ?p)))
        )
    )
    :effect (and 
        (not (at ?i ?l)) 
        (holding ?a ?i)
        (not (load ?a ?c_pre)) (load ?a ?c_post)
        (when (not (transportable ?i)) (blocked ?a))
    )
  )

  ;; --- RILASCIO ---
  
  (:action drop
    :parameters (?a - agent ?i - item ?l - location ?c_pre ?c_post - count)
    :precondition (and 
        (at ?a ?l) (holding ?a ?i)
        (load ?a ?c_pre) (next ?c_post ?c_pre)
    )
    :effect (and 
        (at ?i ?l) 
        (not (holding ?a ?i))
        (not (load ?a ?c_pre)) (load ?a ?c_post)
        
        ;; Sblocca SOLO se posi l'oggetto fragile. 
        ;; Se posi il Pod (oggetto sicuro) mentre hai l'artefatto fragile nell'altra mano, 
        ;; rimani bloccato (corretto, perché non puoi muoverti col fragile nudo).
        (when (not (transportable ?i)) (not (blocked ?a)))
    )
  )

  ;; --- GESTIONE POD ---

  (:action load-pod
    :parameters (?a - agent ?art - item ?pod - item ?l - location ?c_pre ?c_post - count)
    :precondition (and 
        (at ?a ?l) (at ?pod ?l) (holding ?a ?art) (empty ?pod)
        (load ?a ?c_pre) (next ?c_post ?c_pre) 
        (not (seismic-active ?l))
    )
    :effect (and 
        (not (holding ?a ?art))
        (not (empty ?pod)) 
        (inside ?art ?pod)
        (not (load ?a ?c_pre)) (load ?a ?c_post)
        ;; L'artefatto è al sicuro -> sblocco
        (not (blocked ?a))
    )
  )

  (:action unload-pod
    :parameters (?a - agent ?art - item ?pod - item ?l - location ?c_pre ?c_post - count)
    :precondition (and 
        (at ?a ?l) (at ?pod ?l) (inside ?art ?pod)
        (load ?a ?c_pre) (next ?c_pre ?c_post) (capacity-ok ?a ?c_post) 
        (not (seismic-active ?l))
    )
    :effect (and 
        (holding ?a ?art)
        (empty ?pod)
        (not (inside ?art ?pod))
        (not (load ?a ?c_pre)) (load ?a ?c_post)
        (when (not (transportable ?art)) (blocked ?a))
    )
  )
)