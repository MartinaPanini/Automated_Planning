(define (domain imv-multi-agent)
  (:requirements :strips :typing :negation :disjunctive-preconditions) 
  ;; Rimossi conditional-effects, universal-preconditions e existential-preconditions

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
    (transportable ?i - item)      
    (seismic-active ?l - location) 
    (blocked ?a - agent)       ;; Blocco fisico (oggetto fragile)
    (temp-blocked ?a - agent)  ;; Blocco termico (oggetto caldo)
    (needs-cooling ?a - artifact) 
    (cooled ?a - artifact)        
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
        (not (temp-blocked ?a))
    )
    :effect (and (not (at ?a ?from)) (at ?a ?to))
  )

  ;; --- RACCOLTA (SPLIT ACTIONS) ---
  
  ;; Caso 1: Oggetto Standard (Transportable E (Freddo O Già Raffreddato))
  (:action pick-up-standard
    :parameters (?a - agent ?i - item ?l - location ?c_pre ?c_post - count)
    :precondition (and 
        (at ?a ?l) (at ?i ?l)
        (load ?a ?c_pre) (next ?c_pre ?c_post) (capacity-ok ?a ?c_post)
        (not (seismic-active ?l))
        (transportable ?i)
        (or (not (needs-cooling ?i)) (cooled ?i))
    )
    :effect (and 
        (not (at ?i ?l)) (holding ?a ?i)
        (not (load ?a ?c_pre)) (load ?a ?c_post)
    )
  )

  ;; Caso 2: Oggetto Caldo (Transportable MA Needs-Cooling e NOT Cooled)
  (:action pick-up-hot
    :parameters (?a - agent ?i - item ?l - location ?c_pre ?c_post - count)
    :precondition (and 
        (at ?a ?l) (at ?i ?l)
        (load ?a ?c_pre) (next ?c_pre ?c_post) (capacity-ok ?a ?c_post)
        (not (seismic-active ?l))
        (transportable ?i)
        (needs-cooling ?i) 
        (not (cooled ?i))
    )
    :effect (and 
        (not (at ?i ?l)) (holding ?a ?i)
        (not (load ?a ?c_pre)) (load ?a ?c_post)
        (temp-blocked ?a) ;; Ti scotti le mani -> non ti muovi
    )
  )

  ;; Caso 3: Oggetto Fragile (NON Transportable)
  (:action pick-up-fragile
    :parameters (?a - agent ?i - item ?l - location ?c_pre ?c_post - count ?p - pod)
    :precondition (and 
        (at ?a ?l) (at ?i ?l)
        (load ?a ?c_pre) (next ?c_pre ?c_post) (capacity-ok ?a ?c_post)
        (not (seismic-active ?l))
        (not (transportable ?i))
        ;; Vincolo Pod: Devi avere un pod vuoto in mano
        (holding ?a ?p) (empty ?p)
    )
    :effect (and 
        (not (at ?i ?l)) (holding ?a ?i)
        (not (load ?a ?c_pre)) (load ?a ?c_post)
        (blocked ?a) ;; Fragile -> non ti muovi
    )
  )

  ;; --- RILASCIO (SPLIT ACTIONS) ---
  
  ;; Rilascio Standard (Non rimuove blocchi perché non ce n'erano)
  (:action drop-standard
    :parameters (?a - agent ?i - item ?l - location ?c_pre ?c_post - count)
    :precondition (and 
        (at ?a ?l) (holding ?a ?i)
        (load ?a ?c_pre) (next ?c_post ?c_pre)
        (transportable ?i)
        (or (not (needs-cooling ?i)) (cooled ?i))
    )
    :effect (and 
        (at ?i ?l) (not (holding ?a ?i))
        (not (load ?a ?c_pre)) (load ?a ?c_post)
    )
  )

  ;; Rilascio Oggetto Caldo (Rimuove temp-blocked)
  (:action drop-hot
    :parameters (?a - agent ?i - item ?l - location ?c_pre ?c_post - count)
    :precondition (and 
        (at ?a ?l) (holding ?a ?i)
        (load ?a ?c_pre) (next ?c_post ?c_pre)
        (transportable ?i)
        (needs-cooling ?i) (not (cooled ?i))
    )
    :effect (and 
        (at ?i ?l) (not (holding ?a ?i))
        (not (load ?a ?c_pre)) (load ?a ?c_post)
        (not (temp-blocked ?a))
    )
  )

  ;; Rilascio Oggetto Fragile (Rimuove blocked)
  (:action drop-fragile
    :parameters (?a - agent ?i - item ?l - location ?c_pre ?c_post - count)
    :precondition (and 
        (at ?a ?l) (holding ?a ?i)
        (load ?a ?c_pre) (next ?c_post ?c_pre)
        (not (transportable ?i))
    )
    :effect (and 
        (at ?i ?l) (not (holding ?a ?i))
        (not (load ?a ?c_pre)) (load ?a ?c_post)
        (not (blocked ?a))
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
        (not (blocked ?a)) ;; Loading rimuove sempre il blocco fisico
    )
  )

  ;; Unload Split: Standard vs Fragile
  
  (:action unload-pod-standard
    :parameters (?a - agent ?art - item ?pod - item ?l - location ?c_pre ?c_post - count)
    :precondition (and 
        (at ?a ?l) (at ?pod ?l) 
        (inside ?art ?pod)
        (load ?a ?c_pre) (next ?c_pre ?c_post) (capacity-ok ?a ?c_post) 
        (not (seismic-active ?l))
        (transportable ?art) ;; Se è robusto
    )
    :effect (and 
        (holding ?a ?art)
        (empty ?pod) (not (inside ?art ?pod))
        (not (load ?a ?c_pre)) (load ?a ?c_post)
    )
  )

  (:action unload-pod-fragile
    :parameters (?a - agent ?art - item ?pod - item ?l - location ?c_pre ?c_post - count)
    :precondition (and 
        (at ?a ?l) (at ?pod ?l) 
        (inside ?art ?pod)
        (load ?a ?c_pre) (next ?c_pre ?c_post) (capacity-ok ?a ?c_post) 
        (not (seismic-active ?l))
        (not (transportable ?art)) ;; Se è fragile
    )
    :effect (and 
        (holding ?a ?art)
        (empty ?pod) (not (inside ?art ?pod))
        (not (load ?a ?c_pre)) (load ?a ?c_post)
        (blocked ?a) ;; Diventi bloccato
    )
  )

  ;; --- RAFFREDDAMENTO ---
  (:action cool-down
    :parameters (?ag - agent ?a - artifact ?l - location)
    :precondition (and 
        (at ?ag ?l) 
        (holding ?ag ?a) 
        (needs-cooling ?a)
        (not (cooled ?a))
    )
    :effect (and 
        (cooled ?a)
        (not (temp-blocked ?ag)) ;; Rimuove il blocco termico
    )
  )
)