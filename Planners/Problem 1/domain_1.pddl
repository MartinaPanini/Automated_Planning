(define (domain imv-simple)
  (:requirements :strips :typing :negation) ;; Aggiunto :negation

  (:types
    robot location item - object
  )

  (:predicates
    (at ?o - object ?l - location)
    (connected ?l1 ?l2 - location)
    (holding ?r - robot ?i - item)
    (handempty ?r - robot)
    
    (is-artifact ?i - item)
    (is-pod ?i - item)
    (inside ?a - item ?p - item)
    (empty ?p - item)
    
    (transportable ?i - item) ; Se vero, l'oggetto può essere trasportato tra stanze
    
    ;; NUOVO PREDICATO SISMICO
    (seismic-active ?l - location) ; Vero se la stanza è sotto terremoto
  )

  ;; Movimento a mani vuote (Sempre possibile, anche col terremoto, per scappare)
  (:action move-empty
    :parameters (?r - robot ?from ?to - location)
    :precondition (and 
        (at ?r ?from) 
        (connected ?from ?to)
        (handempty ?r)
    )
    :effect (and 
        (not (at ?r ?from)) 
        (at ?r ?to)
    )
  )

  ;; Movimento con carico (Solo se l'oggetto è sicuro)
  (:action move-carrying
    :parameters (?r - robot ?from ?to - location ?i - item)
    :precondition (and 
        (at ?r ?from) 
        (connected ?from ?to)
        (holding ?r ?i)
        (transportable ?i)
    )
    :effect (and 
        (not (at ?r ?from)) 
        (at ?r ?to)
    )
  )

  ;; Raccolta: BLOCCATA SE C'È SISMA
  (:action pick-up
    :parameters (?r - robot ?i - item ?l - location)
    :precondition (and 
        (at ?r ?l) 
        (at ?i ?l) 
        (handempty ?r)
        ;; VINCOLO DI SICUREZZA AGGIUNTO
        (not (seismic-active ?l))
    )
    :effect (and (not (at ?i ?l)) (not (handempty ?r)) (holding ?r ?i))
  )

  (:action drop
    :parameters (?r - robot ?i - item ?l - location)
    :precondition (and (at ?r ?l) (holding ?r ?i))
    :effect (and (at ?i ?l) (handempty ?r) (not (holding ?r ?i)))
  )

  ;; Caricamento nel Pod: Blocchiamo anche questo se c'è sisma (operazione delicata)
  (:action load-pod
    :parameters (?r - robot ?a - item ?p - item ?l - location)
    :precondition (and 
        (at ?r ?l) (at ?p ?l) (holding ?r ?a) (empty ?p)
        (is-artifact ?a) (is-pod ?p)
        (not (seismic-active ?l)) ;; Sicurezza
    )
    :effect (and 
        (not (holding ?r ?a)) (handempty ?r)
        (not (empty ?p)) (inside ?a ?p)
    )
  )

  (:action unload-pod
    :parameters (?r - robot ?a - item ?p - item ?l - location)
    :precondition (and 
        (at ?r ?l) (at ?p ?l) (handempty ?r) (inside ?a ?p)
        (not (seismic-active ?l)) ;; Sicurezza
    )
    :effect (and (holding ?r ?a) (not (handempty ?r)) (empty ?p) (not (inside ?a ?p)))
  )
)