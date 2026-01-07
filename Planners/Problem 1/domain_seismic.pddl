(define (domain imv-seismic)
  (:requirements :strips :typing :negation :disjunctive-preconditions)

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
    
    (transportable ?i - item)
    (seismic-active ?l - location)
    
    (needs-cooling ?i - item)
    (cooled ?i - item)
  )

  (:action wait-for-stability
    :parameters (?l - location)
    :precondition (seismic-active ?l)
    :effect (not (seismic-active ?l))
  )

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

  (:action move-carrying
    :parameters (?r - robot ?from ?to - location ?i - item)
    :precondition (and 
        (at ?r ?from) 
        (connected ?from ?to)
        (holding ?r ?i)
        (transportable ?i)
        (or (not (needs-cooling ?i)) (cooled ?i))
    )
    :effect (and 
        (not (at ?r ?from)) 
        (at ?r ?to)
    )
  )

  (:action cool-down
    :parameters (?r - robot ?i - item ?l - location)
    :precondition (and 
        (at ?r ?l) 
        (holding ?r ?i) 
        (needs-cooling ?i)
        (not (cooled ?i))
    )
    :effect (cooled ?i)
  )

  (:action pick-up
    :parameters (?r - robot ?i - item ?l - location)
    :precondition (and 
        (at ?r ?l) 
        (at ?i ?l) 
        (handempty ?r)
        (not (seismic-active ?l))
    )
    :effect (and (not (at ?i ?l)) (not (handempty ?r)) (holding ?r ?i))
  )

  (:action drop
    :parameters (?r - robot ?i - item ?l - location)
    :precondition (and (at ?r ?l) (holding ?r ?i))
    :effect (and (at ?i ?l) (handempty ?r) (not (holding ?r ?i)))
  )

  (:action load-pod
    :parameters (?r - robot ?a - item ?p - item ?l - location)
    :precondition (and 
        (at ?r ?l) (at ?p ?l) (holding ?r ?a) (empty ?p)
        (is-artifact ?a) (is-pod ?p)
        (not (seismic-active ?l))
    )
    :effect (and 
        (not (holding ?r ?a)) (handempty ?r)
        (not (empty ?p)) (inside ?a ?p)
    )
  )

  (:action unload-pod
    :parameters (?r - robot ?a - item ?p - item ?l - location)
    :precondition (and 
        (at ?r ?l) (at ?p ?l) 
        (inside ?a ?p)
        (not (seismic-active ?l))
        (handempty ?r)
    )
    :effect (and 
        (holding ?r ?a) (not (handempty ?r))
        (empty ?p) (not (inside ?a ?p))
    )
  )
)