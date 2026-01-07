(define (domain imv-multi-agent)
  (:requirements :strips :typing :negation :disjunctive-preconditions) 
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
    (blocked ?a - agent)      
    (temp-blocked ?a - agent) 
    (needs-cooling ?a - artifact) 
    (cooled ?a - artifact)        
  )

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
        (temp-blocked ?a) 
    )
  )

  (:action pick-up-fragile
    :parameters (?a - agent ?i - item ?l - location ?c_pre ?c_post - count ?p - pod)
    :precondition (and 
        (at ?a ?l) (at ?i ?l)
        (load ?a ?c_pre) (next ?c_pre ?c_post) (capacity-ok ?a ?c_post)
        (not (seismic-active ?l))
        (not (transportable ?i))
        (holding ?a ?p) (empty ?p)
    )
    :effect (and 
        (not (at ?i ?l)) (holding ?a ?i)
        (not (load ?a ?c_pre)) (load ?a ?c_post)
        (blocked ?a) 
    )
  )
  
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
        (not (blocked ?a)) 
    )
  )

  (:action unload-pod-standard
    :parameters (?a - agent ?art - item ?pod - item ?l - location ?c_pre ?c_post - count)
    :precondition (and 
        (at ?a ?l) (at ?pod ?l) 
        (inside ?art ?pod)
        (load ?a ?c_pre) (next ?c_pre ?c_post) (capacity-ok ?a ?c_post) 
        (not (seismic-active ?l))
        (transportable ?art)
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
        (not (transportable ?art)) 
    )
    :effect (and 
        (holding ?a ?art)
        (empty ?pod) (not (inside ?art ?pod))
        (not (load ?a ?c_pre)) (load ?a ?c_post)
        (blocked ?a) 
    )
  )

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
        (not (temp-blocked ?ag)) 
    )
  )
)