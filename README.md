# Libraries used:

Everything has been developed with Python3.9
and has been tested on
<span style="color:red">MacOS Sonoma 14.4 and Ubuntu 18?</span> 

## General libraries

- [NumPy](https://numpy.org/)
- [polytope](http://tulip-control.github.io/polytope/)
- [matplotlib](https://matplotlib.org/) (*Optional for plotting*)

## Libraries for FACTEST

Depending on which version of FACTEST is used, three are different solvers and libraries which are required. 
We give a brief description of each version of FACTEST as well as the different libraries used for each version.

### Base FACTEST options

This is the basic version of FACTEST which synthesizes controllers for agents with static reach-avoid specifications.
A description of the reach-avoid problem and the basic algorithm can be found in the [original CAV 2020 FACTEST paper](https://link.springer.com/chapter/10.1007/978-3-030-53288-8_31).
This version of FACTEST has been implemented with three different solvers.

*Choose one of the following*
- [Z3](https://ericpony.github.io/z3py-tutorial/guide-examples.htm)
- [yices2](https://github.com/SRI-CSL/yices2_python_bindings)
- [Gurobi](https://www.gurobi.com/)

### Omega FACTEST options

This is the omega-regular version of FACTEST which synthesizes controllers for agents with omega-regular specifications and static environments.
A description of the omega-regular synthesis problem and Omega-FACTEST algorithm can be found in the omega-regular FACTEST paper.
This version of FACTEST has been implemented using the Z3 solver and uses the Spot library to construct Büchi automaton.

*The following are required*
- [Z3](https://ericpony.github.io/z3py-tutorial/guide-examples.htm)
- [Spot](https://spot.lre.epita.fr/)

### Dynamic FACTEST options

This is the dynamic version of FACTEST which synthesizes controllers for agents in scenarios where the environments is dynamic or partially unknown.
The FACTEST algorithm is deployed in a receding horizon fashion.
A description of the problem and Dynamic-FACTEST algorithm can be found in the [ADHS 2021 FACTEST paper](https://www.sciencedirect.com/science/article/pii/S2405896321012684).
This version of FACTEST has been implemented using Gurobi.

*The following are required*
- [Gurobi](https://www.gurobi.com/)

# Demos and tests
To see if FACTEST is running properly on your machine, you can run the following scripts.
To test the base version of FACTEST, we have provided the examples from the [original CAV 2020 FACTEST paper](https://link.springer.com/chapter/10.1007/978-3-030-53288-8_31),
which can be run using the following command:
```
python3.9 demo/demo_cav20.py [model] [env] [--solver] [--segs] [--parts] [--print] [--plot]
```
| Arg      | Description                    | Options |
| -------- | ------------------------------ | ------- |
| `model`  | agent model                    | `car` |
| `env`    | initial, goal, and unsafe sets | `maze_2d`, `scots_hscc16` |
| `solver` | solver to be used with FACTEST | `z3`, `yices`, `gurobi` |
| `segs`   | number of line segments        | any positive integer |
| `parts`  | maximum partition depth        | any positive integer |
| `print`  | print statements for FACTEST   | `True`, `False` |
| `plot`   | plot the resulting solution    | `True`, `False` |

<span style="color:red">TODO: We will also provide demos for Dynamic FACTEST and Omega FACTEST
</span>


# Using FACTEST
Here, we provide a brief description on how to use {Base, Omega, Dynamic}-FACTEST.
There are three main components to using FACTEST: (i) the agent model and tracking controller, (ii) the environment defined by a workspace and polytope regions, and (iii) the {Base, Omega, Dynamic}-FACTEST synthesis algorithm.
In this section, we provide a tutorial on how each of these components works and how they are brought together to synthesize controllers.


## Base-FACTEST

The base version of FACTEST has been implemented with three different solvers: Z3 (SMT), Yices (SMT), and Gurobi (MIP).
The arguments to run each one are the same and described here.

```
factest = FACTEST_z3(initial_poly, goal_poly, unsafe_polys, model = None, workspace = None, seg_max = 3, part_max = 2, print_statements = True)
```

```
factest = FACTEST_yices(initial_poly, goal_poly, unsafe_polys, model = None, workspace = None, seg_max = 3, part_max = 2, print_statements = True)
```

```
factest = FACTEST_gurobi(initial_poly, goal_poly, unsafe_polys, model = None, workspace = None, seg_max = 3, part_max = 2, print_statements = True)
```

| Arg                | Type              | Default | Description |
| ------------------ | ----------------- | ------- | ----------- |
| `initial_poly`     | Polytope          |         | Set of possible initial positions for the agent                   |
| `goal_poly`        | Polytope          |         | Set of possible positions which the agent must reach              |
| `unsafe_polys`     | list of Polytopes |         | List of sets in the workspace that the agent must avoid           |
| `model`            |                   | `None`  | Agent and tracking controller dynamics                            |
| `workspace`        | Polytope          | `None`  | *Optional*: Closed set of positions that the agent must remain in |
| `seg_max`          | int               | `3`     | Maximum number of segments for the reference trajectories         |
| `part_max`         | int               | `2`     | Maximum partition depth of the initial set                        |
| `print_statements` | bool              | `True`  | *Optiona*: Turn print statements on or off                        |

The controllers are synthesized by running the following command.

```
controllers = factest.run()
```

The resulting controller is given in the following format:
```
controllers = {controller_idx0 : {'init': Polytope,
                                  'xref': [[x0], [x1], ..]}, 
                controller_idx1 : {'init': Polytope,
                                   'xref': [[x0], [x1], ..]}, 
                ...}
```
where each controller index is a str, 
`'init'` returns the polytope region for which the controller is valid, and
`'xref'` is the sequence of waypoints that the agent must track.

## Omega-FACTEST

Omega-FACTEST uses the base version of FACTEST to construct a hybrid automaton.
We provide the functionality to construct two types of automata: a transition-based Buchi automaton (TBA) and a hybrid automaton.

### TBA synthesis

A TBA is constructed from a linear temporal logic (LTL) formula defined over some atomic propositions (AP) or letters.
We refer to a set of letters as a *label*.
An environment is a set of workspace polytopes and their associated labels.
A TBA is constructed using the following command:

```
disc_aut = buchi_from_ltl(ltl_formula, env)
```

| Arg           | Type                                       | Default | Description |
| ------------- | ------------------------------------------ | ------- | ----------- |
| `ltl_formula` | str                                        |         | LTL specifications using Spot operators |
| `env`         | dict (keys: label (str), entries: Polytope)|         | Polytopes and associated labels         |

A TBA has the following properties:

| Property            | Type                                                     | Description |
| ------------------- | -------------------------------------------------------- | ----------- |
| `buchi_states`      | list of str                                              | List of büchi automaton states                                                                 |
| `buchi_inits`       | list of str                                              | List of possible initial states                                                                |
| `buchi_AP`          | list of str                                              | List of AP                                                               |
| `buchi_alphabet`    | list of str                                              | List of letters which can label transitions (power set of AP)                                  |
| `buchi_transitions` | dict (keys: letter (str), entries: list of tuples)       | Dictionary which maps each state to a list of possible transitions of the form (letter, state) |
| `buchi_acceptances` | *not implemented*                                        | *not implemented*                                                                              |
| `buchi_run`         | dict (keys: 'prefix', 'cycle', entries: list of letters) | Word accepted by TBA. Prefix happens first, cycle repeats infinitely                           |


### Hybrid automaton synthesis

A hybrid automaton is constructed from an LTL formula defined over some AP, the environment, and some agent and tracking controller dynamics.
A hybrid automaton is constructed using the following command:

```
hybrid_aut = hybrid_from_ltl(ltl_formula, env, model = None, workspace = None)
```

| Arg           | Type                                       | Default | Description |
| ------------- | ------------------------------------------ | ------- | ----------- |
| `ltl_formula` | str                                        |         | LTL specifications using Spot operators                           |
| `env`         | dict (keys: label (str), entries: Polytope)|         | Polytopes and associated labels                                   |
| `model`       |                                            | `None`  | Agent and tracking controller dynamics                            |
| `workspace`   | Polytope                                   | `None`  | *Optional*: Closed set of positions that the agent must remain in |

The hybrid automaton has all the same properties as the TBA, as well as the following additional properties.

| Property                  | Type                                                 | Description |
| ------------------------- | ---------------------------------------------------- | ----------- |
| `flows`                   | dict (keys: state (str), entries: dict (keys: letter (str), entries: dict (keys: ['init', 'xref'], entries: [Polytope, list of lists of floats]))) | Maps each state and transition label to an initial set partition and associated controllers |
| `buchi_state_init_labels` | dict (keys: state (str), entries: list of Polytopes) | Maps each state to a set of initial positions the agent can take in that state       |
| `transition_reqs`         | dict (keys: letter(str), entries: Polytope)          | Maps each transition label to a set of positions for which that transition can occur |

A hybrid automaton simulates a TBA.
The discrete states between the hybrid automaton and TBA are the same, and the "flow" or trajectory of an agent simulates a transition of the TBA.

## Dynamic-FACTEST

The dynamic version of FACTEST is similar to the base version of FACTEST; however, dynamic-FACTEST can handle obstacles defined in space-time.
Instead of returning a sequence of waypoints that the agent must track, dynamic-FACTEST returns a sequence of time-stamped waypoints.

```
dynamic_factest = dynamic_FACTEST_gurobi(initial_poly, goal_poly, unsafe_polys, timed_unsafe_polys = [], dt = 1, max_length = 5, model=None, workspace=None, seg_max=6, part_max=2, print_statements=True)
```

| Arg                  | Type              | Default | Description |
| -------------------- | ----------------- | ------- | ----------- |
| `initial_poly`       | Polytope          |         | Set of possible initial positions for the agent                   |
| `goal_poly`          | Polytope          |         | Set of possible positions which the agent must reach              |
| `unsafe_polys`       | list of Polytopes |         | List of sets in the workspace that the agent must avoid           |
| `timed_unsafe_polys` | list of Polytopes | `[]`    | List of timed sets in the workspace that the agent must avoid     |
| `dt`                 | float             | `None`  | Time difference between two consecutive waypoints                 |
| `max_length`         | float             | `None`  | Maximum distance between two consecutive waypoints                |
| `model`              |                   | `None`  | Agent and tracking controller dynamics                            |
| `workspace`          | Polytope          | `None`  | *Optional*: Closed set of positions that the agent must remain in |
| `seg_max`            | int               | `6`     | Maximum number of segments for the reference trajectories         |
| `part_max`           | int               | `2`     | Maximum partition depth of the initial set                        |
| `print_statements`   | bool              | `True`  | *Optiona*: Turn print statements on or off                        |

```
dynamic_factest.run()
```

The resulting controller is given in the same format as base-FACTEST, however each `'xref'` is given as `[[x0,t0],[x1,t1], ...]`.

## Agent and tracking controller

The agent model encodes the agent and tracking controller dynamics.
When creating an agent, the user must include a function `errBound` which takes in a polytope `init_poly` and a line segment number `i`.
It should return the size of the error bound over segment `i`.
An example can be seen below:

```
class agent:
    def __init__(self):
        pass
    def errBound(self, init_poly, i):
        init_r = init_poly.chebR
        r = init_r + 0.1*i
        return r
```

An actual example for different vehicles can be found in the `demo/models` folder.

## Workspaces and polytope regions

The workspace is the 2- or 3-dimensional physical space in which the agent lives.
The polytope regions should be defined in this workspace with the only exception being the space-time polytopes used in dynamic-FACTEST.
Polytopes can be plotted via the `plotPoly` and `plotPoly_3d` functions in `factest/plotting/plot_polytopes.py`.

```
plotPoly(poly, ax = None, color = 'red')
plotPoly_3d(poly, ax = None, color='red')
```

| Arg     | Type            | Default | Description |
| ------- | --------------- | ------- | ---------------------- |
| `poly`  | Polytope        |         | Polytope to be plotted |
| `ax`    | matplotlib.axes | None    | Axes to plot `poly` on |
| `color` | str             | 'red'   | Color to plot polytope |


## Tutorial: running simulations

<span style="color:red">Tutorial needs to be fleshed out more</span> 

We provide limited functionality in running simulations of the controllers synthesized; however, we provide a short walkthrough on how we simulated our demo results.
In this tutorial, we simulate the Dubins car provided in `demo/models/dubins_car.py`.

### Base-FACTEST

```
def set_ref(self, xref, vref):
    self.ref_traj = []
    self.ref_input = []

    curr_time = 0
    prev_t = 0
    for i in range(len(xref)-1):
        p1 = xref[i]
        p2 = xref[i+1]

        mx = p2[0] - p1[0]
        bx = p1[0]
        
        my = p2[1] - p1[1]
        by = p1[1]
        
        theta_ref = np.arctan2((np.array(p2) - np.array(p1))[1], (np.array(p2) - np.array(p1))[0])

        t = np.linalg.norm(np.array(p2)-np.array(p1))/vref

        while curr_time <= t + prev_t:
            px = mx*((curr_time - prev_t)/t) + bx
            py = my*((curr_time - prev_t)/t) + by
            self.ref_traj.append((px,py,theta_ref))
            self.ref_input.append((vref,0))
            curr_time += self.dt

        prev_t += t

    return None

def run_simulation(self, xref, initial_state, T, vref = 1, sim_type = "base"):
    if sim_type == "base":
        self.set_ref(xref, vref)
    else:
        self.set_timed_ref(xref)

    time_array = np.arange(0,T,self.dt)
    state_trace = odeint(self.dubinsControlledDynamics, initial_state, time_array)
    return state_trace
```

```
FACTEST_prob = FACTEST_Z3(Theta, G, O, workspace, model, seg_max, part_max, print_statements)
result_dict = FACTEST_prob.run()
result_keys = list(result_dict.keys())
xref = result_dict[result_keys[0]]['xref']
states = model.run_simulation(xref,init_state,T)
```

### Omega-FACTEST

<span style="color:red">This may change more...</span> 

```
def run_omega_simulation(self, hybrid_aut, curr_state, vref = 1, num_cycles = 3):
    if len(hybrid_aut.buchi_inits) > 1:
        raise Exception('Runs not implemented for automata with multiple possible initial states!')


    curr_buchi_state = hybrid_aut.buchi_inits[0]

    prefix_run = hybrid_aut.buchi_run['prefix']
    cycle_run = hybrid_aut.buchi_run['cycle']

    all_states = []

    for transition in prefix_run:
        print('curr buchi state is ', curr_buchi_state)
        print('prefix transition is ', transition)
        possible_transitions = hybrid_aut.buchi_transitions[curr_buchi_state]

        possible_flows = hybrid_aut.flows[curr_buchi_state][str(transition)]
        found_flow = False
        for potential_flow in possible_flows:
            init_part = potential_flow['init']
            if init_part.contains(np.array([[curr_state[0]],[curr_state[1]]]))[0] and not found_flow:
                waypoints = potential_flow['xref']
                found_flow = True

        print('running simulation from ', curr_state)
        length = 0
        for i in range(1,len(waypoints)):
            length += np.linalg.norm(np.array(waypoints[i] - np.array(waypoints[i-1])))
        
        T = length/vref
        
        if length > 0:
            states = self.run_simulation(waypoints, curr_state, T, vref=vref)
            curr_state = states[-1]
            all_states.extend(states)

        for potential_transition in possible_transitions:
            if potential_transition[0] == transition:
                print('updating buchi state to ', potential_transition[1])
                curr_buchi_state = potential_transition[1]

    curr_cycle = 1
    while curr_cycle <= num_cycles:
        for transition in cycle_run:
            print('curr buchi state is ', curr_buchi_state)
            print('cycle ', curr_cycle,' transition is ', transition)
            possible_transitions = hybrid_aut.buchi_transitions[curr_buchi_state]

            possible_flows = hybrid_aut.flows[curr_buchi_state][str(transition)]
            found_flow = False
            for potential_flow in possible_flows:
                init_part = potential_flow['init']
                if init_part.contains(np.array([[curr_state[0]],[curr_state[1]]]))[0] and not found_flow:
                    waypoints = potential_flow['xref']
                    found_flow = True

            print('running simulation from ', curr_state)
            length = 0
            for i in range(1,len(waypoints)):
                length += np.linalg.norm(np.array(waypoints[i] - np.array(waypoints[i-1])))
            
            T = length/vref
            
            if length > 0:
                states = self.run_simulation(waypoints, curr_state, T, vref=vref)
                curr_state = states[-1]
                all_states.extend(states)

            for potential_transition in possible_transitions:
                if potential_transition[0] == transition:
                    print('updating buchi state to ', potential_transition[1])
                    curr_buchi_state = potential_transition[1]
        curr_cycle += 1
    
    return all_states
```

```
myHybrid = hybrid_from_ltl(ltl_formula,env, model, workspace)
all_states = model.run_omega_simulation(myHybrid, curr_state)
```

### Dynamic-FACTEST

```
def set_timed_ref(self, xref):
    self.ref_traj = []
    self.ref_input = []

    curr_time = 0
    prev_t = 0
    for i in range(len(xref)-1):
        p1 = xref[i]
        p2 = xref[i+1]

        mx = p2[0] - p1[0]
        bx = p1[0]
        
        my = p2[1] - p1[1]
        by = p1[1]
        
        theta_ref = np.arctan2((np.array(p2) - np.array(p1))[1], (np.array(p2) - np.array(p1))[0])

        t = p2[2] - p1[2]
        vref = np.linalg.norm(np.array(p2) - np.array(p1))/t

        while curr_time <= t + prev_t:
            px = mx*((curr_time - prev_t)/t) + bx
            py = my*((curr_time - prev_t)/t) + by
            self.ref_traj.append((px,py,theta_ref))
            self.ref_input.append((vref,0))
            curr_time += self.dt

        prev_t += t
        
    return None
```

```
FACTEST_prob = dynamic_FACTEST_gurobi(initial_poly, goal_poly, unsafe_polys, workspace, model)
result_dict = FACTEST_prob.run()
result_keys = list(result_dict.keys())
xref = result_dict[result_keys[0]]['xref']

model.set_timed_ref(xref)

states = model.run_simulation(xref, initial_state, T, vref = 1, sim_type = "timed")
```
