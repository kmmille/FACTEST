from factest_base_z3 import FACTEST_Z3

import spot
spot.setup()
import z3

####################
# Modified LTL2GBA #
####################
class buchi_from_ltl:
    def __init__(self, ltl_formula, env) -> None:
        self.ltl_str = ltl_formula
        self.env = env

        dim_mat = self.env[list(self.env.keys())[0]].A
        self.dims = len(dim_mat[0])

        self.getAlphabet()
        self.buchi_states, self.buchi_inits, self.buchi_AP, self.buchi_alphabet, self.buchi_transitions, self.buchi_acceptances, self.buchi_run = self.getBuchi()

    def getAlphabet(self):
        AP = list(self.env.keys())
        set_size = len(AP)
        power_set_size = 2**(set_size)
        
        self.alphabet = []

        # Code from: https://www.geeksforgeeks.org/power-set/
        for counter in range(0, power_set_size): 
            new_letter = []
            for j in range(0, set_size): 
                if((counter & (1 << j)) > 0): 
                    new_letter.append(AP[j])
                    
            self.alphabet.append(new_letter)
    
    def getBuchiAlphabet(self, AP_list):
        set_size = len(AP_list)
        power_set_size = 2**(set_size)
        
        buchi_alphabet = []

        # Code from: https://www.geeksforgeeks.org/power-set/
        for counter in range(0, power_set_size): 
            new_letter = []
            for j in range(0, set_size): 
                if((counter & (1 << j)) > 0): 
                    new_letter.append(AP_list[j])
                   
            buchi_alphabet.append(new_letter)
        
        return buchi_alphabet

    def checkIntersection(self, letter):
        #Code from: https://github.com/IllinoisReliableAutonomyGroup/topology-feasibility/blob/main/intersection.py 
        x = [z3.Real(f'x_{i}') for i in range(self.dims)]
        s = z3.Solver()

        for key in letter:
            env_poly = self.env[key]
            A = env_poly.A
            b = env_poly.b

            for i in range(len(A)):
                s.add(z3.Sum([A[i][j] * x[j] for j in range(self.dims)]) <= b[i])

        return s.check() == z3.sat

    def getExclusion(self):
        exclusion_str = 'true'
        AP_set = list(self.env.keys())
        for letter in self.alphabet:
            if letter == []:
                new_exclusion_str = ''
                for AP in AP_set:
                    if new_exclusion_str == '':
                        new_exclusion_str = '!'+str(AP)
                    else:
                        new_exclusion_str = new_exclusion_str + ' & !'+str(AP)
                exclusion_str = exclusion_str + ' & !('+new_exclusion_str+')'

            if not self.checkIntersection(letter):
                new_exclusion_str = ''
                for AP in AP_set:
                    if AP in letter:
                        if new_exclusion_str == '':
                            new_exclusion_str = AP
                        else:
                            new_exclusion_str = new_exclusion_str + ' & '+AP
                    else:
                        if new_exclusion_str == '':
                            new_exclusion_str = '!'+AP
                        else:
                            new_exclusion_str = new_exclusion_str + ' & !'+AP
                exclusion_str = exclusion_str + ' & !('+new_exclusion_str+')'
        return exclusion_str
        
    def getBuchi(self):
        exclusion_str = self.getExclusion()
        new_ltl_str = self.ltl_str+' & G ('+exclusion_str+')'
        
        a = spot.translate(new_ltl_str, 't')
        aut_string = a.to_str('hoa')

        buchi_states = []
        buchi_AP = []
        buchi_inits = []
        buchi_transitions = {}
        buchi_acceptances = {}

        for line in aut_string.splitlines():
            # Get initial states for automaton
            if 'Start:' in line:
                start_init_state = False
                init_state = ''
                for s in line:
                    if start_init_state:
                        init_state = init_state + s
                    if s == ' ':
                        start_init_state = True
                buchi_inits.append(init_state)

            # Get AP and the order used in TBA
            if 'AP:' in line:
                start_AP = False
                new_AP = ''
                for s in line:
                    if s == '"':
                        start_AP = not start_AP
                        if not start_AP:
                            buchi_AP.append(new_AP)
                            new_AP = ''
                    
                    if start_AP and s != '"':
                        new_AP = new_AP + s

            # Get accpetance conditions for the automata
            if 'Acceptance:' in line:
                start_acceptance = False
                num_acceptances = ''
                for s in line:
                    if s == ' ':
                        start_acceptance = not start_acceptance
                    if start_acceptance:
                        num_acceptances = num_acceptances + s
                for i in range(int(num_acceptances)):
                    buchi_acceptances[str(i)] = []
            
            # Get states used in TBA
            if 'State:' in line:
                start_state = False
                new_state = ''
                for s in line:
                    if start_state:
                        new_state = new_state + s

                    if s == ' ':
                        start_state = not start_state
                buchi_states.append(new_state)
                
                curr_state = new_state
                buchi_transitions[curr_state] = []

            # Get the transition function for the automata
            if '[' in line:
                end_transition = False
                transition = ''
                start_new_state = False
                new_state = ''
                start_acceptance_condition = False
                new_acceptance = '' #TODO: Need to update acceptance conditions
                acceptance_ids = []

                for s in line:
                    if not end_transition:
                        transition = transition + s
                        if s == ']':
                            end_transition = True
                            start_new_state = not start_new_state

                    elif start_new_state:
                        if s != ' ':
                            if s != '{':
                                new_state = new_state + s
                            else:
                                start_new_state = not start_new_state
                                start_acceptance_condition = True
                    
                    elif start_acceptance_condition:
                        if s != '}':
                            new_acceptance = new_acceptance + s 
                        else:
                            pass
                            
                letter = self.transitionToLetter(transition, buchi_AP)
                buchi_transitions[curr_state].append((letter, new_state))
                for id in acceptance_ids:
                    buchi_acceptances[id].append((curr_state, letter, new_state))
             
        buchi_alphabet = self.getBuchiAlphabet(buchi_AP)

        aut_run = a.accepting_run()
        
        prefix_run = []
        for i in range(len(aut_run.prefix)):
            letter = str(spot.bdd_format_formula(a.get_dict(), aut_run.prefix[i].label))
            new_letter = self.spotLetterToLetter(letter, buchi_AP)
            prefix_run.append(new_letter)

        cycle_run = []
        for i in range(len(aut_run.cycle)):
            letter = str(spot.bdd_format_formula(a.get_dict(), aut_run.cycle[i].label))
            new_letter = self.spotLetterToLetter(letter, buchi_AP)
            cycle_run.append(new_letter)

        buchi_run = {'prefix': prefix_run, 'cycle':cycle_run}
        
        return buchi_states, buchi_inits, buchi_AP, buchi_alphabet, buchi_transitions, buchi_acceptances, buchi_run

    def spotLetterToLetter(self, letter, AP_list):
        stop_letter = False
        curr_AP = ''
        no_AP = []
        yes_AP = []
        for s in letter:
            if s == ' ':
                stop_letter = not stop_letter
                if stop_letter:
                    if '!' in curr_AP:
                        no_AP.append(curr_AP[1:])
                        curr_AP = ''
                    else:
                        yes_AP.append(curr_AP)
                        curr_AP = ''

            if not stop_letter and s!=' ':
                curr_AP = curr_AP+s 
        if '!' in curr_AP:
            no_AP.append(curr_AP[1:])
            curr_AP = ''
        else:
            yes_AP.append(curr_AP)
            curr_AP = ''
        
        new_letter = []
        for ap in AP_list:
            if ap not in no_AP:
                new_letter.append(ap)

        return new_letter

    def transitionToLetter(self, transition, AP_list):
        yes_keys = []
        no_keys = []
        all_keys = []

        curr_key = ''
        for i in range(len(transition)):
            s = transition[i]
            if s != '&' and s != ' ' and s != '['  and s != ']' and s != '|':
                curr_key = curr_key + s
            else:
                if curr_key != '':
                    all_keys.append(curr_key)
                    curr_key = ''

        for key in all_keys:
            if '!' in key:
                no_keys.append(int(key[1:]))
            else:
                yes_keys.append(int(key))

        for i in range(len(list(self.env.keys()))):
            if i not in no_keys and i not in yes_keys:
                yes_keys.append(i)
        
        yes_keys.sort()
        no_keys.sort()

        letter = []
        for key in yes_keys:
            letter.append(AP_list[key])

        return letter

################################
# Discrete to hybrid automaton #
################################

class hybrid_from_ltl(buchi_from_ltl):
    def __init__(self, ltl_formula, env, model = None, workspace = None) -> None:
        super().__init__(ltl_formula, env)

        self.workspace = workspace
        self.model = model

        self.flow_cache = {}

        for prop in self.buchi_AP:
            if prop not in list(self.env.keys()):
                print('Atomic props and environment do not match!')

        self.buchi_state_init_labels = self.label_buchi_with_init()
        self.transition_reqs = self.get_transition_reqs()
        self.flows = self.omega_factest()

    def label_buchi_with_init(self):
        transitions_in = {}
        resets = {}
        for state in self.buchi_states:  
            resets[state.strip()] = []
            transitions_in[state.strip()] = []

        for state in self.buchi_states:
            for (letter, new_state) in self.buchi_transitions[state]:
                if letter not in transitions_in[new_state]:
                    transitions_in[new_state].append(letter)
        
        for state in self.buchi_states:
            buchi_letters = transitions_in[state.strip()]
            for letter in buchi_letters:
                resets[state.strip()].append(letter)

        return resets

    def get_transition_reqs(self):
        jumps = {}
        for letter in self.buchi_alphabet:
            jumps[str(letter)] = {'goal':[],'avoid':[]}
            for prop in self.buchi_AP:
                if prop in letter:
                    jumps[str(letter)]['goal'].append(self.env[prop])
                else:
                    jumps[str(letter)]['avoid'].append(self.env[prop])
        return jumps

    def get_flow(self):
        flows = {}

        cover = {}
        no_cover = {}

        new_exclusions = []

        for state in self.buchi_states:
            flows[state] = {}
            cover[state] = {}
            no_cover[state] = {}
            
            for (letter, new_state) in self.buchi_transitions[state]:
                flows[state][str(letter)] = []

                for initial_key in self.buchi_state_init_labels[state]:
                    try:
                        transition_dict = self.flow_cache[str(initial_key)][str(letter)]
                        flows[state][str(letter)] = transition_dict
                    except:
                        try:
                            cover[state][str(initial_key)][str(letter)] = []
                            no_cover[state][str(initial_key)][str(letter)] = []
                        except:
                            cover[state][str(initial_key)] = {str(letter):[]}
                            no_cover[state][str(initial_key)] = {str(letter):[]}
                        
                        init_set = []
                        for poly_key in initial_key:
                            init_set.append(self.env[poly_key])

                        goal = self.transition_reqs[str(letter)]['goal']
                        avoid = self.transition_reqs[str(letter)]['avoid']

                        for initial_poly in init_set:
                            if initial_poly in avoid:
                                avoid.remove(initial_poly)

                        if len(goal) > 0:
                            initial_poly = init_set[0] #TODO: NEED TO UPDATE THIS TO SOME INTERSECTION STUFF
                            goal_poly = goal[0] #TODO: NEED TO UPDATE THIS TO SOME INTERSECTION STUFF

                            factest = FACTEST_Z3(initial_poly, goal_poly, avoid, model=self.model, workspace=self.workspace) #TODO: NEED TO ADD IN THE MODEL STUFF
                            final_parts = factest.run()
                            del factest

                            for key in list(final_parts.keys()):
                                if final_parts[key]['xref'] == None:
                                    no_cover[state][str(initial_key)][str(letter)].append(final_parts[key]['poly'])
                                else:
                                    cover[state][str(initial_key)][str(letter)].append(final_parts[key]['poly'])
                                    transition_dict = {'init':final_parts[key]['poly'], 'xref':final_parts[key]['xref']}
                                    flows[state][str(letter)].append(transition_dict)
                            
                            if cover[state][str(initial_key)][str(letter)] != []:
                                try:
                                    self.flow_cache[str(initial_key)][str(letter)] = flows[state][str(letter)]
                                except:
                                    self.flow_cache[str(initial_key)] = {str(letter):flows[state][str(letter)]}
                            
                            else:
                                ex_next_state = ''
                                ex_curr_state = ''
                                for prop in self.buchi_AP:
                                    if prop in letter:
                                        next_str = prop
                                    else:
                                        next_str = '!'+prop

                                    if prop in initial_key:
                                        curr_str = prop
                                    else:
                                        curr_str = '!'+prop

                                    if ex_next_state == '':
                                        ex_next_state = next_str
                                    else:
                                        ex_next_state = ex_next_state + ' & ' + next_str
                                    
                                    if ex_curr_state == '':
                                        ex_curr_state = curr_str
                                    else:
                                        ex_curr_state = ex_curr_state + ' & ' + curr_str
                                
                                exclusion_constraint = 'G (('+ex_curr_state+') -> X !('+ex_next_state+'))'                               
                                new_exclusions.append(exclusion_constraint) #TODO: NEED TO COME UP WITH WAY TO GET EXCLUSION CONSTRAINTS
        
        return flows, no_cover, cover, new_exclusions
    
    def omega_factest(self):
        flows, no_cover, cover, new_exclusions = self.get_flow()

        if new_exclusions == []:
            return flows
        else:
            for exclusion_str in new_exclusions:
                self.ltl_str = self.ltl_str + ' & ' + exclusion_str            

            self.buchi_states, self.buchi_inits, self.buchi_AP, self.buchi_alphabet, self.buchi_transitions, self.buchi_acceptances, self.buchi_run = self.getBuchi()
            
            self.buchi_state_init_labels = self.label_buchi_with_init()
            self.transition_reqs = self.get_transition_reqs()
            
            flows, no_cover, cover, new_exclusions = self.get_flow()

            return flows


if __name__ == "__main__":
    import sys, os
    currFile = os.path.abspath(__file__)
    modelPath = currFile.replace('/factest/synthesis/omega_factest_z3.py', '')
    sys.path.append(modelPath)

    from demo.models.dubins_car import dubins_car
    from factest.plotting.plot_polytopes import plotPoly

    import numpy as np
    import polytope as pc
    import matplotlib.pyplot as plt

    A = np.array([[-1,0],[1,0],[0,-1],[0,1]])

    b_goal1 = np.array([-5,7,-5,7])
    b_goal2 = np.array([7,-5,7,-5])

    b_unsafe1 = np.array([11,-10,11,11])
    b_unsafe2 = np.array([-10,11,11,11])
    b_unsafe3 = np.array([11,11,-10,11])
    b_unsafe4 = np.array([11,11,11,-10])
    b_unsafe5 = np.array([11,-2,1,1])
    b_unsafe6 = np.array([-2,11,1,1])

    b_workspace = np.array([15,15,15,15])

    workspace_poly = pc.Polytope(A, b_workspace)

    E1 = pc.Polytope(A, b_goal1) # goal set 1
    E2 = pc.Polytope(A, b_goal2) # goal set 2
    E3 = pc.Polytope(A, b_unsafe5) # unsafe set 1
    E4 = pc.Polytope(A, b_unsafe6) # unsafe set 2

    env = {'E1':E1,'E2':E2,'E3':E3,'E4':E4}   

    reach_str = 'F E1 & F E2 & G (E1 -> F E2) & G (E2 -> F E1)'
    avoid_str = 'G !E3 & G !E4'
    ltl_formula = reach_str + ' & ' + avoid_str

    # myBuchi = buchi_from_ltl(ltl_formula=ltl_formula,env=env)
    # print('states', myBuchi.buchi_states)
    # print('inits', myBuchi.buchi_inits)
    # print('AP',  myBuchi.buchi_AP)
    # print('alphabet', myBuchi.buchi_alphabet)
    # print('transitions', myBuchi.buchi_transitions)
    # print('acceptances', myBuchi.buchi_acceptances)
    # print('run', myBuchi.buchi_run)

    model = dubins_car()
    myHybrid = hybrid_from_ltl(ltl_formula=ltl_formula,env=env, model=model, workspace=workspace_poly)
    print('flows', myHybrid.flows)


    E2_flows = myHybrid.flows['0']["['E2']"]
    E1_flows = myHybrid.flows['0']["['E1']"]


    fig = plt.figure()
    
    ax = fig.add_subplot(221)
    plotPoly(E1,ax,'green')
    plotPoly(E2,ax,'green')

    plotPoly(E3,ax,'red')
    plotPoly(E4,ax,'red')

    xref = E2_flows[0]['xref']
    xvals = [state[0] for state in xref]
    yvals = [state[1] for state in xref]

    ax.plot(xvals,yvals)

    ax.set_xlim(-10,10)
    ax.set_ylim(-10,10)
    ax.xaxis.set_tick_params(labelbottom=False)
    ax.yaxis.set_tick_params(labelleft=False)

    ax = fig.add_subplot(222)
    plotPoly(E1,ax,'green')
    plotPoly(E2,ax,'green')

    plotPoly(E3,ax,'red')
    plotPoly(E4,ax,'red')

    ax.set_xlim(-10,10)
    ax.set_ylim(-10,10)
    ax.xaxis.set_tick_params(labelbottom=False)
    ax.yaxis.set_tick_params(labelleft=False)

    ax = fig.add_subplot(223)
    plotPoly(E1,ax,'green')
    plotPoly(E2,ax,'green')

    plotPoly(E3,ax,'red')
    plotPoly(E4,ax,'red')

    ax.set_xlim(-10,10)
    ax.set_ylim(-10,10)
    ax.xaxis.set_tick_params(labelbottom=False)
    ax.yaxis.set_tick_params(labelleft=False)

    ax = fig.add_subplot(224)
    plotPoly(E1,ax,'green')
    plotPoly(E2,ax,'green')

    plotPoly(E3,ax,'red')
    plotPoly(E4,ax,'red')

    ax.set_xlim(-10,10)
    ax.set_ylim(-10,10)
    ax.xaxis.set_tick_params(labelbottom=False)
    ax.yaxis.set_tick_params(labelleft=False)

    plt.show()
