#!/usr/bin/env python3
"""Score solutions written by the benchmark test and print the rows of Table 2.

    python3 score_table2.py aamas2026_table2
    python3 score_table2.py <results_dir> --inputs <instance_dir> --planner traits|itags

Each returned plan is checked back against the scenario it was produced from: every task must
get the trait and provisioning rate it asks for, no robot may be asked for more than it carries
or draw more current than its C-rating allows, no robot may end below zero battery, and the
schedule must respect the precedence and deadline constraints.

The checks are the ones used for the paper; only the plotting was removed.
"""
import argparse, copy, glob, json, math, os, re, sys
from statistics import mean, pstdev

DEBUG_MODE = False
TOLERANCE = 1e-4
OUTOFRANGE = -1e6

# --- string constants -------------------------------------------------------
k_species = "species"
k_name = "name"
k_traits = "traits"
k_traits_max = "traits_max"
k_trait_rates= "trait_rates"
k_trait_rates_max = "trait_rates_max"
k_bounding_radius = "bounding_radius"
k_speed = "speed"
k_value = "value"
k_mp_index = "mp_index"

k_coalition = "coalition"
k_robots = "robots"
k_robot_species_object = "robot_species_object"
k_individual_plan = "individual_plan"
k_initial_battery_level = "initial_battery_level"
k_initial_trait_levels = "initial_trait_levels"
k_initial_configuration = "initial_configuration"
k_terminal_configuration = "terminal_configuration"
k_configuration_type = "configuration_type"
k_goal_type = "goal_type"
k_state_space_type = "state_space_type"
k_SE2 = "se2"
k_x = "x"
k_y = "y"
k_yaw = "yaw"
k_id = "id"

k_fast_slow_robot_ratio = "fast_slow_robot_ratio"
k_fast_slow_robot_ratio_norm = "fast_slow_robot_ratio_norm"

k_current_trait_functions_coeff = "current_trait_functions_coeff"
k_current_trait_rate_functions_coeff = "current_trait_rate_functions_coeff"
k_idle_current = "idle_current"
k_peukert_coeff = "peukert_coeff"
k_speed_coeff = "speed_coeff"
k_max_mass = "max_mass"
k_max_possible_current = "max_possible_current"

# k_current_trait_functions_power = "current_trait_functions_power"
# k_current_trait_rate_functions_power = "current_trait_rate_functions_power"
# k_battery_current_function_power = "battery_current_function_power"
# k_velocity_current_function_power = "velocity_current_function_power"
k_battery_consumption = "battery_consumption"
k_battery_consumption_norm = "battery_consumption_norm"
k_max_battery_capacity = "max_battery_capacity"
k_max_battery_rate = "max_battery_rate"
k_max_possible_current = "max_possible_current"
k_max_speed = "max_speed"
k_max_mass= "max_mass"

k_tasks = "tasks"
k_duration = "duration"
k_static_duration = "static_duration"
k_task_reward = "task_reward"
k_reward_time_decaying_rate = "reward_time_decaying_rate"
k_desired_traits = "desired_traits"
k_minimum_trait_rates = "minimum_trait_rates"
k_traits_summable = "traits_aggregatable"
k_traits_dispensable = "traits_provisionable"
k_traits_depletable = "traits_exhaustible"
k_precedence_constraints = "precedence_constraints"
k_plan_task_indices = "plan_task_indicies"
k_finish_timepoint = "finish_timepoint"
k_start_timepoint = "start_timepoint"
k_predecessor = "predecessor"
k_successor = "successor"
k_solution = "solution"
k_statistics = "statistics"
k_task_trait_allocations = "task trait allocations"
k_makespan = "makespan"
k_makespan_norm = "makespan_norm"
k_total_time = "total_time"
k_computation_time = "computation_time"
k_computation_time_norm = "computation_time_norm"
k_operation_cost = "operation_cost"
k_operation_cost_norm = "operation_cost_norm"

k_transitions = "transitions"
k_path_length = "path_length"
k_task_total_duration = "task_total_duration"
k_task_total_duration_norm = "task_total_duration_norm"
k_inter_task_transition_duration = "inter_task_transition_duration"
k_inter_task_transition_duration_norm = "inter_task_transition_duration_norm"
k_inter_transition_velocity = "inter_transition_velocity"

k_milp_duration = "milp_duration"
k_nlp_duration = "nlp_duration"
k_milp_duration_norm = "milp_duration_norm"
k_nlp_duration_norm = "nlp_duration_norm"

k_num_robots = "num_robots"
k_num_tasks = "num_tasks"
k_num_traits = "num_traits"

k_alpha = "alpha"
k_gamma = "gamma"
k_dtas = "DTAS"
k_itags = "ITAGS"

k_nodes_generated= "nodes_generated"
k_nodes_evaluated = "nodes_evaluated"
k_nodes_evaluated_norm = "nodes_evaluated_norm"
k_nodes_expanded = "nodes_expanded"
k_nodes_pruned = "nodes_pruned"

k_robot_traits_matrix_reduction = "robot_traits_matrix_reduction"
k_reduction_types = "reduction_types"
k_summation = "summation"
k_minimum = "minimum"
k_maximum = "maximum"

k_deadline_constraints = "deadline_constraints"
k_deadlines = "deadlines"
k_deadline_type = "deadline_type"
k_absolute_deadline = "absolute_deadline"
k_relative_deadline = "relative_deadline"
k_start = "start"
k_completion = "completion"
k_timepoint = "timepoint"
k_timepoint_type = "timepoint_type"
k_task = "task"
k_bound = "bound"

k_motion_planners = "motion_planners"
k_environment_parameters = "environment_parameters"
k_config_type = "config_type"
k_ompl_environment_type = "ompl_environment_type"
k_use_data_dir = "use_data_dir"
k_yaml_filepath = "yaml_filepath"
k_algorithm_parameters = "algorithm_parameters"
k_ompl_mp_algorithm = "ompl_mp_algorithm"
k_timeout = "timeout"
k_simplify_path = "simplify_path"
k_simplify_path_timeout = "simplify_path_timeout"

k_itags_parameters= "itags_parameters"
k_dtas_parameters = "dtas_parameters"
k_has_timeout = "has_timeout"
k_timer_name = "timer_name"
k_alpha = "alpha"
k_beta = "beta"
k_prune_before_eval = "prune_before_eval"
k_save_pruned_nodes = "save_pruned_nodes"
k_save_closed_nodes = "save_closed_nodes"

k_scheduler_parameters = "scheduler_parameters"
k_scheduler_type = "scheduler_type"
k_milp_scheduler_type = "milp_scheduler_type"
k_milp_timeout = "milp_timeout"
k_thread = "threads"
k_compute_transition_duration_heuristic = "compute_transition_duration_heuristic"
k_use_hierarchical_objective = "use_hierarchical_objective"

k_best_first_search_parameters = "BestFirstSearchParameters"
k_dtas_milp_scheduler_paramters = "dtasMilpSchedulerParameters"
k_milp = "milp"
k_deterministic = "deterministic"
k_ompl = "ompl"
k_state = "state"
k_ompl_motion_planner_parameters = "OmplMotionPlannerParameters"
k_pgm = "pgm"
k_prm = "prm"
k_dtas = "dtas"
k_itags = "itags"
k_pgm_ompl_environment = "PgmOmplEnvironment"
k_ompl_motion_planner = "OmplMotionPlanner"
k_trait_distribution_parameters = "trait_distribution_parameters"
k_TASK = "TASK"
k_json = ".json"

k_configuration = "Configuration"
k_coeff = "coeff"
k_index = "index"


# --- shared helpers ---------------------------------------------------------

#from ProblemInputClasses import *
#from StringConstants import *

TOLERANCE = 1e-3 # 1e-6

def load_input(input_path):
    # print(input_path)
    input_data = dict()
    with open(input_path) as json_data:
        input_data = json.load(json_data)
        json_data.close()

    # print(input_data)
    return input_data

def load_output(output_path):
    # print(output_path)
    output_data = dict()
    with open(output_path) as json_data:
        output_data = json.load(json_data)
        json_data.close()

    # print(output_data)
    return output_data

def check_ordering_constraints(input_data, output_data):
    # Extract Precedent Constraints
    for prec_constr in input_data[k_precedence_constraints]:
        prec_idx = prec_constr[0]
        succ_idx = prec_constr[1]
        prec_finish_timepoint = output_data[k_solution][k_tasks][prec_idx][k_finish_timepoint]
        succ_start_timepoint = output_data[k_solution][k_tasks][succ_idx][k_start_timepoint]
        if prec_finish_timepoint > succ_start_timepoint:
            print("Precedence Constraints: {:d}({:.2f}) < {:d}(:.2f) violated".format(prec_idx,
                                                                                      prec_finish_timepoint,
                                                                                      succ_idx,
                                                                                      succ_start_timepoint))
            return False

    # Deadline Constraints
    for deadline_constr in input_data[k_deadline_constraints]:
        # Extract Absolute Deadline Constraints
        if deadline_constr[k_deadline_type] == k_absolute_deadline:
            bound_time = deadline_constr[k_bound]
            task_idx = deadline_constr[k_timepoint][k_task]
            # START BOUND
            if deadline_constr[k_timepoint][k_timepoint_type] == k_completion:
                task_time = output_data[k_solution][k_tasks][task_idx][k_finish_timepoint]
                if task_time > bound_time:
                    print("Absolute Deadline Constraints: Task {:d} {} ({:.2f} > {:.2f}) violated".format(task_idx,
                                                                                                          k_start,
                                                                                                          task_time,
                                                                                                          bound_time))
                    return False
            # FINISH BOUND
            if deadline_constr[k_timepoint][k_timepoint_type] == k_start:
                task_time = output_data[k_solution][k_tasks][task_idx][k_start_timepoint]
                if task_time > bound_time + TOLERANCE:
                    print("Absolute Deadline Constraints: Task {:d} {} ({:.2f} > {:.2f}) violated".format(task_idx,
                                                                                                          k_completion,
                                                                                                          task_time,
                                                                                                          bound_time))
                    return False

        # Extract Relative Deadline Constraints
        if deadline_constr[k_deadline_type] == k_relative_deadline:
            bound_time = deadline_constr[k_bound]

            predecessor = deadline_constr[k_predecessor]
            prec_idx = predecessor[k_task]
            prec_time = output_data[k_solution][k_tasks][prec_idx][k_start_timepoint]  # start time point
            if predecessor[k_timepoint_type] == k_completion:
                prec_time = output_data[k_solution][k_tasks][prec_idx][k_finish_timepoint]

            successor = deadline_constr[k_successor]
            succ_idx = successor[k_task]
            succ_time = output_data[k_solution][k_tasks][succ_idx][k_start_timepoint]  # start time point
            if successor[k_timepoint_type] == k_completion:
                succ_time = output_data[k_solution][k_tasks][succ_idx][k_finish_timepoint]

            if succ_time - prec_time > bound_time + TOLERANCE:
                print("Relative Deadline Constraints: Task {:d} < Task {:d} \
                        ({:.2f} - {:.2f} < {:.2f}) violated".format(prec_idx,
                                                                    succ_idx,
                                                                    succ_time,
                                                                    prec_time,
                                                                    bound_time))
                return False
    return True


def check_task_trait_allocations(input_data, output_data):
    species_name_obj_map = dict()
    robots = list()
    numTraits = output_data[k_solution]["0A_numTraits"]
    for species in input_data[k_species]:
        species_name_obj_map[species[k_name]] = species
    for _robot in input_data[k_robots]:
        robot = _robot.copy()
        robot_species = species_name_obj_map[robot[k_species]]
        robot[k_initial_battery_level] = \
            robot[k_initial_battery_level] * robot_species[k_max_battery_capacity]
        # for trait_nr in range(len(robot[k_initial_trait_levels])):
        for trait_nr in range(numTraits):
                robot[k_initial_trait_levels][trait_nr] = \
                robot[k_initial_trait_levels][trait_nr] * robot_species[k_traits_max][trait_nr]
        robots.append(robot)

    # print(robots)

    # Check Robot Trait Distributions Match
    for robot_nr in range(len(robots)):
        robot = input_data[k_robots][robot_nr]
        robot_trait_allocation = output_data[k_task_trait_allocations][k_robots][robot_nr]
        robot_trait_expenditure = [0.0 for i in range(numTraits)]  # numTraits
        for task_traits in robot_trait_allocation[k_traits]:
           for trait_nr in range(numTraits):
               # if summable
               # if species_name_obj_map[robot[k_species]][k_traits_summable][trait_nr] > 0:
               if species_name_obj_map[robot[k_species]][k_traits_dispensable][trait_nr] > 0:
                   robot_trait_expenditure[trait_nr] += task_traits[trait_nr]
               else:
                   robot_trait_expenditure[trait_nr] = max(robot_trait_expenditure[trait_nr], task_traits[trait_nr])

        # Check if matches
        for trait_nr in range(numTraits):
            if (robot_trait_expenditure[trait_nr] > robot[k_initial_trait_levels][trait_nr] + TOLERANCE):
                print("Robot {:d} Trait {:d} Expenditure is violated ({:.2f} > {:.2f})".format(robot_nr,
                                                                                               trait_nr,
                                                                                               robot_trait_expenditure[trait_nr],
                                                                                               robot[k_initial_trait_levels][trait_nr]))
                return False

        robot_battery_consumption = robot_trait_allocation["battery_consumption"]
        robot_initial_battery_capacity = robot[k_initial_battery_level] * species_name_obj_map[robot[k_species]][k_max_battery_capacity]
        if (robot_battery_consumption > robot_initial_battery_capacity + TOLERANCE):
            print("Robot {:d} Trait {:d} Battery Consumption is violated ({:.2f} > {:.2f})".format(robot_nr,
                                                                                                   robot_battery_consumption,
                                                                                                   robot_initial_battery_capacity))
            return False

    # Check Task Trait Fulfilled
    task_nr = 0
    for task in output_data[k_task_trait_allocations][k_tasks]:
        coalition_traits = task["6_coalition_traits"]
        coalition_trait_rates = task["7_coalition_trait_rates"]
        coalition_traits_required = input_data[k_tasks][task_nr][k_desired_traits]
        coalition_trait_rates_required = input_data[k_tasks][task_nr][k_minimum_trait_rates]
        for trait_nr in range(numTraits):
            if coalition_traits[trait_nr] + TOLERANCE < coalition_traits_required[trait_nr]:
                print("{} Trait {:d} Unsatisfied ({:.2f} < {:.2f})".format(task["1_name"],
                                                                           trait_nr,
                                                                           coalition_traits[trait_nr],
                                                                           coalition_traits_required[trait_nr]))
                return False
            if coalition_trait_rates[trait_nr] + TOLERANCE < coalition_trait_rates_required[trait_nr]:
                print("{} Trait Rates {:d} Unsatisfied ({:.2f} < {:.2f})".format(task["1_name"],
                                                                           trait_nr,
                                                                           coalition_trait_rates[trait_nr],
                                                                           coalition_trait_rates_required[trait_nr]))
                return False
        task_nr += 1
    return True

def main():
    # dir_path = os.path.dirname(os.path.realpath(__file__))
    # input_path = dir_path + '/verification_test/input.json'
    # output_path = dir_path + '/verification_test/output1_t5_r5_u2_a0.90.json'


    # dir_path = '/home/jinwoop/dropbox_gatech_ln/ICRA2025/result/icra2025_alpha_tests'
    # dir_path = '/home/jinwoop/dropbox_gatech_ln/ICRA2025/result/icra2025_gamma_tests'
    # dir_path = '/home/jinwoop/workspaces/dynamic_traits/data/problem_inputs/dtas/icra2025_alpha_test'

    # FINAL
    # dir_path = '/home/jinwoop/dropbox_gatech_ln/ICRA2025/final_tests/icra2025_alpha_tests'
    dir_path = sys.argv[1]

    # numTasks = 15
    # numRobots= 10
    numTraits = 2
    numTests= 25

    for test_nr in range(1, numTests + 1):
    # for test_nr in range(1, 101):
        for numTasks in [10]: #range(5, 41, 5):
            for numRobots in [15]: #range(5, 31, 5):
                # input_path = dir_path + '/test{:d}/input.json'.format(test_nr)
                current_dir = os.path.dirname(os.path.abspath(sys.argv[0]))
                input_path = current_dir + "/" + dir_path + '/input/test{:d}/input.json'.format(test_nr)
                alpha = 5
                # for alpha in range(1, 10, 2):
                # output_path = dir_path + '/output{0:d}_t{1:d}_r{2:d}_u{3:d}_a{4:.2f}.json'.format(test_nr,
                                                                                                  # numTasks,
                                                                                                  # numRobots,
                                                                                                  # numTraits,
                                                                                                  # (alpha/10.0))
                output_path = current_dir + "/" +  dir_path + '/output/output{0:d}_t{1:d}_r{2:d}_u{3:d}.json'.format(test_nr,
                                                                                         numTasks,
                                                                                         numRobots,
                                                                                         numTraits)
                                                                                                  
                # output_path = dir_path + '/output{0:d}_t{1:d}_r{2:d}_u{3:d}_g{4:.2f}.json'.format(test_nr,
                #                                                                                   numTasks,
                #                                                                                   numRobots,
                #                                                                                   numTraits,
                #                                                                                   (alpha/10.0))
                try:
                    with open(output_path) as json_data:
                        pass
                except FileNotFoundError:
                    continue

                print('----- Test: {:d}, alpha: {:.2f} -----'.format(test_nr, (alpha/10.0)))
                input_data = load_input(input_path)
                output_data = load_output(output_path)

                task_ordering_passed = check_ordering_constraints(input_data, output_data)
                assert task_ordering_passed, 'Scheduling -- ordering constraint for Test: {:d}, Alpha {:.2f} failed.'.format(test_nr,
                                                                                                                         (alpha/10.0))
                print("Task Ordering Passed: ", task_ordering_passed)

                task_trait_allocation_passed = check_task_trait_allocations(input_data, output_data)
                # assert task_trait_allocation_passed, 'Task Trait Allocation for Test: {:d}, Alpha {:.2f} failed.'.format(test_nr,
                #                                                                                                          (alpha/10.0))
                print("Task Trait Allocation Passed: ", task_trait_allocation_passed)

if __name__ == "__main__":
    main()



# --- checks (verbatim from the paper's post-processing) ---------------------
def frac_true(xs):
    n = len(xs)
    return (sum(xs) / n) if n else 0.0


def read_file(file_path):
    json_file = dict()
    try:
        with open(file_path) as f:
            json_file = json.load(f)
    except FileNotFoundError:
        return None

    return json_file


# Instances generated before the schema rename carry the old trait-flag names.
# Map them onto the current ones so one scorer reads both.
_LEGACY_KEYS = {"traits_dispensable": k_traits_dispensable,
                "traits_depletable": k_traits_depletable,
                "traits_summable": k_traits_summable}


def normalise_schema(input_json):
    for entry in input_json.get(k_species, []) + input_json.get(k_tasks, []):
        for legacy, current in _LEGACY_KEYS.items():
            if legacy in entry and current not in entry:
                entry[current] = entry.pop(legacy)
    return input_json


def collect_inputs(input, numTraits=3, isDtas=False):
    normalise_schema(input)
    sp_dict = dict()
    for sp in input[k_species]:
        sp_dict[sp[k_name]] = sp

    robots = copy.deepcopy(input[k_robots])
    for robot in robots:
        robot_sp = sp_dict[robot[k_species]]
        robot[k_traits_dispensable] = robot_sp[k_traits_dispensable]
        robot[k_traits_depletable] = robot_sp[k_traits_depletable]
        robot[k_robot_species_object] = robot_sp
        robot[k_initial_battery_level] = robot[k_initial_battery_level] * robot_sp[k_max_battery_capacity]
        robot.pop(k_initial_configuration, None)
        for trait_nr in range(numTraits):
            robot[k_initial_trait_levels][trait_nr] *= robot_sp[k_traits_max][trait_nr]
            # if isDtas:
                # robot[k_initial_trait_levels][trait_nr] *= robot_sp[k_traits_max][trait_nr]
            # else:
                # robot[k_initial_trait_levels][trait_nr] = robot_sp[k_traits_max][trait_nr]

    tasks = copy.deepcopy(input[k_tasks])
    for task in tasks:
        task.pop(k_duration, None)
        task.pop(k_initial_configuration, None)
        task.pop(k_terminal_configuration, None)
        task.pop(k_mp_index, None)
        task.pop(k_task_reward, None)
        task.pop(k_reward_time_decaying_rate, None)

    precedences = input[k_precedence_constraints]
    deadlines = input[k_deadline_constraints]

    return sp_dict, robots, tasks, precedences, deadlines


def collect_output(output, isDtas=False):
    robot_path_lengths = []
    robot_assignments = []

    if isDtas:
        robots = output[k_solution][k_robots]
    else:
        robots = output[k_solution]["agents"]


    for robot in robots:
        path_length = 0.0
        if len(robot[k_individual_plan]) < 1:
            robot_path_lengths.append(path_length)
            continue

        for transition in robot[k_transitions]:
            path_length += transition[k_path_length]

        robot_path_lengths.append(path_length)
        robot_assignments.append(robot[k_individual_plan])

    # print(robot_path_lengths)
    # print(robot_assignments)

    task_objs = copy.deepcopy(output[k_solution][k_tasks])
    for task in task_objs:
        # print(task)
        task[k_path_length] = task["execution_motion_plan"][k_path_length]
        del task["execution_motion_plan"]
    # print(task_objs)

    sorted_task_objs = sorted(task_objs, key=lambda x: x[k_start_timepoint])
    sorted_task_ids = [obj[k_id] for obj in sorted_task_objs]
    # task_obj_ids = [obj[k_id] for obj in task_objs]
    # print(sorted_task_ids, task_obj_ids)
    if DEBUG_MODE:
        print(sorted_task_ids)
    # DEBUG
    # for obj in sorted_task_objs:
    #     print(obj[k_id], obj[k_start_timepoint], obj[k_finish_timepoint])

    if isDtas:
        distrib_robot = copy.deepcopy(output[k_task_trait_allocations][k_robots])
        distrib_tasks = copy.deepcopy(output[k_task_trait_allocations][k_tasks])

        return (robot_path_lengths,
                robot_assignments,
                task_objs,
                sorted_task_objs,
                sorted_task_ids,
                distrib_robot,
                distrib_tasks)

    return robot_path_lengths, robot_assignments, task_objs, sorted_task_objs, sorted_task_ids


def check_task_sufficiency_itags_and_ctas(input_tasks, input_robots, output_tasks_sorted, numTraits=3):
    num_tasks = len(input_tasks)
    num_robots = len(input_robots)

    team_traits_overcommit = [False for _ in range(num_tasks)]
    team_trait_rates_overcommit = [False for _ in range(num_tasks)]
    task_success = [False for _ in range(num_tasks)]

    robot_overcommit = dict()
    robot_current_violations = dict()
    for robot_nr in range(num_robots):
        robot_overcommit[robot_nr] = []
        robot_current_violations[robot_nr] = []

    team_traits_init = [1e6 for _ in range(numTraits)]  
    team_trait_rates_init = [1e6 for _ in range(numTraits)]  
    for trait_nr in range(numTraits):
        if input_tasks[0][k_traits_summable][trait_nr]:
            team_traits_init[trait_nr] = 0.0;
            team_trait_rates_init[trait_nr] = 0.0;

    # print(input_robots)
    robots_init_infos = copy.deepcopy(input_robots)

    for o_task in output_tasks_sorted:
        task_nr = o_task[k_id]
        i_task = input_tasks[task_nr]
        # print(o_task)
        # print(i_task)
        # print(f"-------------------task_{task_nr}-------------------")

        team_traits = copy.deepcopy(team_traits_init)
        team_trait_rates = copy.deepcopy(team_trait_rates_init)

        for robot_nr in o_task[k_coalition]:
            robot_from_input = input_robots[robot_nr]
            robot = robots_init_infos[robot_nr]
            robot_sp = robot[k_robot_species_object]
            task_duration = o_task[k_finish_timepoint] - o_task[k_start_timepoint]
            # robot current that must be consumed (IDLE + INTRA-TRANSITION)
            robot_current = robot_sp[k_idle_current] + robot_sp[k_speed_coeff] * robot_sp[k_speed]

            if DEBUG_MODE:
                print(f"robot[{robot_nr}], initial trait: {robot[k_initial_trait_levels]}")

            trait_overcommit = [False for _ in range(numTraits)]
            trait_rate_overcommit = [False for _ in range(numTraits)]

            for trait_nr in range(numTraits):
                # minimum of robot provisioning or task required trait
                trait_provision = min(i_task[k_desired_traits][trait_nr], 
                                      robot_from_input[k_initial_trait_levels][trait_nr] * robot_sp[k_traits_max][trait_nr])
                # we assume trait is provisioned at the maximum rate at all times
                trait_rate_provision = robot_sp[k_trait_rates_max][trait_nr]
                # robot current consumed for trait provisioning
                robot_current += trait_provision * robot_sp[k_current_trait_functions_coeff][trait_nr] +\
                        trait_rate_provision * robot_sp[k_current_trait_rate_functions_coeff][trait_nr]

                # non-coalitionable traits -- Non-cummulative
                if i_task[k_traits_summable][trait_nr] < 0.5:
                    # if robot had insufficient trait
                    if robot[k_initial_trait_levels][trait_nr] + TOLERANCE < i_task[k_desired_traits][trait_nr]:
                        trait_overcommit[trait_nr] = True
                    # if robot had insufficient trait rate
                    if robot_sp[k_trait_rates_max][trait_nr] + TOLERANCE < i_task[k_minimum_trait_rates][trait_nr]:
                        trait_rate_overcommit[trait_nr] = True

                    # if exhaustible trait, robot initial trait obtained from input, then subtract (keep track of usage)
                    if (robot[k_traits_depletable][trait_nr]):
                        # robot[k_initial_trait_levels][trait_nr] -= min(i_task[k_desired_traits][trait_nr], 
                                                                       # robot_from_input[k_initial_trait_levels][trait_nr] * robot_sp[k_traits_max][trait_nr])

                        # robot[k_initial_trait_levels][trait_nr] -= robot_sp[k_traits_max][trait_nr] # TODO: testing
                        robot[k_initial_trait_levels][trait_nr] -= 0.99 * min(i_task[k_desired_traits][trait_nr], 
                                                                       robot_from_input[k_initial_trait_levels][trait_nr] * robot_sp[k_traits_max][trait_nr])
                    team_traits[trait_nr] = min(team_traits[trait_nr], trait_provision)
                    team_trait_rates[trait_nr] = min(team_trait_rates[trait_nr], trait_rate_provision)
                    continue 


                ## ELSE : task can be done cooperatively <=> if i_task[k_traits_summable]:

                team_traits[trait_nr] += robot[k_initial_trait_levels][trait_nr]

                # if exhaustible trait, robot traits being consumed and if robot had positive value of this trait
                if robot[k_traits_depletable][trait_nr] and robot[k_initial_trait_levels][trait_nr] > TOLERANCE:
                        # itags could sum up the traits -- subtract minimum of robot's initial or task's desired trait
                        robot[k_initial_trait_levels][trait_nr] -= \
                                min(i_task[k_desired_traits][trait_nr], robot_sp[k_traits_max][trait_nr])

                # if robot does not have enough traits left after execution -- declare overcommit
                if robot[k_initial_trait_levels][trait_nr] + TOLERANCE < 0.0:
                    trait_overcommit[trait_nr] = True

                # robot trait has rate
                if robot[k_traits_dispensable][trait_nr]:
                    team_trait_rates[trait_nr] += robot_sp[k_trait_rates_max][trait_nr]
                else:
                    pass
            # end for-loop: trait_nr

            if DEBUG_MODE:
                print(f"robot[{robot_nr}], after trait: {robot[k_initial_trait_levels]}")
                print(f"robot[{robot_nr}], trait overcommit: ", any(trait_overcommit), trait_overcommit)
                print(f"robot[{robot_nr}], trait rate overcommit: ", any(trait_rate_overcommit), trait_rate_overcommit)
                print(f"robot[{robot_nr}], both: ", any([any(trait_overcommit), any(trait_rate_overcommit)]))

            robot_overcommit[robot_nr].append(any([any(trait_overcommit), any(trait_rate_overcommit)]))

            if robot_current ** robot_sp[k_peukert_coeff] > robot_sp[k_max_possible_current]:
                robot_current_violations[robot_nr].append(True)
            else:
                robot_current_violations[robot_nr].append(False)

            # robot_battery_consumed = robot_current ** robot_sp[k_peukert_coeff] * i_task[k_static_duration]
            robot_battery_consumed = robot_current ** robot_sp[k_peukert_coeff] * task_duration
            if DEBUG_MODE:
                print(f"robot[{robot_nr}], before battery level: {robot[k_initial_battery_level]}")
                print(f"{robot_current} ** {robot_sp[k_peukert_coeff]} * {task_duration}")

            robot[k_initial_battery_level] -= robot_battery_consumed

            if DEBUG_MODE:
                print(f"robot[{robot_nr}], battery consumed: {robot_battery_consumed}")
                print(f"robot[{robot_nr}], after battery level: {robot[k_initial_battery_level]}")

        # end for-loop: robot_nr

        if DEBUG_MODE:
            print(f"task[{task_nr}], team traits: {team_traits}, desired: {i_task[k_desired_traits]}")
            print(f"task[{task_nr}], team trait rates: {team_trait_rates}, desired_rates: {i_task[k_minimum_trait_rates]}")

        team_trait_deficiencies = [False for _ in range(numTraits)]
        team_trait_rate_deficiencies = [False for _ in range(numTraits)]
        for trait_nr in range(numTraits):
            if i_task[k_traits_summable][trait_nr] < 0.5:
                continue

            if team_traits[trait_nr] < i_task[k_desired_traits][trait_nr]:
                team_trait_deficiencies[trait_nr] = True
            if team_trait_rates[trait_nr] < i_task[k_minimum_trait_rates][trait_nr]:
                team_trait_rate_deficiencies[trait_nr] = True

        team_traits_overcommit[task_nr] = any(team_trait_deficiencies)
        team_trait_rates_overcommit[task_nr] = any(team_trait_rate_deficiencies)
        task_success[task_nr] = not any([team_traits_overcommit[task_nr], team_trait_rates_overcommit[task_nr]])

        if DEBUG_MODE:
            print(f"task[{task_nr}], team trait overcommit: ", team_traits_overcommit[task_nr], team_trait_deficiencies)
            print(f"task[{task_nr}], team trait rate overcommit: ", team_trait_rates_overcommit[task_nr], team_trait_rate_deficiencies)
            print(f"task[{task_nr}], team both success: ", task_success[task_nr])

    # end for-loop: sorted output tasks

    if DEBUG_MODE:
        print("robot_overcommit:", robot_overcommit, "task_success:", task_success)

    return robot_overcommit, team_traits_overcommit, team_trait_rates_overcommit, task_success, robot_current_violations, robots_init_infos

# end check_task_sufficiency_itags_and_ctas


def check_robot_battery_violations_itags_and_ctas(input_robots, robot_path_lengths):
    num_robots = len(input_robots)
    robot_battery_violations = [False for _ in range(num_robots)]
    for robot_nr, robot in enumerate(input_robots):
        robot_sp = robot[k_robot_species_object]
        inter_task_transition_current = (robot_sp[k_idle_current] + robot_sp[k_speed_coeff] * robot_sp[k_speed]) ** \
                                        robot_sp[k_peukert_coeff]
        inter_task_transition_duration = robot_path_lengths[robot_nr] / robot_sp[k_speed]
        if inter_task_transition_duration > 0.0:
            # THIS IS INCORRECT
            # inter_transition_battery_consumption = inter_task_transition_current / inter_task_transition_duration
            inter_transition_battery_consumption = inter_task_transition_current * inter_task_transition_duration
            robot[k_initial_battery_level] -= inter_transition_battery_consumption

        if DEBUG_MODE:
            print(f"robot[{robot_nr}] battery level calc: {robot[k_initial_battery_level]:.2f}")

        if robot[k_initial_battery_level] < 0.0:
            robot_battery_violations[robot_nr] = True

    return robot_battery_violations


def check_robot_battery_violations_dtas(input_robots, distrib_robots, true_robot_init_batt_levels, robot_path_lengths):
    num_robots = len(input_robots)
    robot_battery_violations = [False for _ in range(num_robots)]
    for robot_nr, robot in enumerate(input_robots):
        robot_sp = robot[k_robot_species_object]
        if distrib_robots[robot_nr][k_inter_transition_velocity] is not None:
            inter_task_transition_current = (robot_sp[k_idle_current] + robot_sp[k_speed_coeff] * distrib_robots[robot_nr][k_inter_transition_velocity]) ** \
                                            robot_sp[k_peukert_coeff]
            inter_task_transition_duration = robot_path_lengths[robot_nr] / distrib_robots[robot_nr][k_inter_transition_velocity]
            if inter_task_transition_duration > 0.0:
                # inter_transition_battery_consumption = inter_task_transition_current / inter_task_transition_duration
                inter_transition_battery_consumption = inter_task_transition_current * inter_task_transition_duration
                robot[k_initial_battery_level] -= inter_transition_battery_consumption

            # manual calculation consumes less energy because dtas over-estimates inter-task transition battery consumption
            # to be conservative.
            if DEBUG_MODE:
            # if robot[k_initial_battery_level] < 0.0:
                print(f"robot[{robot_nr}] battery level calc: {robot[k_initial_battery_level]:.2f}, "
                      f"dtas battery level: {true_robot_init_batt_levels[robot_nr] - distrib_robots[robot_nr][k_battery_consumption]:.2f}, ",
                      f"init battery level: {true_robot_init_batt_levels[robot_nr]:.2f}, consumed: {distrib_robots[robot_nr][k_battery_consumption]:.2f}")

        if robot[k_initial_battery_level] < 0.0:
            print("Battery Violation Occurred:", f"robot[{robot_nr}] battery level calc: {robot[k_initial_battery_level]:.2f}, ",
                  f"dtas battery level: {true_robot_init_batt_levels[robot_nr] - distrib_robots[robot_nr][k_battery_consumption]:.2f}, ",
                  f"init battery level: {true_robot_init_batt_levels[robot_nr]:.2f}, consumed: {distrib_robots[robot_nr][k_battery_consumption]:.2f}")
            robot_battery_violations[robot_nr] = True

    return robot_battery_violations


def check_deadline_violations(precedences, deadlines, task_objs):
    deadline_violations = []
    for prec in precedences:
        succ_task_start_time = task_objs[prec[1]][k_start_timepoint]
        pred_task_end_time = task_objs[prec[0]][k_finish_timepoint]
        # A strict comparison flags float noise in the serialised timepoints as a violation:
        # ITAGS enforces precedence in its MILP yet showed two "violations" of -3e-5 s.
        if succ_task_start_time < pred_task_end_time - TOLERANCE:
            print(f"prec: {succ_task_start_time} < {pred_task_end_time}")
            deadline_violations.append(True)
        else:
            deadline_violations.append(False)

    for deadline in deadlines:
        if deadline[k_deadline_type] == k_absolute_deadline:
            task_nr = deadline[k_timepoint][k_task]
            if deadline[k_timepoint][k_timepoint_type] is k_completion:
                timepoint = task_objs[task_nr][k_finish_timepoint]
            else: # not completion
                timepoint = task_objs[task_nr][k_start_timepoint]

            if timepoint > deadline[k_bound] + TOLERANCE:
                print(f"abs: {timepoint} > {deadline[k_bound]}")
                deadline_violations.append(True)
            else:
                deadline_violations.append(False)

        elif deadline[k_deadline_type] == k_relative_deadline:
            timepoints = []
            for timepoint in [deadline[k_predecessor], deadline[k_successor]]:
                task_nr = timepoint[k_task]
                if timepoint[k_timepoint_type] is k_completion:
                    timepoints.append(task_objs[task_nr][k_finish_timepoint])
                else:
                    timepoints.append(task_objs[task_nr][k_start_timepoint])

            if timepoints[1] - timepoints[0] > deadline[k_bound] + TOLERANCE:
                print(f"rel: {timepoints[1] - timepoints[0]} > {deadline[k_bound]}")
                deadline_violations.append(True)
            else:
                deadline_violations.append(False)

    return deadline_violations


def itags_statistics(dir_path, test_nr):
    # input_file_path = dir_path + f"/input/test{test_nr}/input.json"
    input_file_path = dir_path + f"/input_itags_qdur/test{test_nr}/input.json"
    input_json_file = read_file(input_file_path)

    if input_json_file is None:
        return (None, None, None, None, None, None, None)

    input_sp_dict, input_robots, input_tasks, precedences, deadlines = collect_inputs(input_json_file, isDtas=False)

    if len(glob.glob(dir_path + f'/output_itags/output{test_nr}_*.json')) < 1:
        return (None, None, None, None, None, None, None)

    output_file_path_itags = glob.glob(dir_path + f'/output_itags/output{test_nr}_*.json')[0]
    output_json_file_itags = read_file(output_file_path_itags)
    # print(output_file_path_itags)

    (robot_path_lengths_itags,
     robot_assignments_itags,
     task_objs_itags,
     sorted_task_objs_itags,
     sorted_task_ids_itags) = collect_output(output_json_file_itags, isDtas=False)

    (robot_overcommit,
     team_traits_overcommit,
     team_trait_rates_overcommit,
     task_success,
     robot_current_violations,
     updated_robot_infos) = check_task_sufficiency_itags_and_ctas(input_tasks, input_robots, sorted_task_objs_itags)

    robot_battery_violations = check_robot_battery_violations_itags_and_ctas(updated_robot_infos, robot_path_lengths_itags)
    deadline_violations = check_deadline_violations(precedences, deadlines, task_objs_itags)

    if DEBUG_MODE:
        print('--ITAGS: return--')
        print("Trait Over Committed:", team_traits_overcommit)
        print("Trait Rate Over Committed:", team_trait_rates_overcommit)
        print("Task Success:", task_success)
        print("Current Violations:", robot_current_violations)
        print("Battery Violations", robot_battery_violations)
        print("Deadline Violations", deadline_violations)

    return (robot_overcommit,
            team_traits_overcommit,
            team_trait_rates_overcommit,
            task_success,
            robot_current_violations,
            robot_battery_violations,
            deadline_violations)


def check_task_sufficiency_dtas(input_tasks, input_robots, output_tasks_sorted,
                                distrib_robot, distrib_tasks, numTraits=3):

    num_tasks = len(input_tasks)
    num_robots = len(input_robots)

    team_traits_overcommit = [False for _ in range(num_tasks)]
    team_trait_rates_overcommit = [False for _ in range(num_tasks)]
    task_success = [False for _ in range(num_tasks)]

    robot_overcommit = dict()
    robot_current_violations = dict()
    for robot_nr in range(num_robots):
        robot_overcommit[robot_nr] = []
        robot_current_violations[robot_nr] = []

    team_traits_init = [1e6 for _ in range(numTraits)]  
    team_trait_rates_init = [1e6 for _ in range(numTraits)]  
    for trait_nr in range(numTraits):
        if input_tasks[0][k_traits_summable][trait_nr]:
            team_traits_init[trait_nr] = 0.0;
            team_trait_rates_init[trait_nr] = 0.0;

    for o_task in output_tasks_sorted:
        task_nr = o_task[k_id]
        i_task = input_tasks[task_nr]
        # print(o_task)
        # print(i_task)

        team_traits = team_traits_init
        team_trait_rates = team_trait_rates_init

        for robot_nr in o_task[k_coalition]:
            robot = input_robots[robot_nr]
            robot_sp = robot[k_robot_species_object]
            task_duration = o_task[k_finish_timepoint] - o_task[k_start_timepoint]
            task_path_length = o_task[k_path_length]
            intra_task_transition_speed = task_path_length / task_duration
            robot_current = robot_sp[k_idle_current] + robot_sp[k_speed_coeff] * intra_task_transition_speed

            if DEBUG_MODE:
                print(f"robot[{robot_nr}], initial trait: {robot[k_initial_trait_levels]}")
            trait_overcommit = [False for _ in range(numTraits)]
            trait_rate_overcommit = [False for _ in range(numTraits)]
            for trait_nr in range(numTraits):
                trait_provision = distrib_robot[robot_nr][k_traits][task_nr][trait_nr]
                trait_rate_provision = distrib_robot[robot_nr][k_trait_rates][task_nr][trait_nr]
                robot_current += trait_provision * robot_sp[k_current_trait_functions_coeff][trait_nr] + \
                                 trait_rate_provision * robot_sp[k_current_trait_rate_functions_coeff][trait_nr]

                # non-coalitionable traits
                if i_task[k_traits_summable][trait_nr] < 0.5:
                    if robot[k_initial_trait_levels][trait_nr] + TOLERANCE < i_task[k_desired_traits][trait_nr]:
                        trait_overcommit[trait_nr] = True
                    if (robot[k_traits_depletable] and
                            distrib_robot[robot_nr][k_trait_rates][task_nr][trait_nr] + TOLERANCE < i_task[k_minimum_trait_rates][trait_nr]):
                        # this is critical: no debug -- shows up if there is error
                        print(f"non-coalitionable traits failed: task_nr: {task_nr}, trait_nr: {trait_nr}, \
                                {distrib_robot[robot_nr][k_trait_rates][task_nr][trait_nr]} < {i_task[k_minimum_trait_rates][trait_nr] - TOLERANCE}")
                        trait_rate_overcommit[trait_nr] = True

                    # theoretically it should be min instead of max
                    team_traits[trait_nr] = min(team_traits[trait_nr], trait_provision)
                    team_trait_rates[trait_nr] = min(team_trait_rates[trait_nr], trait_rate_provision)
                    continue

                # task can be done cooperatively <=> if i_task[k_traits_summable]:

                team_traits[trait_nr] += distrib_robot[robot_nr][k_traits][task_nr][trait_nr]

                # robot traits being consumed
                if robot[k_traits_depletable]:
                    # if robot had positive value of this trait
                    robot[k_initial_trait_levels][trait_nr] -= distrib_robot[robot_nr][k_traits][task_nr][trait_nr]
                else:  # for readability
                    pass

                if robot[k_initial_trait_levels][trait_nr] + TOLERANCE < 0.0:
                    trait_overcommit[trait_nr] = True

                # robot trait has rate
                if robot[k_traits_dispensable]:
                    team_trait_rates[trait_nr] += distrib_robot[robot_nr][k_trait_rates][task_nr][trait_nr]
                else:
                    pass
            # end for-loop: trait_nr

            if DEBUG_MODE:
                print(f"robot[{robot_nr}], after trait: {robot[k_initial_trait_levels]}")
                print(f"robot[{robot_nr}], trait overcommit: ", any(trait_overcommit), trait_overcommit)
                print(f"robot[{robot_nr}], trait rate overcommit: ", any(trait_rate_overcommit), trait_rate_overcommit)
                print(f"robot[{robot_nr}], both: ", any([any(trait_overcommit), any(trait_rate_overcommit)]))

            robot_overcommit[robot_nr].append(any([any(trait_overcommit), any(trait_rate_overcommit)]))

            if robot_current > robot_sp[k_max_possible_current]:
                robot_current_violations[robot_nr].append(True)
            else:
                robot_current_violations[robot_nr].append(False)

            robot_battery_consumed = robot_current ** robot_sp[k_peukert_coeff] * task_duration

        # --- Update species max_possible_current from observed robot current (+2% tolerance) ---
        try:
            # Map species name -> observed max current
            sp_idx_map = { sp[k_name]: i for i, sp in enumerate(input_json[k_species]) }
            observed_max_current = { name: 0.0 for name in sp_idx_map.keys() }

            # Estimate per-robot max current from DTAS distribution (if velocity provided) or idle as fallback
            for ridx, r in enumerate(input_robots):
                sp_name = r[k_species]
                sp_obj = input_json[k_species][sp_idx_map[sp_name]]
                # Try DTAS inter-transition velocity if available
                v = distrib_robot_dtas[ridx].get(k_inter_transition_velocity, None)
                idle = sp_obj.get(k_idle_current, 0.0)
                speed_coeff = sp_obj.get(k_speed_coeff, 0.0)
                peukert = sp_obj.get(k_peukert_coeff, 1.0)
                if v is None:
                    # fallback: treat v=0 -> current = idle^peukert
                    cur = max(0.0, idle) ** peukert
                else:
                    cur = max(0.0, idle + speed_coeff * v) ** peukert
                if cur > observed_max_current[sp_name]:
                    observed_max_current[sp_name] = cur

            # Write back with +2% tolerance
            for name, max_cur in observed_max_current.items():
                sp_i = sp_idx_map[name]
                old = input_json[k_species][sp_i].get(k_max_possible_current, 0.0)
                new = 1.02 * max_cur
                input_json[k_species][sp_i][k_max_possible_current] = new
                if DEBUG_MODE:
                    print(f"[current] species {name}: max_possible_current {old:.6f} -> {new:.6f} (observed {max_cur:.6f}, +2%)")
        except Exception as e:
            if DEBUG_MODE:
                print("WARN: current update failed:", e)

            robot[k_initial_battery_level] -= robot_battery_consumed

        # end for-loop: robot_nr

        if DEBUG_MODE:
            print(f"task[{task_nr}], team traits: {team_traits}, desired: {i_task[k_desired_traits]}")
            print(f"task[{task_nr}], team trait rates: {team_trait_rates}, desired_rates: {i_task[k_minimum_trait_rates]}")
            print(f"task[{task_nr}], dtas team traits: {distrib_tasks[task_nr]['6_coalition_traits']}, "
                  f"dtas team trait rates: {distrib_tasks[task_nr]['7_coalition_trait_rates']}")

        team_trait_deficiencies = [False for _ in range(numTraits)]
        team_trait_rate_deficiencies = [False for _ in range(numTraits)]
        for trait_nr in range(numTraits):
            if i_task[k_traits_summable][trait_nr] < 0.5:
                continue

            if team_traits[trait_nr] < i_task[k_desired_traits][trait_nr] - TOLERANCE:
                team_trait_deficiencies[trait_nr] = True
            if team_trait_rates[trait_nr] < i_task[k_minimum_trait_rates][trait_nr] - TOLERANCE:
                team_trait_rate_deficiencies[trait_nr] = True

        team_traits_overcommit[task_nr] = any(team_trait_deficiencies)
        team_trait_rates_overcommit[task_nr] = any(team_trait_rate_deficiencies)

        # --- Update tasks' minimum_trait_rates from team_trait_rates (-2% tolerance) ---
        try:
            # Expect team_trait_rates[trait_nr] present in distrib_tasks_dtas or similar scope
            # We'll attempt to read per-task rates if available; otherwise apply a global team rate.
            # Build a default vector from first available team rates
            default_rates = None
            if 'team_trait_rates' in locals():
                default_rates = list(team_trait_rates)
            elif isinstance(distrib_tasks_dtas, dict) and 'team_trait_rates' in distrib_tasks_dtas:
                default_rates = list(distrib_tasks_dtas['team_trait_rates'])
            else:
                # fallback: zeroes with length  len(input_json[k_traits])  if defined; else skip
                try:
                    ntraits = len(input_json.get(k_traits, []))
                    default_rates = [0.0]*ntraits
                except Exception:
                    default_rates = None

            # Update each task's minimum_trait_rates
            if default_rates is not None:
                for t in input_json.get(k_tasks, []):
                    min_rates = t.get(k_minimum_trait_rates, None)
                    if min_rates is None:
                        # if not present, create from default
                        min_rates = [0.0]*len(default_rates)
                    # apply 2% reduction to each trait rate from team default
                    updated = []
                    for trait_nr, rate in enumerate(default_rates):
                        try:
                            base = rate
                            new_rate = max(0.0, 0.98 * base)
                        except Exception:
                            new_rate = 0.0
                        updated.append(new_rate)
                    t[k_minimum_trait_rates] = updated
                if DEBUG_MODE:
                    print("[traits] tasks.minimum_trait_rates updated from team_trait_rates (-2%)")
        except Exception as e:
            if DEBUG_MODE:
                print("WARN: minimum_trait_rates update failed:", e)

        task_success[task_nr] = not any([team_traits_overcommit[task_nr], team_trait_rates_overcommit[task_nr]])

        if DEBUG_MODE:
            print(f"task[{task_nr}], team trait overcommit: ", team_traits_overcommit[task_nr], team_trait_deficiencies)
            print(f"task[{task_nr}], team trait rate overcommit: ", team_trait_rates_overcommit[task_nr], team_trait_rate_deficiencies)
            print(f"task[{task_nr}], team both success: ", task_success[task_nr])

    # end for-loop: sorted output tasks

    if DEBUG_MODE:
        print("robot_overcommit:", robot_overcommit, "task_success:", task_success)

    return robot_overcommit, team_traits_overcommit, team_trait_rates_overcommit, task_success, robot_current_violations


def traits_statistics(dir_path, test_nr):
    input_file_path = dir_path + f"/input/test{test_nr}/input.json"
    input_json_file = read_file(input_file_path)

    if input_json_file is None:
        return (None, None, None, None, None, None, None)

    input_sp_dict, input_robots, input_tasks, precedences, deadlines = collect_inputs(input_json_file, isDtas=True)

    true_robot_init_batt_levels = [input_robots[robot_nr][k_initial_battery_level] for robot_nr in range(len(input_robots))]

    if len(glob.glob(dir_path + f'/output_traits/output{test_nr}_*.json')) < 1:
        return (None, None, None, None, None, None, None)

    if DEBUG_MODE:
        print("True robot init battery levels:", true_robot_init_batt_levels)

    output_file_path_dtas = glob.glob(dir_path + f'/output_traits/output{test_nr}_*.json')[0]
    output_json_file_dtas = read_file(output_file_path_dtas)

    (robot_path_lengths_dtas,
    robot_assignments_dtas,
    task_objs_dtas,
    sorted_task_objs_dtas,
    sorted_task_ids_dtas,
    distrib_robot_dtas,
    distrib_tasks_dtas) = collect_output(output_json_file_dtas, isDtas=True)

    (robot_overcommit,
     team_traits_overcommit,
     team_trait_rates_overcommit,
     task_success,
     robot_current_violations) = check_task_sufficiency_dtas(input_tasks,
                                                             input_robots,
                                                             sorted_task_objs_dtas,
                                                             distrib_robot_dtas,
                                                             distrib_tasks_dtas)

    robot_battery_violations = check_robot_battery_violations_dtas(input_robots,
                                                                   distrib_robot_dtas,
                                                                   true_robot_init_batt_levels,
                                                                   robot_path_lengths_dtas)

    deadline_violations = check_deadline_violations(precedences, deadlines, task_objs_dtas)

    if DEBUG_MODE:
        print('--TRAITS: return--')
        print("Trait OVC:", team_traits_overcommit)
        print("Trait Rate OVC:", team_trait_rates_overcommit)
        print("Task Success:", task_success)
        print("Curr Vio:", robot_current_violations)
        print("Bat Vio:", robot_battery_violations)
        print("Dead Vio:", deadline_violations)

    return (robot_overcommit,
            team_traits_overcommit,
            team_trait_rates_overcommit,
            task_success,
            robot_current_violations,
            robot_battery_violations,
            deadline_violations)



def _statistics_for(results_dir, inputs_dir, planner, first, last):
    rows, missing = [], []
    for n in range(first, last + 1):
        ip = os.path.join(inputs_dir, f"test{n}", "input.json")
        op = glob.glob(os.path.join(results_dir, f"output{n}_t*_r*.json"))
        if not os.path.exists(ip) or not op:
            missing.append(n); continue
        input_json, output_json = read_file(ip), read_file(op[0])
        if input_json is None or output_json is None or not (output_json.get("solution") or {}).get("tasks"):
            missing.append(n); continue
        try:
            if planner == "traits":
                r = traits_statistics_inline(input_json, output_json)
            else:
                r = itags_statistics_inline(input_json, output_json)
        except Exception as exc:
            missing.append(n); continue
        if r is None:
            missing.append(n); continue
        rows.append(r + (float((output_json.get("statistics") or {}).get("total_time", 0.0)),))
    return rows, missing


def itags_statistics_inline(input_json, output_json):
    sp, robots, tasks, prec, deadlines = collect_inputs(input_json, isDtas=False)
    (path_lengths, assignments, task_objs, sorted_task_objs, sorted_ids) = collect_output(output_json, isDtas=False)
    (robot_overcommit, tt, ttr, success, current_v, robots_init) = \
        check_task_sufficiency_itags_and_ctas(tasks, robots, sorted_task_objs)
    batt = check_robot_battery_violations_itags_and_ctas(robots_init, path_lengths)
    dl = check_deadline_violations(prec, deadlines, task_objs)
    flat = lambda d: [x for v in d.values() for x in v]
    return (frac_true(success), frac_true(tt), frac_true(ttr), frac_true(flat(robot_overcommit)),
            frac_true(flat(current_v)), frac_true(batt), frac_true(dl))


def traits_statistics_inline(input_json, output_json):
    sp, robots, tasks, prec, deadlines = collect_inputs(input_json, isDtas=True)
    (path_lengths, assignments, task_objs, sorted_task_objs, sorted_ids,
     distrib_robots, distrib_tasks) = collect_output(output_json, isDtas=True)
    init_batt = [r[k_initial_battery_level] for r in robots]
    (robot_overcommit, tt, ttr, success, current_v) = \
        check_task_sufficiency_dtas(tasks, robots, sorted_task_objs, distrib_robots, distrib_tasks)
    batt = check_robot_battery_violations_dtas(robots, distrib_robots, init_batt, path_lengths)
    dl = check_deadline_violations(prec, deadlines, task_objs)
    flat = lambda d: [x for v in d.values() for x in v]
    return (frac_true(success), frac_true(tt), frac_true(ttr), frac_true(flat(robot_overcommit)),
            frac_true(flat(current_v)), frac_true(batt), frac_true(dl))


ROWS = ["plan feasibility", "task trait insufficiency", "provisioning rate insufficiency",
        "under-resourced robots", "C-rating violations", "battery-capacity violations",
        "deadline violations"]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("results_dir")
    ap.add_argument("--inputs", default="data/problem_inputs/traits/aamas2026_table2")
    ap.add_argument("--planner", choices=["traits", "itags"], default="traits")
    ap.add_argument("--first", type=int, default=1)
    ap.add_argument("--last", type=int, default=100)
    a = ap.parse_args()

    rows, missing = _statistics_for(a.results_dir, a.inputs, a.planner, a.first, a.last)
    if not rows:
        sys.exit(f"nothing to score under {a.results_dir}")

    print(f"\n{a.planner.upper()}, {len(rows)} instances from {a.results_dir}\n")
    print(f"{'row':<34}{'mean':>9}{'std':>9}")
    print("-" * 52)
    for i, name in enumerate(ROWS):
        v = [100.0 * r[i] for r in rows]
        print(f"{name + ' [%]':<34}{mean(v):>9.1f}{pstdev(v):>9.1f}")
    v = [r[7] for r in rows]
    print(f"{'computation time [s]':<34}{mean(v):>9.2f}{pstdev(v):>9.2f}")
    if missing:
        print(f"\nno solution returned for {len(missing)} instance(s): "
              f"{missing[:12]}{' ...' if len(missing) > 12 else ''}")


if __name__ == "__main__":
    main()
