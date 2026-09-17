import json
from StringConstants import *


class EnvironmentParameters:
    def __init__(self, env_type, config_type, ompl_env_type, use_data_dir, yaml_filepath):
        self.config_type = env_type
        self.configuration_type = config_type
        self.ompl_environment_type = ompl_env_type
        self.use_data_dir = use_data_dir
        self.yaml_filepath = yaml_filepath

    def to_json(self):
        return self.__dict__


class AlgorithmParameter:
    def __init__(self, config_type, ompl_mp_algorithm, timeout, simplify_path,
                 simplify_path_timeout, connection_range, configuration_type):
        self.config_type = config_type
        self.ompl_mp_algorithm = ompl_mp_algorithm
        self.timeout = timeout
        self.simplify_path = simplify_path
        self.simplify_path_timeout = simplify_path_timeout
        # self.connection_range = connection_range
        self.configuration_type = configuration_type

    def to_json(self):
        return self.__dict__


class MotionPlanner:
    def __init__(self, environment_param, algorithm_param, config_type):
        self.environment_parameters = environment_param
        self.algorithm_parameters = algorithm_param
        self.config_type = config_type


class Configuration:
    def __init__(self, config_type, goal_type, state_space_type, x, y, yaw):
        self.configuration_type = config_type
        self.goal_type = goal_type
        self.state_space_type = state_space_type
        self.x = x
        self.y = y
        self.yaw = yaw

    def to_json(self):
        return self.__dict__

class Species:
    def __init__(self, name, traits, trait_rates_max, traits_dispensable, traits_depletable,
                 current_trait_functions_coeff, current_trait_rate_functions_coeff,
                 idle_current, max_battery_capacity, max_possible_current,
                 peukert_coeff, max_speed, speed_coeff,
                 bounding_radius, max_mass, value, mp_index):
        # need to account for ITAGS test case -- discard
        self.name = name
        self.traits = traits
        self.traits_max = traits
        self.trait_rates_max = trait_rates_max
        self.traits_dispensable = traits_dispensable
        self.traits_depletable = traits_depletable
        self.current_trait_functions_coeff = current_trait_functions_coeff
        self.current_trait_rate_functions_coeff = current_trait_rate_functions_coeff
        self.idle_current = idle_current
        self.max_battery_capacity = max_battery_capacity
        self.max_possible_current = max_possible_current
        self.bounding_radius = bounding_radius
        self.max_mass = max_mass
        self.speed = max_speed
        self.speed_coeff = speed_coeff
        self.peukert_coeff = peukert_coeff
        self.value = value
        self.mp_index = mp_index

    def to_json(self):
        return self.__dict__

class Robot:
    def __init__(self, name, initial_battery_level, initial_trait_levels, initial_configuration, species):
        self.name = name
        self.initial_battery_level = initial_battery_level
        self.initial_trait_levels = initial_trait_levels
        self.initial_configuration = initial_configuration
        self.species = species


class Task:
    def __init__(self, name, duration, task_reward, reward_time_decaying_rate,
                 desired_traits, minimum_trait_rates, traits_summable,
                 initial_config, terminal_config, mp_index):
        self.name = name
        self.duration = duration
        self.static_duration = duration
        self.task_reward = 0.0  # task_reward
        self.reward_time_decaying_rate = 0.0  # reward_time_decaying_rate
        self.desired_traits = desired_traits
        self.minimum_trait_rates = minimum_trait_rates
        self.traits_summable = traits_summable
        self.initial_configuration = initial_config
        self.terminal_configuration = terminal_config
        self.mp_index = mp_index


class TimePoint:
    def __init__(self, timepoint_type, task):
        self.timepoint_type = timepoint_type  # "start" or "completion"
        self.task = task

    def to_json(self):
        return self.__dict__


class AbsoluteDeadline:
    def __init__(self, timepoint, bound):
        self.deadline_type = "absolute_deadline"
        self.timepoint = timepoint
        self.bound = bound

    def to_json(self):
        return self.__dict__


class RelativeDeadline:
    def __init__(self, predecessor, successor, bound):
        self.deadline_type = "relative_deadline"
        self.predecessor = predecessor
        self.successor = successor
        self.bound = bound

    def to_json(self):
        return self.__dict__

class traitDistributionParameters:
    def __init__(self, config_type, timeout, nlp_timeout):
        self.config_type = config_type
        self.timeout = timeout
        self.nlp_timeout = nlp_timeout
        self.threads = 0

    def to_json(self):
        return self.__dict__


class dtasParameter:
    def __init__(self, config_type, has_timeout, timeout, timer_name, alpha=0.5, beta=0.5,
                 gamma=0.5, prune_before_eval=True, save_pruned_nodes=False, save_closed_nodes=False):
        self.config_type = config_type
        self.has_timeout = has_timeout
        self.timeout = timeout
        self.timer_name = timer_name
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.prune_before_eval = prune_before_eval
        self.save_pruned_nodes = save_pruned_nodes
        self.save_closed_nodes = save_closed_nodes

    def to_json(self):
        return self.__dict__

class SchedulerParameter:
    def __init__(self, config_type, scheduler_type, milp_scheduler_type, timeout, milp_timeout, threads,
                 compute_transition_duration_heuristic, use_hierarchical_objective):
        self.config_type = config_type
        self.scheduler_type = scheduler_type
        self.milp_scheduler_type = milp_scheduler_type
        self.timeout = timeout
        self.milp_timeout = milp_timeout
        self.threads = threads
        self.compute_transition_duration_heuristic = compute_transition_duration_heuristic
        self.use_hierarchical_objective = use_hierarchical_objective

    def to_json(self):
        return self.__dict__


class ProblemInputEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, Species):
            return {
                k_name: obj.name,
                # k_traits: obj.traits,
                k_traits_max: obj.traits_max,
                k_trait_rates_max: obj.trait_rates_max,
                k_traits_dispensable: obj.traits_dispensable,
                k_traits_depletable: obj.traits_depletable,
                k_current_trait_functions_coeff: obj.current_trait_functions_coeff,
                k_current_trait_rate_functions_coeff: obj.current_trait_rate_functions_coeff,
                k_max_battery_capacity: obj.max_battery_capacity,
                k_max_possible_current: obj.max_possible_current,
                k_idle_current: obj.idle_current,
                k_speed: obj.speed,
                k_speed_coeff: obj.speed_coeff,
                k_peukert_coeff: obj.peukert_coeff,
                k_max_mass: obj.max_mass,
                k_bounding_radius: obj.bounding_radius,
                k_value: obj.value,
                k_mp_index: obj.mp_index
            }
        if isinstance(obj, Robot):
            return {
                k_name: obj.name,
                k_initial_battery_level: obj.initial_battery_level,
                k_initial_trait_levels: obj.initial_trait_levels,
                k_initial_configuration: obj.initial_configuration.__dict__,
                k_species: obj.species
            }
        if isinstance(obj, Task):
            return {
                k_name: obj.name,
                k_duration: obj.duration,
                k_static_duration : obj.duration,
                k_task_reward: obj.task_reward,
                k_reward_time_decaying_rate: obj.reward_time_decaying_rate,
                k_desired_traits: obj.desired_traits,
                k_minimum_trait_rates: obj.minimum_trait_rates,
                k_traits_summable: obj.traits_summable,
                k_initial_configuration: obj.initial_configuration.__dict__,
                k_terminal_configuration: obj.terminal_configuration.__dict__,
                k_mp_index: obj.mp_index
            }
        if isinstance(obj, MotionPlanner):
            return {
                k_environment_parameters: obj.environment_parameters.__dict__,
                k_algorithm_parameters: obj.algorithm_parameters.__dict__,
                k_config_type: obj.config_type
            }
        if hasattr(obj, "to_json"):
            return obj.to_json()

        return super().default(obj)
