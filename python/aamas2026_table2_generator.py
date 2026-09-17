"""Generate a fresh set of 100 warehouse instances for the TRAITS benchmark.

Writes data/problem_inputs/traits/<--out>/test1..test100, four scenarios for each
(tasks, robots) pair drawn from {5, 10, 15, 20, 25}. Run them with

  TRAITS_TABLE2_DIR=/problem_inputs/traits/<--out> ./unittests \
      --gtest_filter=TRAITS.aamas2026_table2

These are new scenarios, not a regeneration of the shipped set: use them to try the
planner on fresh problems. The numbers in Table 2 come from the fixed set in
data/problem_inputs/traits/aamas2026_table2.

Tasks are created first and their trait demands divided across the species, rather
than species first. Both guarantee a feasible allocation exists, but doing it in this
order means adding robots makes a scenario easier, which is the intuitive direction.
"""
import argparse
import random
import time
import sys
import os
import re
import copy
import numpy as np
# Local
from ProblemInputClasses import *
from StringConstants import *

# Seeded deterministically so that a generated instance set can be reproduced.
# Both RNGs must be seeded: most of the sampling below goes through np.random.
DEFAULT_SEED = 20260214
DEFAULT_OUT = "aamas2026_table2_regenerated"


def parseArgs():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seed", default=str(DEFAULT_SEED),
                    help='RNG seed, or "time" to seed from the clock')
    ap.add_argument("--out", default=DEFAULT_OUT,
                    help="directory name under data/problem_inputs/traits")
    return ap.parse_args()


# The solver reads a slightly different vocabulary than this generator's internal one.
# Emit the schema that traits/problem_inputs expects.
_SPECIES_RENAME = {"traits_dispensable": "traits_provisionable",
                   "traits_depletable": "traits_exhaustible"}
_SPECIES_DROP = ("max_mass", "value")
_TASK_RENAME = {"traits_summable": "traits_aggregatable"}
_TASK_DROP = ("task_reward", "reward_time_decaying_rate")


def _rename(entry, mapping, drop):
    return {mapping.get(k, k): v for k, v in entry.items() if k not in drop}


def toSolverSchema(data):
    out = dict()
    out["species"] = [_rename(s, _SPECIES_RENAME, _SPECIES_DROP) for s in data["species"]]
    out["robots"] = data["robots"]
    out["tasks"] = [_rename(t, _TASK_RENAME, _TASK_DROP) for t in data["tasks"]]
    out["plan_task_indicies"] = data["plan_task_indicies"]
    out["precedence_constraints"] = data["precedence_constraints"]
    out["deadline_constraints"] = data["deadline_constraints"]
    out["motion_planners"] = data["motion_planners"]

    traits_parameters = dict(data["dtas_parameters"])
    traits_parameters.pop("beta", None)
    traits_parameters["timer_name"] = "traits"
    out["traits_parameters"] = traits_parameters

    out["trait_distribution_parameters"] = data["trait_distribution_parameters"]

    scheduler_parameters = dict(data["scheduler_parameters"])
    scheduler_parameters["config_type"] = "TraitsMilpSchedulerParameters"
    out["scheduler_parameters"] = scheduler_parameters
    return out

# Classes
class TimePoint:
    def __init__(self, start_or_completion, task_nr):
        self.timepoint_type = start_or_completion
        self.task = task_nr

    def to_json(self):
        return self.__dict__


class DeadlineBase:
    def __init__(self, deadline_type, bound):
        self.deadline_type = deadline_type
        self.bound = bound

    def to_json(self):
        return self.__dict__


class AbsoluteDeadline(DeadlineBase):
    def __init__(self, time_point_obj, bound):
        DeadlineBase.__init__(self, k_absolute_deadline, bound)
        self.timepoint = time_point_obj

    def to_json(self):
        return self.__dict__

class RelativeDeadline(DeadlineBase):
    def __init__(self, predecessor, successor, bound):
        DeadlineBase.__init__(self, k_relative_deadline, bound)
        self.predecessor = predecessor
        self.successor = successor

    def to_json(self):
        return self.__dict__

NLP_TIMEOUT = 1.0
# Search budget written into every instance. Generous on purpose: an instance that runs
# long is a slow machine, not an unsolvable scenario.
SEARCH_BUDGET = 800.0

# Constants
MIN_STATIC_DURATION = 1
MAX_STATIC_DURATION = 1.001
NUM_TRAILING_DIGITS = 3
TASK_POS_UNCERTAINTY = 2.0  # 5.0  # 1.5 to account for near wall cases -- 2.5 doesn't work

# Dispense: 1: Husky, 2: Jackal, No dispense: 3
NUM_TRAITS = 3
PROVISIONABLE_INDEX = [1, 1, 0]
EXHAUSTIBLE_INDEX = [1, 1, 1]
SUMMABLE_INDEX = [1, 0, 1]

# Effective Battery Capacity = Battery Capacity / CAPACITY_FACTOR (the smaller, larger the capacity)
# CAPACITY_FACTOR = 0.001  # EASY -- will tighten later by postprocess
CAPACITY_FACTOR = 0.1  # EASY
# CAPACITY_FACTOR = 0.5  # EASY -- sometimes still not feasible
# CAPACITY_FACTOR = 1.0  # MEDIUM
# CAPACITY_FACTOR = 2.0  # HARD
# CAPACITY_FACTOR = 5.0  # VERY HARD

## RANDOM GENERATION BOUNDS
###### CHANGING TO RELATIVE -- product of numTasks**2/numRobots
REL_DEADLINE_LB = 4 * 40
REL_DEADLINE_UB = 20 * 40
ABS_DEADLINE_LB = 6 * 40
ABS_DEADLINE_UB = 18 * 40

# # -- WORKING RANGES --
# still large
MAX_TRAIT = 5
MAX_TRAIT_RATES = 10
# MAX_TRAIT = 3
# MAX_TRAIT_RATES = 5

MIN_BATTERY_LEVEL = 0.2
MIN_TRAIT_LEVEL = 0.2


ROBOT_TRAITS_SUM = [0.0 for i in range(NUM_TRAITS)]
ROBOT_TRAITS_MIN = [MAX_TRAIT + 1 for i in range(NUM_TRAITS)]
ROBOT_TRAIT_RATES_MIN = [MAX_TRAIT_RATES + 1 for i in range(NUM_TRAITS)]
ROBOT_TRAIT_RATES_MAX = [0.0 for i in range(NUM_TRAITS)]

TASK_TRAITS_SUM = [0.0 for i in range(NUM_TRAITS)]
TASK_TRAITS_MIN = [MAX_TRAIT + 1 for i in range(NUM_TRAITS)]
TASK_TRAIT_RATES_MIN = [MAX_TRAIT_RATES + 1 for i in range(NUM_TRAITS)]
TASK_TRAIT_RATES_MAX = [0.0 for i in range(NUM_TRAITS)]

NUM_HUSKY_SPECIES_VARIANT = -1
NUM_JACKAL_SPECIES_VARIANT = -1



I_AVG = 4.2 # A for both Husky and Jackal  # EASY
# I_AVG = 20.0 # To induce some battery constr violations  # MEDIUM
# I_AVG = 40.0 # To induce some battery constr violations  # HARD
# I_AVG = 50.0 # To induce some battery constr violations  # VERY HARD

# SPECIES_NAME_IDX_MAP = {"Husky_Slow_Rate_Large_Capacity": 0,
#                         "Husky": 1,
#                         "Husky_Fast_Rate_Small_Capacity": 2,
#                         "Jackal_Slow_Rate_Large_Capacity": 3,
#                         "Jackal": 4,
#                         "Jackal_Fast_Rate_Small_Capacity": 5,
#                         "Impotent": 6}

SPECIES_NAME_IDX_MAP = {"Husky_Slow_Rate_Large_Capacity": 0,
                        "Husky": 1,
                        "Jackal_Slow_Rate_Large_Capacity": 2,
                        "Jackal": 3,
                        "Impotent": 4}


def resetSum():
    global ROBOT_TRAITS_SUM
    global ROBOT_TRAITS_MIN
    global ROBOT_TRAIT_RATES_MIN
    global ROBOT_TRAIT_RATES_MAX
    global TASK_TRAITS_SUM
    global TASK_TRAITS_MIN
    global TASK_TRAIT_RATES_MIN
    global TASK_TRAIT_RATES_MAX

    ROBOT_TRAITS_SUM = [0.0 for i in range(NUM_TRAITS)]
    ROBOT_TRAITS_MIN = [MAX_TRAIT + 1 for i in range(NUM_TRAITS)]
    ROBOT_TRAIT_RATES_MIN = [MAX_TRAIT_RATES + 1 for i in range(NUM_TRAITS)]
    ROBOT_TRAIT_RATES_MAX = [0.0 for i in range(NUM_TRAITS)]

    TASK_TRAITS_SUM = [0.0 for i in range(NUM_TRAITS)]
    TASK_TRAITS_MIN = [MAX_TRAIT + 1 for i in range(NUM_TRAITS)]
    TASK_TRAIT_RATES_MIN = [MAX_TRAIT_RATES + 1 for i in range(NUM_TRAITS)]
    TASK_TRAIT_RATES_MAX = [0.0 for i in range(NUM_TRAITS)]


def generateHalfDispensable():
    while True:
        rv = [random.randint(0, 1) for _ in range(NUM_TRAITS)]
        if sum(rv[:-1]) == NUM_TRAITS // 2:
            return rv 
        # if sum(rv) == NUM_TRAITS // 2:
            # return rv 


def generateSingleMinimumTrait():
    while True:
        rv = np.random.choice([0, 1], size=NUM_TRAITS).tolist()
        if sum(rv) == NUM_TRAITS - 1:
            return rv 



def openConfigurations(x_open, x_partial_open, y_open, y_partial_open):
    x_vals = x_open + x_partial_open
    y_vals = y_open + y_partial_open
    loc = set()
    for x in x_vals:
        for y in y_vals:
            if y in y_partial_open and x in x_partial_open:
                continue
            loc.add((x, y))

    print(len(loc), loc)
    return loc


def warehouse4by3FreeConfigurations(loc_tasks, loc_robots=None):
    x_vals = [20, 60, 100, 140, 180, 220, 260, 300, 340, 380, 420, 460, 500, 540, 580, 620]
    y_vals = [20, 50, 80, 110, 140, 170, 200, 230, 260, 290, 320, 350, 380]
    for x in x_vals:
        for y in y_vals:
            if y in [50, 80, 140, 170, 230, 260, 320, 350] \
                    and x in [60, 100, 140, 180, 260, 300, 340, 380, 460, 500, 540, 580]:
                continue
            loc_tasks.add((x, y))

    loc_robots = loc_tasks.copy() if loc_robots is None else loc_robots
    yaml_paths = ["/geometric_planning/maps/warehouse_4_by_3.yaml"]
    return yaml_paths, loc_tasks, loc_robots












def createTasks(data, numTasks, freeConfig, numTraits=NUM_TRAITS):
    sampleLocation = random.sample(list(freeConfig), k=2 * numTasks)  # w/o replacement

    data[k_tasks] = list()
    for task_nr in range(numTasks):
        initX = round(sampleLocation[task_nr][0] + TASK_POS_UNCERTAINTY * np.random.random(), NUM_TRAILING_DIGITS)
        initY = round(sampleLocation[task_nr][1] + TASK_POS_UNCERTAINTY * np.random.random(), NUM_TRAILING_DIGITS)
        initConfig = Configuration(k_ompl,
                                   k_state,
                                   k_SE2,
                                   initX,
                                   initY,
                                   0.0)

        # FOR CTAS
        termX = initX
        termY = initY

        termConfig = Configuration(k_ompl,
                                   k_state,
                                   k_SE2,
                                   termX,
                                   termY,
                                   0.0)

        traits = list()
        min_trait_rates = list()

        # trait_compositions = {1: [1, 0, 0],
        #                       2: [0, 1, 0],
        #                       3: [1, 0, 1],  # coalitionable
        #                       4: [0, 1, 1],  # coalitionable
        #                       5: [0, 0, 1]}
        # trait_composition_choice = np.random.choice([1, 2, 3, 4, 5], size=1, p=[0.2, 0.2, 0.25, 0.25, 0.1])[0]

        trait_compositions = {1: [1, 0, 0],
                              2: [1, 0, 1],  # coalitionable
                              3: [0, 1, 0]}
        trait_composition_choice = np.random.choice([1, 2, 3], size=1, p=[0.3, 0.4, 0.3])[0]



        for trait_nr in range(numTraits):

            # Traits
            random_ratio = 0.2 + 0.8 * np.random.random()
            task_trait = random_ratio * MAX_TRAIT
            TASK_TRAITS_SUM[trait_nr] += task_trait

            # if not dispensable -- static traits OR not summable
            if PROVISIONABLE_INDEX[trait_nr] < 0.5 or SUMMABLE_INDEX[trait_nr] < 0.5:
                TASK_TRAITS_MIN[trait_nr] = min(TASK_TRAITS_MIN[trait_nr], task_trait)

            task_trait = task_trait * trait_compositions[trait_composition_choice][trait_nr]  # choose composition
            traits.append(round(task_trait, NUM_TRAILING_DIGITS))


            # Trait Rates
            random_ratio_rate = 0.5 + 0.5 * np.random.random()  # EASY
            min_task_trait_rate = random_ratio_rate * MAX_TRAIT_RATES
            TASK_TRAIT_RATES_MIN[trait_nr] = min(TASK_TRAIT_RATES_MIN[trait_nr], min_task_trait_rate)
            TASK_TRAIT_RATES_MAX[trait_nr] = max(TASK_TRAIT_RATES_MAX[trait_nr], min_task_trait_rate)

            # If dispensable and active trait chosen
            if trait_compositions[trait_composition_choice][trait_nr] > 0 and trait_nr != 2:
                min_trait_rates.append(round(min_task_trait_rate, NUM_TRAILING_DIGITS))
            else:
                min_trait_rates.append(0.0) # third trait is not dispensable.


        print(f"Task: {task_nr}, trait: {traits}, min_trait_rates: {min_trait_rates}")
        data[k_tasks].append(
            Task(k_TASK + "-" + str(task_nr),
                 1.0,
                 # np.random.randint(MIN_STATIC_DURATION, MAX_STATIC_DURATION),
                 0.0,  # INITIAL_TASK_REWARD,
                 0.0,  # task_reward_decay_rate,
                 traits,
                 min_trait_rates,
                 SUMMABLE_INDEX, # TASK_TRAITS_SUMMABLE,
                 initConfig,
                 termConfig,
                 0
                 )
        )

    data[k_plan_task_indices] = [task_nr for task_nr in range(numTasks)]













def createSpecies(data, numTasks, numRobots, numTraits=NUM_TRAITS):
    global CAPACITY_FACTOR
    global TASK_TRAITS_SUM
    global TASK_TRAITS_MIN
    global TASK_TRAIT_RATES_MIN
    global TASK_TRAIT_RATES_MAX
    global NUM_HUSKY_SPECIES_VARIANT

    # To make complementary
    HUSKY_PROVISIONABLE_INDEX = [1, 0, 0]  # for three traits case
    HUSKY_PROVISIONABLE_INDEX[-1] = 0  # last trait_nr is not dispensable -- Non Provisional Trait
    JACKAL_PROVISIONABLE_INDEX = [0, 1, 0]  # for three traits case
    JACKAL_PROVISIONABLE_INDEX[-1] = 0  # last trait_nr is not dispensable -- Non Provisional Trait

    EXTRA_MARGIN_MULTIPLIER = 1.1
    TRAIT_VARIATION_RATIOS = [2.0, 1.0, 0.5]
    TRAIT_RATE_VARIATION_RATIOS = [0.5, 1.0, 2.0]

    # HUSKY ------------------------------------------------------------------------------------------------------------
    husky_traits_max = [1.0 for _ in range(NUM_TRAITS)]
    husky_trait_rates_max = [1.0 for _ in range(NUM_TRAITS)]
    current_trait_functions_coeff = [0.0 for _ in range(numTraits)]
    current_trait_rate_functions_coeff = [0.0 for _ in range(numTraits)]

    traits_max_orig = [EXTRA_MARGIN_MULTIPLIER * float(TASK_TRAITS_SUM[i] // NUM_HUSKY_SPECIES_VARIANT) / MIN_TRAIT_LEVEL for i in range(numTraits)]
    # THIS IS DOABLE for this scenario because we assume if task has trait rates, then rate can be summed.
    # trait_rates_max_orig = [float(np.random.uniform(TASK_TRAIT_RATES_MIN[i], TASK_TRAIT_RATES_MAX[i])) for i in range(numTraits)]

    # For small robot size scenarios, no robot have enough trait rates, so must start from max
    # Usually ok when there are at least two robots per species, but this is for single robot per species cases
    trait_rates_max_orig = [float(np.random.uniform(TASK_TRAIT_RATES_MAX[i], 1.01 * TASK_TRAIT_RATES_MAX[i])) for i in range(numTraits)]

    for i in range(numTraits):
        husky_traits_max[i] = traits_max_orig[i] * HUSKY_PROVISIONABLE_INDEX[i]
        if i == numTraits - 1:
            husky_traits_max[i] = 0.0

        if HUSKY_PROVISIONABLE_INDEX[i]:
            husky_trait_rates_max[i] = trait_rates_max_orig[i]
            current_trait_functions_coeff[i] = 0.0  # only trait rate correlates to energy consumption 
            current_trait_rate_functions_coeff[i] = \
                    round(I_AVG / husky_trait_rates_max[i] / numTasks / (1.5 + 0.6 * np.random.random()), NUM_TRAILING_DIGITS)  # HARD
        else:  # not dispensable
            husky_trait_rates_max[i] = 0.0
            current_trait_rate_functions_coeff[i] = 0.0  # no trait rate, no energy consumption
            if husky_traits_max[i] < 1.0:  # doesn't have that capability
                current_trait_functions_coeff[i] = 0.0  
            else:
                current_trait_functions_coeff[i] = \
                        round(I_AVG / husky_traits_max[i] / numTasks / (1.5 + 0.6 * np.random.random()), NUM_TRAILING_DIGITS)  # HARD

    # HUSKY_NAMES = ["Husky_Slow_Rate_Large_Capacity", "Husky", "Husky_Fast_Rate_Small_Capacity"]
    HUSKY_NAMES = ["Husky_Slow_Rate_Large_Capacity", "Husky"]
    for idx, husky_name in enumerate(HUSKY_NAMES):
        data[k_species].append(Species(husky_name,
                                       list(round(TRAIT_VARIATION_RATIOS[idx] * husky_traits_max[j], NUM_TRAILING_DIGITS) for j in range(numTraits)),
                                       list(round(TRAIT_RATE_VARIATION_RATIOS[idx] * husky_trait_rates_max[j], NUM_TRAILING_DIGITS) for j in range(numTraits)),
                                       PROVISIONABLE_INDEX,
                                       EXHAUSTIBLE_INDEX,
                                       current_trait_functions_coeff,
                                       current_trait_rate_functions_coeff,
                                       0.1, # idle current [A]  # EASY  # 2.5, # idle current [A]  # HARD
                                       20.0 * 3600.0 / CAPACITY_FACTOR * min(1.0, numTasks/numRobots), # batt capacity: 20 Ah * 3600 [hr/s] = [As = J/V]
                                       20.0 * 1.0, # max current: 20Ah * 1 C-rating [A]
                                       1.1, # Peukert's coeff for Sealed Lead Acid
                                       1.0,  # max speed: [m/s]
                                       0.15,  # speed coeff.  # EASY # 1.1,  # speed coeff. # HARD
                                       15.0 / 2.0,  # cm due to the size of map -- but true Jackal radius 30 cm
                                       75.0,  # maximum weight[kg]
                                       18500.0,  # $18,500
                                       0))



    # JACKAL ------------------------------------------------------------------------------------------------------------
    jackal_traits_max = [0.0 for _ in range(NUM_TRAITS)]
    jackal_trait_rates_max = [0.0 for _ in range(NUM_TRAITS)]
    current_trait_functions_coeff = [0.0 for _ in range(numTraits)]
    current_trait_rate_functions_coeff = [0.0 for _ in range(numTraits)]

    # traits_max_orig = [EXTRA_MARGIN_MULTIPLIER * float(TASK_TRAITS_SUM[i] // NUM_JACKAL_SPECIES_VARIANT) / MIN_TRAIT_LEVEL * 2.5 for i in range(numTraits)]
    traits_max_orig = [EXTRA_MARGIN_MULTIPLIER * float(TASK_TRAITS_SUM[i] // NUM_JACKAL_SPECIES_VARIANT) / MIN_TRAIT_LEVEL for i in range(numTraits)]

    # THIS IS DOABLE for this scenario because we assume if task has trait rates, then rate can be summed.
    # trait_rates_max_orig = [float(np.random.uniform(TASK_TRAIT_RATES_MIN[i], TASK_TRAIT_RATES_MAX[i])) for i in range(numTraits)]

    # For small robot size scenarios, no robot have enough trait rates, so must start from max
    # Usually ok when there are at least two robots per species, but this is for single robot per species cases
    trait_rates_max_orig = [float(np.random.uniform(TASK_TRAIT_RATES_MAX[i], 1.01 * TASK_TRAIT_RATES_MAX[i])) for i in range(numTraits)]

    for i in range(numTraits):
        jackal_traits_max[i] = traits_max_orig[i] * JACKAL_PROVISIONABLE_INDEX[i]
        if i == numTraits - 1:
            jackal_traits_max[i] = 0.0

        if JACKAL_PROVISIONABLE_INDEX[i]:
            jackal_trait_rates_max[i] = trait_rates_max_orig[i]
            current_trait_functions_coeff[i] = 0.0  # only trait rate correlates to energy consumption 
            current_trait_rate_functions_coeff[i] = \
                    round(I_AVG / jackal_trait_rates_max[i] / numTasks / (1.5 + 0.6 * np.random.random()), NUM_TRAILING_DIGITS)  # HARD
        else:  # not dispensable
            jackal_trait_rates_max[i] = 0.0
            current_trait_rate_functions_coeff[i] = 0.0  # no trait rait, no energy consumption
            if jackal_traits_max[i] < 1:  # doesn't have that capability
                current_trait_functions_coeff[i] = 0.0  
            else:
                current_trait_functions_coeff[i] = \
                        round(I_AVG / jackal_traits_max[i] / numTasks / (1.5 + 0.6 * np.random.random()), NUM_TRAILING_DIGITS)  # HARD

    # JACKAL_NAMES = ["Jackal_Slow_Rate_Large_Capacity", "Jackal", "Jackal_Fast_Rate_Small_Capacity"]
    JACKAL_NAMES = ["Jackal_Slow_Rate_Large_Capacity", "Jackal"]
    for idx, jackal_name in enumerate(JACKAL_NAMES):
        data[k_species].append(Species(jackal_name,
                                       list(round(TRAIT_VARIATION_RATIOS[idx] * jackal_traits_max[j], NUM_TRAILING_DIGITS) for j in range(numTraits)),
                                       list(round(TRAIT_RATE_VARIATION_RATIOS[idx] * jackal_trait_rates_max[j], NUM_TRAILING_DIGITS) for j in range(numTraits)),
                                       PROVISIONABLE_INDEX,
                                       EXHAUSTIBLE_INDEX,
                                       current_trait_functions_coeff,
                                       current_trait_rate_functions_coeff,
                                       0.1, # idle current [A] # 1.41, # idle current [A]
                                       270.0 / 24.0 * 3600.0 / CAPACITY_FACTOR * min(1.0, numTasks / numRobots), # batt capacity: 270 Wh / 24 V * 3600 [hr/s] = [As = J/V]
                                       270.0 / 24.0 * 5.0, # max current: 270 Wh / 24 V * 5 C-rating [A]
                                       1.05, # Peukert's coeff for Lithium ion
                                       2.0,  # max speed: [m/s]
                                       0.15,  # speed coeff.  # EASY # 0.45,  # speed coeff. # EASY
                                       10.0 / 2.0,  # cm due to the size of map -- but true Jackal radius 30 cm
                                       17.0,  # maximum weight[kg]
                                       16000.0,  # $16,000
                                       0))



    # IMPOTENT ---------------------------------------------------------------------------------------------------------
    traits_max_orig = [float(TASK_TRAITS_SUM[j]) / MIN_TRAIT_LEVEL for j in range(numTraits)]
    trait_rates_max_orig= [0.0 for _ in range(numTraits)]
    current_trait_rate_functions_coeff = [0.0 for _ in range(numTraits)]

    current_trait_functions_coeff = \
        [round(I_AVG / traits_max_orig[i] / (1.5 + 0.6 * np.random.random()), NUM_TRAILING_DIGITS) for i in range(numTraits)]  # HARD
    current_trait_functions_coeff[-1] = 0.0  # last one is non dispensable

    for i in range(2):
        traits_max_orig[i] = 0.0
        current_trait_functions_coeff[i] = 0.0

    data[k_species].append(Species("Impotent",
                                   traits_max_orig,
                                   trait_rates_max_orig,
                                   PROVISIONABLE_INDEX,
                                   EXHAUSTIBLE_INDEX,
                                   current_trait_functions_coeff,
                                   current_trait_rate_functions_coeff,
                                   0.1, # idle current [A] # 1.41, # idle current [A]
                                   270.0 / 24.0 * 3600.0 / CAPACITY_FACTOR * min(1.0, numTasks / numRobots), # batt capacity: 270 Wh / 24 V * 3600 [hr/s] = [As = J/V]
                                   270.0 / 24.0 * 5.0, # max current: 270 Wh / 24 V * 5 C-rating [A] 
                                   1.05, # Peukert's coeff for Lithium ion
                                   2.0,  # max speed: [m/s]
                                   0.15,  # speed coeff.  # EASY # 0.45,  # speed coeff. # EASY
                                   10.0 / 2.0,  # cm due to the size of map -- but true Jackal radius 30 cm
                                   17.0,  # maximum weight[kg]
                                   1000.0,  # $1,000
                                   0))



def createRobots(data, speciesName, numRobots, sampleLocations, numTraits=NUM_TRAITS, isInitRandom=True, listInitConfigs=[]):
    global ROBOT_TRAITS_SUM
    global ROBOT_TRAITS_MIN
    global ROBOT_TRAIT_RATES_MIN
    global ROBOT_TRAIT_RATES_MAX
    global TASK_TRAITS_SUM
    global TASK_TRAITS_MIN
    global TASK_TRAIT_RATES_MIN
    global TASK_TRAIT_RATES_MAX

    for robot_nr in range(numRobots):
        name = "".join(word[0] for word in speciesName.split("_")) # extract initials

        configObj = Configuration(k_ompl, k_state, k_SE2, 0.0, 0.0, 0.0)
        if isInitRandom:
            location = sampleLocations[0]
            sampleLocations.pop(0)
            configObj.x = float(location[0])
            configObj.y = float(location[1])

        # WORKING COMPOSITION -- EASY
        # initial_battery_level = round(0.7 + 0.3 * np.random.random(), NUM_TRAILING_DIGITS)
        # initial_trait_levels = [round(0.8 + 0.2 * np.random.random(), NUM_TRAILING_DIGITS) for i in range(numTraits)]

        # TESTING COMPOSITION -- HARD
        initial_battery_level = round(MIN_BATTERY_LEVEL + 0.8 * np.random.random(), NUM_TRAILING_DIGITS)
        initial_trait_levels = [round(MIN_TRAIT_LEVEL + 0.8 * np.random.random(), NUM_TRAILING_DIGITS) for i in range(numTraits)]

        # print(speciesName, data[k_species][SPECIES_NAME_IDX_MAP[speciesName]].trait_rates_max)
        for trait_nr in range(numTraits):
            ROBOT_TRAITS_SUM[trait_nr] += \
                    initial_trait_levels[trait_nr] * \
                    data[k_species][SPECIES_NAME_IDX_MAP[speciesName]].traits_max[trait_nr]


            if data[k_species][SPECIES_NAME_IDX_MAP[speciesName]].traits_max[trait_nr] > 0.0:
                ROBOT_TRAITS_MIN[trait_nr] = \
                        min(initial_trait_levels[trait_nr] * \
                                data[k_species][SPECIES_NAME_IDX_MAP[speciesName]].traits_max[trait_nr], \
                            ROBOT_TRAITS_MIN[trait_nr])
            else:
                initial_trait_levels[trait_nr] = 0.0

            if data[k_species][SPECIES_NAME_IDX_MAP[speciesName]].trait_rates_max[trait_nr] > 0.0:
                ROBOT_TRAIT_RATES_MIN[trait_nr] = \
                        min(data[k_species][SPECIES_NAME_IDX_MAP[speciesName]].trait_rates_max[trait_nr], \
                            ROBOT_TRAIT_RATES_MIN[trait_nr])
                ROBOT_TRAIT_RATES_MAX[trait_nr] = \
                        max(data[k_species][SPECIES_NAME_IDX_MAP[speciesName]].trait_rates_max[trait_nr], \
                            ROBOT_TRAIT_RATES_MAX[trait_nr])


        data[k_robots].append(Robot(name+"-"+str(robot_nr),
                                    initial_battery_level,
                                    initial_trait_levels,
                                    configObj,
                                    speciesName))





def createMotionPlanners(data, yaml_paths):
    data[k_motion_planners] = list()
    for yaml_path in yaml_paths:
        k_environment_parameters = EnvironmentParameters(k_pgm_ompl_environment,
                                                         k_ompl,
                                                         k_pgm,
                                                         True,
                                                         yaml_path)
        k_algorithm_parameters = AlgorithmParameter(k_ompl_motion_planner_parameters,
                                                    k_prm,
                                                    10.0,
                                                    True,
                                                    -1.0,
                                                    0.7,  # 0.1, -- this causes no path
                                                    k_ompl)
        k_config_type = k_ompl_motion_planner

        data[k_motion_planners].append(MotionPlanner(k_environment_parameters,
                                                     k_algorithm_parameters,
                                                     k_config_type))




def createDtasParameters(data, alpha, beta, gamma=0.5):
    data[k_dtas_parameters] = dtasParameter(k_best_first_search_parameters,
                                            True,  # Timeout
                                            SEARCH_BUDGET,
                                            k_dtas,
                                            alpha,
                                            beta,
                                            gamma,
                                            True,  # False: prune before eval
                                            False,
                                            False
                                            )
    data[k_trait_distribution_parameters] = traitDistributionParameters(k_trait_distribution_parameters,
                                                                        1.5 * NLP_TIMEOUT,
                                                                        NLP_TIMEOUT)




def createItagsParameters(data):
    data[k_itags_parameters]  = {
        "config_type": "BestFirstSearchParameters",
        "has_timeout": True,
        "timeout": SEARCH_BUDGET,
        "timer_name": "itags",
        "prune_before_eval": True,
        "save_pruned_nodes": False,
        "save_closed_nodes": False
    }



def createSchedulerParameters(data):
    data[k_scheduler_parameters] = SchedulerParameter(k_dtas_milp_scheduler_paramters,
                                                      k_milp,
                                                      k_deterministic,
                                                      10.0 + max(len(data[k_tasks]), len(data[k_robots])) * 6.0,
                                                      max(len(data[k_tasks]), len(data[k_robots])) * 6.0,
                                                      0,
                                                      False,
                                                      True)




def createPrecedenceConstraints(numTasks, numConstraints):
    if numConstraints == 0:
        return list()

    all_possible_constraints = list()
    for i in range(numTasks):
        for j in range(1, numTasks):
            if i != j:
                all_possible_constraints.append((i, j))
                all_possible_constraints.append((j, i))

    rv_idx = np.random.choice(len(all_possible_constraints), numConstraints, replace=False)
    rv = [all_possible_constraints[idx] for idx in rv_idx]
    pc_rv_excluded = set(all_possible_constraints).symmetric_difference(set(rv))
    pc_rv_excluded = list(pc_rv_excluded)
    return rv, pc_rv_excluded



def createDeadlineConstraints(numTasks, numRobots, numAbs, numRel, pc_rv_excluded):
    if numAbs + numRel == 0:
        return list()

    rv = list()
    if numAbs > 0:
        for i in range(numAbs):
            start_or_completion = k_start if (np.random.random() < 0.5) else k_completion
            task_nr = np.random.randint(0, numTasks)
            tp_obj = TimePoint(start_or_completion, task_nr)
            # bound = float(np.random.randint(ABS_DEADLINE_LB, ABS_DEADLINE_UB))
            # bound = float(np.random.randint(round(ABS_DEADLINE_LB * numTasks**2 / numRobots),
            #                                 round(ABS_DEADLINE_UB * numTasks**2 / numRobots)))
            bound = float(np.random.randint(round(0.5 * ABS_DEADLINE_UB * numTasks**2 / np.sqrt(numRobots)),
                                            round(ABS_DEADLINE_UB * numTasks**2 / np.sqrt(numRobots))))
            if tp_obj.timepoint_type == "start":
                # bound = float(np.random.randint(round(ABS_DEADLINE_LB * numTasks**2 / numRobots),
                #                                 round(1.5 * ABS_DEADLINE_LB * numTasks**2 / numRobots)))
                bound = float(np.random.randint(round(ABS_DEADLINE_LB * numTasks**2 / 5),
                                                round(1.5 * ABS_DEADLINE_LB * numTasks**2 / 5)))
                # 왜냐하면, 로봇이 많다고 빨리 출발할 수 없으니까, initial configuration 까지 가는 시간은 어떤 팀이던 비슷함.
            print("absolute bound", bound)
            rv.append(AbsoluteDeadline(tp_obj, bound))
    if numRel > 0:
        task_chosen_id = np.random.choice(len(pc_rv_excluded), numRel, replace=False)
        tasks_chosen = [pc_rv_excluded[i] for i in task_chosen_id]

        print("tasks chosen", tasks_chosen)
        for j in range(numRel):
            start_or_completion = k_start if (np.random.random() < 0.5) else k_completion
            predecessor = TimePoint(start_or_completion, tasks_chosen[j][0])

            start_or_completion = k_start if (np.random.random() < 0.5) else k_completion
            successor = TimePoint(start_or_completion, tasks_chosen[j][1])
            # bound = float(np.random.randint(REL_DEADLINE_LB, REL_DEADLINE_UB))
            bound = float(np.random.randint(round(REL_DEADLINE_LB * numTasks**2 / numRobots),
                                            round(REL_DEADLINE_UB * numTasks**2 / numRobots)))
            if predecessor.timepoint_type == "start" and successor.timepoint_type == "completion":
                bound = float(np.random.randint(round(0.5 * REL_DEADLINE_UB * numTasks**2 / numRobots),
                                                round(REL_DEADLINE_UB * numTasks**2 / numRobots)))


            print("relative bound", bound)
            rv.append(RelativeDeadline(predecessor, successor, bound))

    print("deadline constraints", rv)
    return rv








def createScenario(data, yaml_paths, output_dir_path, NUM_ROBOTS_DICT, NUM_TASKS, freeConfig_tasks, freeConfig_robots,
                   numTraits=NUM_TRAITS, test_nr=1, alpha=0.8, robotarium_test=False):

    # CREATE ROBOTS
    data[k_robots] = list()
    sampleLocations = random.sample(list(freeConfig_robots), k=sum(NUM_ROBOTS_DICT.values()))  # w/o replacement


    # CREATE TASKS -- do this FIRST
    createTasks(data, NUM_TASKS, freeConfig_tasks)

    # print(f"TASK_TRAITS_SUM: {TASK_TRAITS_SUM}")
    # print(f"TASK_TRAITS_MIN: {TASK_TRAITS_MIN}")
    # print(f"TASK_TRAIT_RATES_MAX: {TASK_TRAIT_RATES_MAX}")
    # print(f"TASK_TRAIT_RATES_MIN: {TASK_TRAIT_RATES_MIN}")

    numRobots = sum(NUM_ROBOTS_DICT.values())
    createSpecies(data, NUM_TASKS, numRobots, NUM_TRAITS)

    #-- if sampleLocations fail, check the size of set and the number to draw (set size must be greater than to draw) --#
    for species, num_robots_per_species in NUM_ROBOTS_DICT.items():
        # print(species)
        createRobots(data, species, num_robots_per_species, sampleLocations, numTraits)

    # print(f"ROBOT_TRAIT_RATES_MIN: {ROBOT_TRAIT_RATES_MIN}")
    # print(f"ROBOT_TRAIT_RATES_MAX: {ROBOT_TRAIT_RATES_MAX}")


    NUM_ROBOTS = len(data[k_robots])

    # TODO: precedence constraints
    data[k_precedence_constraints], pc_rv_excluded = createPrecedenceConstraints(NUM_TASKS, 1)

    # TODO: deadline constraints
    data[k_deadline_constraints] = createDeadlineConstraints(NUM_TASKS, NUM_ROBOTS, 1, 1, pc_rv_excluded)

    # CREATE MOTION PLANNERS
    createMotionPlanners(data, yaml_paths)

    # CREATE dtas PARAMETERS
    createDtasParameters(data, alpha, beta=0.0)

    # CREATE SCHEDULER PARAMETERS
    createSchedulerParameters(data)

    createItagsParameters(data)

    if not robotarium_test:
        test_nr = 0
        dirlist = os.scandir(output_dir_path)
        for entry in dirlist:
            if entry.is_dir():
                temp = [int(s) for s in re.findall(r'\d+', entry.name)]
                test_nr = max(test_nr, max(temp) if len(temp) >= 1 else 0)
        dirlist.close()

    test_dir_path = output_dir_path + "/test" + r"{:d}".format(test_nr + 1)
    os.mkdir(test_dir_path)

    # Store the JSON data in a file
    output_file_name = "input"
    with open(test_dir_path + "/" + output_file_name + k_json, "w") as write_file:
        serialised = json.loads(json.dumps(data, cls=ProblemInputEncoder))
        json.dump(toSolverSchema(serialised), write_file, indent=2)
        # print("[DEBUG] Data stored into a file:", write_file.name)








def generateInstanceSet(out_name):
    global NUM_HUSKY_SPECIES_VARIANT
    global NUM_JACKAL_SPECIES_VARIANT

    # -------------------- BEGIN: USER INPUT REQUIRED --------------------#

    freeConfig = set()
    yaml_paths, freeConfig_tasks, freeConfig_robots = warehouse4by3FreeConfigurations(freeConfig)
    alpha = 0.5

    # -------------------- END: USER INPUT REQUIRED --------------------#

    dir_path = os.path.dirname(os.path.realpath(__file__))
    # tests_dir_path = dir_path + "/gradient_plot_benchmark_easy"
    # tests_dir_path = dir_path + "/gradient_plot_benchmark_hard_slight"
    # tests_dir_path = dir_path + "/gradient_plot_benchmark_hardest"
    repo_root = os.path.dirname(dir_path)
    tests_dir_path = os.path.join(repo_root, "data", "problem_inputs", "traits", out_name)

    try:
        os.mkdir(tests_dir_path)
    except OSError as error:
        print(error)

    for num_tasks in [5, 10, 15, 20, 25]:
        for num_robots in [5, 10, 15, 20, 25]:
            NUM_HUSKY_SPECIES_VARIANT = 2
            NUM_JACKAL_SPECIES_VARIANT = 2
            NUM_ROBOTS_DICT = {"Husky_Slow_Rate_Large_Capacity": round(num_robots / 5),
                               "Husky": round(num_robots / 5),
                               "Jackal_Slow_Rate_Large_Capacity": round(num_robots / 5),
                               "Jackal": round(num_robots / 5),
                               "Impotent": round(num_robots / 5)}


            data = dict()
            data[k_species] = list()
            for num_tests in range(4):
                resetSum()
                unique_data = copy.deepcopy(data)

                createScenario(unique_data, yaml_paths, tests_dir_path, NUM_ROBOTS_DICT, num_tasks,
                               freeConfig_tasks, freeConfig_robots, alpha=alpha, robotarium_test=False)



if __name__ == "__main__":
    args = parseArgs()
    seed = int(time.time() * 1000) if args.seed == "time" else int(args.seed)
    random.seed(seed)
    np.random.seed(seed % (2 ** 32))
    print(f"RNG seed = {seed}")
    generateInstanceSet(args.out)
