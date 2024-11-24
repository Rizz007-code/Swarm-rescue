import argparse
import gc
import os
import sys
import traceback
from typing import Tuple

from spg_overlay.entities.sensor_disablers import ZoneType
from spg_overlay.reporting.result_path_creator import ResultPathCreator
from spg_overlay.utils.constants import DRONE_INITIAL_HEALTH
from spg_overlay.reporting.evaluation import EvalConfig, EvalPlan, ZonesConfig
from spg_overlay.reporting.score_manager import ScoreManager
from spg_overlay.reporting.data_saver import DataSaver
from spg_overlay.reporting.team_info import TeamInfo
from spg_overlay.gui_map.gui_sr import GuiSR

from maps.map_intermediate_01 import MyMapIntermediate01
from maps.map_intermediate_02 import MyMapIntermediate02
from maps.map_medium_01 import MyMapMedium01
from maps.map_medium_02 import MyMapMedium02

from solutions.my_drone_eval import MyDroneEval


class MyDrone(MyDroneEval):
    def __init__(self):
        super().__init__()

    def navigate(self, position, zones):
        """
        Implements navigation logic based on the drone's current position and zones.
        Avoids kill zones, handles no communication zones, and no GPS zones.
        """
        if zones.is_in_kill_zone(position):
            print(f"Drone {self.drone_id}: Avoiding Kill Zone at {position}")
            self.avoid_kill_zone(position)
        elif zones.is_in_no_com_zone(position):
            print(f"Drone {self.drone_id}: Navigating No Communication Zone at {position}")
            self.proceed_with_caution(position)
        elif zones.is_in_no_gps_zone(position):
            print(f"Drone {self.drone_id}: Navigating No GPS Zone at {position}")
            self.rely_on_backup_navigation(position)
        else:
            print(f"Drone {self.drone_id}: Exploring position {position}")
            self.explore(position)

    def avoid_kill_zone(self, position):
        print(f"Drone {self.drone_id}: Calculating safe position to avoid Kill Zone.")
        safe_position = (position[0] - 2, position[1] - 2) if position[0] > 0 and position[1] > 0 else (0, 0)
        print(f"Drone {self.drone_id}: Moving to safe position {safe_position}.")
        self.move_to_position(safe_position)

    def proceed_with_caution(self, position):
        print(f"Drone {self.drone_id}: Reducing speed and scanning in No Communication Zone.")
        self.reduce_speed(0.5)
        nearby_obstacle = self.scan_for_obstacles(position)
        if nearby_obstacle:
            print(f"Drone {self.drone_id}: Detected obstacle nearby, adjusting direction.")
            self.change_direction(90)  # Example: Turn right
        self.move_forward()

    def rely_on_backup_navigation(self, position):
        print(f"Drone {self.drone_id}: Relying on backup sensors in No GPS Zone.")
        if self.near_wall(position):
            print(f"Drone {self.drone_id}: Following wall to navigate.")
            self.follow_wall()
        else:
            self.move_forward()

    def explore(self, position):
        print(f"Drone {self.drone_id}: Exploring area at position {position}.")
        if self.detect_wounded_person(position):
            print(f"Drone {self.drone_id}: Wounded person detected. Initiating rescue.")
            self.rescue_wounded_person()
        else:
            print(f"Drone {self.drone_id}: No wounded person detected. Scanning area.")
            self.scan_area()

    def move_to_position(self, position):
        print(f"Drone {self.drone_id}: Moving to position {position}.")
        pass

    def scan_for_obstacles(self, position):
        print(f"Drone {self.drone_id}: Scanning for obstacles around {position}.")
        return position[0] % 2 == 0 and position[1] % 2 == 0

    def reduce_speed(self, factor):
        print(f"Drone {self.drone_id}: Reducing speed by factor {factor}.")
        pass

    def near_wall(self, position):
        return position[0] <= 1 or position[1] <= 1

    def follow_wall(self):
        print(f"Drone {self.drone_id}: Following wall for navigation.")
        pass

    def detect_wounded_person(self, position):
        print(f"Drone {self.drone_id}: Checking for wounded person at {position}.")
        return (position[0] + position[1]) % 5 == 0

    def rescue_wounded_person(self):
        print(f"Drone {self.drone_id}: Rescuing wounded person.")
        pass

    def scan_area(self):
        print(f"Drone {self.drone_id}: Scanning area for hazards and unexplored zones.")
        pass

    def change_direction(self, degrees):
        print(f"Drone {self.drone_id}: Changing direction by {degrees} degrees.")
        pass

    def move_forward(self):
        print(f"Drone {self.drone_id}: Moving forward.")
        pass


class Launcher:
    def __init__(self):
        self.team_info = TeamInfo()
        self.eval_plan = EvalPlan()

        eval_config = EvalConfig(map_type=MyMapIntermediate01, nb_rounds=2)
        self.eval_plan.add(eval_config=eval_config)

        eval_config = EvalConfig(map_type=MyMapIntermediate02)
        self.eval_plan.add(eval_config=eval_config)

        zones_config: ZonesConfig = (ZoneType.NO_COM_ZONE, ZoneType.NO_GPS_ZONE, ZoneType.KILL_ZONE)
        eval_config = EvalConfig(map_type=MyMapMedium01, zones_config=zones_config, nb_rounds=2, config_weight=1)
        self.eval_plan.add(eval_config=eval_config)

        eval_config = EvalConfig(map_type=MyMapMedium02, zones_config=zones_config, nb_rounds=2, config_weight=2)
        self.eval_plan.add(eval_config=eval_config)

        self.number_drones = None
        self.max_timestep_limit = None
        self.max_walltime_limit = None
        self.number_wounded_persons = None
        self.size_area = None
        self.score_manager = None

        stat_saving_enabled = False
        self.video_capture_enabled = False

        self.result_path = None
        if stat_saving_enabled or self.video_capture_enabled:
            rpc = ResultPathCreator(self.team_info)
            self.result_path = rpc.path
        self.data_saver = DataSaver(team_info=self.team_info, result_path=self.result_path, enabled=stat_saving_enabled)

    def one_round(self, eval_config: EvalConfig, num_round: int, hide_solution_output: bool = False):
        print("\n********************************")
        my_map = eval_config.map_type(eval_config.zones_config)
        self.number_drones = my_map.number_drones
        self.max_timestep_limit = my_map.max_timestep_limit
        self.max_walltime_limit = my_map.max_walltime_limit
        self.number_wounded_persons = my_map.number_wounded_persons
        self.size_area = my_map.size_area

        self.score_manager = ScoreManager(
            number_drones=self.number_drones,
            max_timestep_limit=self.max_timestep_limit,
            max_walltime_limit=self.max_walltime_limit,
            total_number_wounded_persons=self.number_wounded_persons,
        )

        my_playground = my_map.construct_playground(drone_type=MyDrone)
        zones = my_map.zones_config
        for drone in my_playground.drones:
            position = drone.get_position()
            drone.navigate(position, zones)

        num_round_str = str(num_round)

    def go(self, stop_at_first_crash: bool = False, hide_solution_output: bool = False):
        for eval_config in self.eval_plan.list_eval_config:
            gc.collect()
            print(f"*** Map: {eval_config.map_name}")
            for num_round in range(eval_config.nb_rounds):
                self.one_round(eval_config, num_round + 1, hide_solution_output)


if __name__ == "__main__":
    gc.disable()
    parser = argparse.ArgumentParser(description="Launcher of a swarm-rescue simulator for the competition")
    parser.add_argument("--stop_at_first_crash", "-s", action="store_true", help="Stop the code at first crash")
    parser.add_argument("--hide_solution_output", "-o", action="store_true", help="Hide print output of the solution")
    args = parser.parse_args()

    launcher = Launcher()
    ok = launcher.go(stop_at_first_crash=args.stop_at_first_crash, hide_solution_output=args.hide_solution_output)
    if not ok:
        exit(1)
