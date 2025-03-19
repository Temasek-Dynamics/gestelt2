import os
import json
import numpy as np
import matplotlib.pyplot as plt

class ScenarioGenerator:
    def __init__(self, take_off_height=1.5, visualize=False):
        self.scenes_dict = {}
        self.visualize = visualize
        self.take_off_height = take_off_height

    def normalizeYaw(self, yaw):
        yaw_norm = yaw
        if (yaw > np.pi):
            yaw_norm -= 2 *  np.pi
        if (yaw < -np.pi):
            yaw_norm += 2 *  np.pi

        return yaw_norm
    
    def forwardFlight(self, m_size_x, m_size_y, num_agents, goal_z, name=None, map_name=None):
        """Generate forward swarm flight scenario

        Args:
            m_size_x (_type_): Map size in x
            m_size_y (_type_): Map size in y
            num_agents (_type_): Number of agents
            goal_z (_type_): height of goal
            name (_type_, optional): _description_. Defaults to None.
            map_name (_type_, optional): _description_. Defaults to None.
        """
        if name is None:
            name = "forward_flight_" + str(num_agents)
        if map_name is None:
            map_name = "empty_map"

        spawns_pos = []
        goals_pos = []

        start_y = -m_size_y/2 - 1.5  
        inter_agent_sep = 1.5 # inter agent separation

        for i in range(num_agents):
            if i%2 == 0:
                spawn_x = (-i/2) * inter_agent_sep
            else: 
                spawn_x = (int(i/2) + 1) * inter_agent_sep
            spawn_y = start_y
            spawn_yaw = self.normalizeYaw(0.0001) 

            goal_x = spawn_x
            goal_y = -spawn_y

            spawns_pos.append([spawn_x, spawn_y, spawn_yaw])
            goals_pos.append([goal_x, goal_y, goal_z])

        self.scenes_dict[name] = {
            "description" : "",
            "map" : map_name,
            "take_off_height": self.take_off_height,
            "num_agents" : num_agents,
            "spawns_pos": spawns_pos,
            "goals_pos": goals_pos
        } 

        if self.visualize:
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6) )

            # Plot spawn points 
            for i in range(len(spawns_pos)):
                ax1.plot(spawns_pos[i][0], spawns_pos[i][1], 
                         'o', label=f'Agent_{i}_spawn')
                # Direction
                dx = goals_pos[i][0] - spawns_pos[i][0]
                dy = goals_pos[i][1] - spawns_pos[i][1]
                ax1.arrow(spawns_pos[i][0], spawns_pos[i][1],
                          dx, dy, width=0.05)
                ax1.text(spawns_pos[i][0], spawns_pos[i][1], 
                         f's_{i}', fontsize=10)
            # # Plot goal points 
            for i in range(len(goals_pos)):
                ax2.plot(goals_pos[i][0], goals_pos[i][1], 
                         'o', label=f'Agent_{i}_goal')
                ax2.text(goals_pos[i][0], goals_pos[i][1], 
                         f'g_{i}', fontsize=10)
            
            ax1.legend()
            ax2.legend()
            plt.show()

        pass
    
    def antipodalSwap(self, radius, num_agents, goal_z, name=None, map_name=None):
        """Generate antipodal swap scenario

        Args:
            radius (_type_): distance between agents
            num_agents (_type_): _description_
            goal_z (_type_): _description_
            name (_type_, optional): _description_. Defaults to None.
            map_name (_type_, optional): _description_. Defaults to None.
        """

        if name is None:
            name = "antipodal_swap_" + str(num_agents)
        if map_name is None:
            map_name = "empty_map"

        spawns_pos = []
        goals_pos = []

        # Angle between the line joining each agent and the center of the map
        angle = (2 * np.pi) / num_agents

        # Start from x-axis and go counter-clockwise
        for i in range(num_agents):
            spawn_x = radius * np.cos(i * angle)
            spawn_y = radius * np.sin(i * angle)
            spawn_yaw = self.normalizeYaw(-np.pi/2 - i * angle ) 

            goal_x = - spawn_x
            goal_y = - spawn_y

            spawns_pos.append([spawn_x, spawn_y, spawn_yaw])
            goals_pos.append([goal_x, goal_y, goal_z])

        self.scenes_dict[name] = {
            "description" : "",
            "map" : map_name,
            "take_off_height": self.take_off_height,
            "num_agents" : num_agents,
            "spawns_pos": spawns_pos,
            "goals_pos": goals_pos
        } 

        if self.visualize:
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6) )

            # Plot spawn points 
            for i in range(len(spawns_pos)):
                ax1.plot(spawns_pos[i][0], spawns_pos[i][1], 
                         'o', label=f'Agent_{i}_spawn')
                
                dx = (radius - 0.6)* np.cos(i * angle) - spawns_pos[i][0]
                dy = (radius - 0.6)* np.sin(i * angle) - spawns_pos[i][1]
                ax1.arrow(spawns_pos[i][0], spawns_pos[i][1],
                          dx, dy, width=0.05)
                ax1.text(spawns_pos[i][0], spawns_pos[i][1], 
                         f's_{i}', fontsize=10)
            # # Plot goal points 
            for i in range(len(goals_pos)):
                ax2.plot(goals_pos[i][0], goals_pos[i][1], 
                         'o', label=f'Agent_{i}_goal')
                ax2.text(goals_pos[i][0], goals_pos[i][1], 
                         f'g_{i}', fontsize=10)
            
            ax1.legend()
            ax2.legend()
            plt.show()

    # def forwardFlight(self):

    # def forwardSwap(self):

    def saveJSON(self, filename):
        if self.scenes_dict is None:
            print("Nothing to save! No scenes were generated.")


        filepath = os.path.join(
            os.path.expanduser("~"), 
            'gestelt_ws/src/gestelt2/gestelt_commander',
            filename
        )

        with open(filepath, mode="w", encoding="utf-8") as write_file:
            json.dump(self.scenes_dict, write_file, indent=2)
            print(f"Saved to {filepath}")

def main():
    scene_gen = ScenarioGenerator(take_off_height=1.5, visualize=True)

    scene_gen.antipodalSwap(
        radius = 2.0, 
        num_agents = 2,
        goal_z = 0.75,
        name = "vicon_antipodal_2", 
        map_name = "empty_20x20", 
    )

    # scene_gen.antipodalSwap(
    #     radius = 8.0, 
    #     num_agents = 10,
    #     goal_z = 1.5,
    #     name = "antipodal_swap_10", 
    #     map_name = "forest_sparse_10x10", 
    # )

    # scene_gen.forwardFlight(
    #     m_size_x = 25.0, 
    #     m_size_y = 50.0,
    #     num_agents = 10,
    #     goal_z = 1.5,
    #     name = "forward_flight_10", 
    #     map_name = "long_forest_sparse_50x25", 
    # )

    scene_gen.saveJSON(
        filename="new_scenarios.json"
    )


if __name__ == "__main__":
    main()