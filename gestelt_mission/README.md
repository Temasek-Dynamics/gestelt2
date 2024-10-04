# gestlet_mission
Scripts for executing pre-defined missions.
The [scenarios.json](scenarios.json) aids creation of different scenarios which can be easily tested by the planner, without having to modify the launch file direcly to launch different maps and agent configurations.

# Creating a scenario
A new scenario is created by adding a new dictionary entry to the JSON file [scenarios.json](scenarios.json).
- `SCENARIO_NAME`: Unique identifier of scenario that is used to refer to a specific setup
- `description`: Optional description 
- `map`: Name of point cloud map contained in `gestelt_bringup/pcd_maps` folder
- `spawns_pos`: Spawn position of agents corresponding to each agent in ascending order
- `goals_pos`: Goal position of agents corresponding to each agent in ascending order

```json
    "SCENARIO_NAME" : {
        "description": "DESCRIPTION_OF_SCENARIO",
        "map": "MAP_NAME",
        "num_agents": 2,
        "spawns_pos": [
            [-5.5, -5.5, 2.0],
            [5.5, 5.5, 2.0]
        ],
        "goals_pos": [
            [5.5, 5.5, 2.0],
            [-5.5, -5.5, 2.0]
        ]
    },
``` 