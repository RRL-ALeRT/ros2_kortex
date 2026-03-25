from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("gen3", package_name="gen3_85_moveit_config").to_moveit_configs()
    pipeline = MoveItConfigsBuilder("gen3", package_name="gen3_85_moveit_config").planning_pipelines(pipelines=["ompl"]).to_moveit_configs()
    moveit_config.planning_pipelines.update(pipeline.planning_pipelines)
    return generate_move_group_launch(moveit_config)
    return generate_move_group_launch(moveit_config)
