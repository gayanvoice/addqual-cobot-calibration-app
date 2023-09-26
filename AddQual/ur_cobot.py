import URBasic
from model.joint_position_model import JointPositionModel
from model.move_j_command_model import MoveJCommandModel


class URCobot:

    def __init__(self):
        self.ur_script_ext = None

    async def connect_ur_cobot_physical_device(self):
        robot_model = URBasic.robotModel.RobotModel()
        self.ur_script_ext = URBasic.urScriptExt.UrScriptExt(
            host="10.2.12.109",
            robotModel=robot_model)
        self.ur_script_ext.reset_error()

    async def connect(self):
        await self.connect_ur_cobot_physical_device()

    async def move_j_command_request_handler(self, move_j_command_model):
        try:
            for joint_position_model in move_j_command_model.joint_position_model_array:
                joint_position_array = JointPositionModel.get_position_array_from_joint_position_model(
                    joint_position_model=joint_position_model
                )
                self.ur_script_ext.movej(q=joint_position_array,
                                         a=move_j_command_model.acceleration,
                                         v=move_j_command_model.velocity,
                                         t=move_j_command_model.time_s,
                                         r=move_j_command_model.blend_radius)
            return True
        except Exception as ex:
            return False

    async def get_is_in_remote_control(self):
        try:
            is_in_remote_control_string = self.ur_script_ext.get_is_in_remote_control()
            decoded_string = is_in_remote_control_string.decode('utf-8').strip()
            is_in_remote_control_bool = decoded_string.lower() == 'true'
            return is_in_remote_control_bool
        except Exception as ex:
            return False
