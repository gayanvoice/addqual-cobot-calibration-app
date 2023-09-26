import asyncio

from AddQual.ur_gripper import URGripper
from AddQual.ur_cobot import URCobot
from model.joint_position_model import JointPositionModel
from model.move_j_command_model import MoveJCommandModel

home_joint_position_model = JointPositionModel.get_joint_position_model_using_arguments(
    base=270, shoulder=-110, elbow=150, wrist1=-130, wrist2=270, wrist3=0)

retract_ct_01_joint_position_model = JointPositionModel.get_joint_position_model_using_arguments(
    base=277.395, shoulder=-85.505, elbow=145.82, wrist1=-58.295, wrist2=99.43, wrist3=180.235)
target_ct_01_joint_position_model = JointPositionModel.get_joint_position_model_using_arguments(
    base=274.82, shoulder=-73.285, elbow=129.04, wrist1=-53.745, wrist2=96.835, wrist3=180.17)
calibrate_ct_01_joint_position_model = JointPositionModel.get_joint_position_model_using_arguments(
    base=274.82, shoulder=-68.455, elbow=129.935, wrist1=-59.47, wrist2=96.835, wrist3=180.155)

retract_cg_base_joint_position_model = JointPositionModel.get_joint_position_model_using_arguments(
    base=141.19, shoulder=-90.35, elbow=152.93, wrist1=-61.65, wrist2=53.17, wrist3=178.51)
calibration_cg_base_joint_position_model = JointPositionModel.get_joint_position_model_using_arguments(
    base=148.43, shoulder=-70.04, elbow=144.92, wrist1=-72.995, wrist2=59.06, wrist3=178.31)


async def cobot_calibration():
    try:
        ur_cobot = URCobot()
        ur_gripper = URGripper()

        await ur_cobot.connect()
        await ur_gripper.connect()

        if await ur_cobot.get_is_in_remote_control():
            if await validate_ct_position(ur_cobot=ur_cobot, ur_gripper=ur_gripper):
                print("ct_01_position is valid")
            else:
                print("ct_01_position is invalid")
            if await validate_cg_position(ur_cobot=ur_cobot, ur_gripper=ur_gripper):
                print("cg_base_position is valid")
            else:
                print("cg_base_position is invalid")
        else:
            print("error cobot calibrate: cobot is local control")

    except Exception as ex:
        print(ex)


async def validate_ct_position(ur_cobot, ur_gripper):
    home_to_retract_ct_01_joint_position_model_array = [
        home_joint_position_model, retract_ct_01_joint_position_model]
    retract_ct_01_to_target_ct_01_joint_position_model_array = [
        retract_ct_01_joint_position_model, target_ct_01_joint_position_model]
    target_ct_01_to_calibrate_ct_01_joint_position_model_array = [
        target_ct_01_joint_position_model, calibrate_ct_01_joint_position_model]
    calibrate_ct_01_to_target_ct_01_joint_position_model_array = [
        calibrate_ct_01_joint_position_model, target_ct_01_joint_position_model]
    target_ct_01_to_retract_ct_01_joint_position_model_array = [
        target_ct_01_joint_position_model, retract_ct_01_joint_position_model]
    retract_ct_01_to_home_joint_position_model_array = [
        retract_ct_01_joint_position_model, home_joint_position_model]

    home_to_retract_ct_01_move_j_command_model = MoveJCommandModel.get_move_j_command_model_using_arguments(
        acceleration=0.5, velocity=0.5, time_s=0, blend_radius=0,
        joint_position_model_array=home_to_retract_ct_01_joint_position_model_array)
    retract_ct_01_to_target_ct_01_move_j_command_model = MoveJCommandModel.get_move_j_command_model_using_arguments(
        acceleration=0.5, velocity=0.5, time_s=0, blend_radius=0,
        joint_position_model_array=retract_ct_01_to_target_ct_01_joint_position_model_array)
    target_ct_01_to_calibrate_ct_01_move_j_command_model = MoveJCommandModel.get_move_j_command_model_using_arguments(
        acceleration=0.5, velocity=0.5, time_s=0, blend_radius=0,
        joint_position_model_array=target_ct_01_to_calibrate_ct_01_joint_position_model_array)
    calibrate_ct_01_to_target_ct_01_move_j_command_model = MoveJCommandModel.get_move_j_command_model_using_arguments(
        acceleration=0.5, velocity=0.5, time_s=0, blend_radius=0,
        joint_position_model_array=calibrate_ct_01_to_target_ct_01_joint_position_model_array)
    target_ct_01_to_retract_ct_01_move_j_command_model = MoveJCommandModel.get_move_j_command_model_using_arguments(
        acceleration=0.5, velocity=0.5, time_s=0, blend_radius=0,
        joint_position_model_array=target_ct_01_to_retract_ct_01_joint_position_model_array)
    retract_ct_01_to_home_move_j_command_model = MoveJCommandModel.get_move_j_command_model_using_arguments(
        acceleration=0.5, velocity=0.5, time_s=0, blend_radius=0,
        joint_position_model_array=retract_ct_01_to_home_joint_position_model_array)

    await ur_gripper.close_gripper()
    await ur_cobot.move_j_command_request_handler(
        move_j_command_model=home_to_retract_ct_01_move_j_command_model)
    await ur_gripper.open_gripper()
    await ur_cobot.move_j_command_request_handler(
        move_j_command_model=retract_ct_01_to_target_ct_01_move_j_command_model)
    await ur_cobot.move_j_command_request_handler(
        move_j_command_model=target_ct_01_to_calibrate_ct_01_move_j_command_model)
    await ur_gripper.close_gripper()
    position = await ur_gripper.get_position()
    print(position)
    await ur_gripper.open_gripper()
    await ur_cobot.move_j_command_request_handler(
        move_j_command_model=calibrate_ct_01_to_target_ct_01_move_j_command_model)
    await ur_cobot.move_j_command_request_handler(
        move_j_command_model=target_ct_01_to_retract_ct_01_move_j_command_model)
    await ur_gripper.close_gripper()
    await ur_cobot.move_j_command_request_handler(
        move_j_command_model=retract_ct_01_to_home_move_j_command_model)
    if position == 18:
        return True
    else:
        return False


async def validate_cg_position(ur_cobot, ur_gripper):
    home_to_retract_cg_base_joint_position_model_array = [
        home_joint_position_model, retract_cg_base_joint_position_model]
    retract_cg_base_to_calibrate_cg_base_joint_position_model_array = [
        retract_cg_base_joint_position_model, calibration_cg_base_joint_position_model]

    calibrate_cg_base_to_retract_cg_base_joint_position_model_array = [
        calibration_cg_base_joint_position_model, retract_cg_base_joint_position_model]
    retract_cg_base_to_home_joint_position_model_array = [
        retract_cg_base_joint_position_model, home_joint_position_model]

    home_to_retract_cg_base_move_j_command_model = MoveJCommandModel.get_move_j_command_model_using_arguments(
        acceleration=0.5, velocity=0.5, time_s=0, blend_radius=0,
        joint_position_model_array=home_to_retract_cg_base_joint_position_model_array)
    retract_cg_base_to_calibrate_cg_base_move_j_command_model = MoveJCommandModel.get_move_j_command_model_using_arguments(
        acceleration=0.5, velocity=0.5, time_s=0, blend_radius=0,
        joint_position_model_array=retract_cg_base_to_calibrate_cg_base_joint_position_model_array)
    calibrate_cg_base_to_retract_cg_base_move_j_command_model = MoveJCommandModel.get_move_j_command_model_using_arguments(
        acceleration=0.5, velocity=0.5, time_s=0, blend_radius=0,
        joint_position_model_array=calibrate_cg_base_to_retract_cg_base_joint_position_model_array)
    retract_cg_base_to_home_move_j_command_model = MoveJCommandModel.get_move_j_command_model_using_arguments(
        acceleration=0.5, velocity=0.5, time_s=0, blend_radius=0,
        joint_position_model_array=retract_cg_base_to_home_joint_position_model_array)

    await ur_gripper.close_gripper()
    await ur_cobot.move_j_command_request_handler(
        move_j_command_model=home_to_retract_cg_base_move_j_command_model)
    await ur_gripper.open_gripper()
    await ur_cobot.move_j_command_request_handler(
        move_j_command_model=retract_cg_base_to_calibrate_cg_base_move_j_command_model)
    await ur_gripper.close_gripper()
    position = await ur_gripper.get_position()
    print(position)
    await ur_gripper.open_gripper()
    await ur_cobot.move_j_command_request_handler(
        move_j_command_model=calibrate_cg_base_to_retract_cg_base_move_j_command_model)
    await ur_gripper.close_gripper()
    await ur_cobot.move_j_command_request_handler(
        move_j_command_model=retract_cg_base_to_home_move_j_command_model)
    if position == 54:
        return True
    else:
        return False


if __name__ == '__main__':
    asyncio.run(cobot_calibration())
