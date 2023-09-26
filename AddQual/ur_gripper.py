from URGripper.ur_gripper_controller import URGripperController


class URGripper:

    def __init__(self):
        self.ur_gripper_controller = None

    async def connect_ur_gripper_physical_device(self):
        self.ur_gripper_controller = URGripperController()
        self.ur_gripper_controller.connect(
            hostname="10.2.12.109",
            port=63352,
            socket_timeout=2.0)
        self.ur_gripper_controller.activate()

    async def connect(self):
        await self.connect_ur_gripper_physical_device()

    async def open_gripper(self):
        try:
            self.ur_gripper_controller.open_gripper()
            return True
        except Exception as ex:
            return False

    async def close_gripper(self):
        try:
            self.ur_gripper_controller.close_gripper()
            return True
        except Exception as ex:
            return False

    async def get_position(self):
        try:
            position_value = self.ur_gripper_controller.get_position()
            return position_value
        except Exception as ex:
            return -1

    async def is_object_detected(self):
        try:
            is_object_detected = self.ur_gripper_controller.get_object_detection()
            return is_object_detected
        except Exception as ex:
            return False
