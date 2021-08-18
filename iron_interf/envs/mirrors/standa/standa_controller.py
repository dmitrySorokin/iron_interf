from . import pyximc
import ctypes


def _open(lib):
    # Set bindy (network) keyfile. Must be called before any call to "enumerate_devices" or "open_device" if you
    # wish to use network-attached controllers. Accepts both absolute and relative paths, relative paths are resolved
    # relative to the process working directory. If you do not need network devices then "set_bindy_key" is optional.
    # In Python make sure to pass byte-array object to this function (b"string literal").
    lib.set_bindy_key('keyfile.sqlite'.encode("utf-8"))

    # This is device search and enumeration with probing. It gives more information about devices.
    probe_flags = pyximc.EnumerateFlags.ENUMERATE_PROBE + pyximc.EnumerateFlags.ENUMERATE_NETWORK
    enum_hints = b"addr="
    # enum_hints = b"addr=" # Use this hint string for broadcast enumerate
    devenum = lib.enumerate_devices(probe_flags, enum_hints)

    dev_count = lib.get_device_count(devenum)
    print("Device count: " + repr(dev_count))

    dev_ids = []

    controller_name = pyximc.controller_name_t()
    for dev_ind in range(0, dev_count):
        enum_name = lib.get_device_name(devenum, dev_ind)
        result = lib.get_enumerate_device_controller_name(devenum, dev_ind, ctypes.byref(controller_name))
        if result == pyximc.Result.Ok:
            print("Enumerated device #{} name (port name): ".format(dev_ind) + repr(
                enum_name) + ". Friendly name: " + repr(controller_name.ControllerName) + ".")

        open_name = lib.get_device_name(devenum, dev_ind)

        device_id = lib.open_device(open_name)
        dev_ids.append(device_id)

    # set_current(lib, device_id, 100)
    # set_feedback(lib, device_id, pyximc.FeedbackType.FEEDBACK_NONE)
    print(dev_ids)
    # return device_id
    return dev_ids

class StandaController(object):
    def __init__(self):
        self.lib = pyximc.lib
        self.device_ids = _open(self.lib)
        # self.set_current(1000)
        # self.set_feedback(pyximc.FeedbackType.FEEDBACK_NONE)
        # self.set_microstep_mode(pyximc.MicrostepMode.MICROSTEP_MODE_FRAC_256)
        # for device_id in range(2): # because we are currently have two axis on controller
        #     self.set_current(device_id, 1000)
        #     self.set_feedback(device_id, pyximc.FeedbackType.FEEDBACK_NONE)
        #     self.set_microstep_mode(device_id, pyximc.MicrostepMode.MICROSTEP_MODE_FRAC_256)

    def close(self):
        for device_id in self.device_ids:
            self.lib.close_device(ctypes.byref(ctypes.cast(device_id, ctypes.POINTER(ctypes.c_int))))

    def get_position(self, device_id):
        x_pos = pyximc.get_position_t()
        result = self.lib.get_position(self.device_ids[device_id], ctypes.byref(x_pos))
        return x_pos.Position, x_pos.uPosition

    def move_absolute(self, device_id, distance, udistance):
        result = self.lib.command_move(self.device_ids[device_id], distance, udistance)
        return self.wait_for_stop(device_id)

    def move_relative(self, device_id, distance, udistance):
        pos, upos = self.get_position(device_id)
        return self.move_absolute(device_id, pos + distance, upos + udistance)

    def wait_for_stop(self, device_id, interval=100):
        result = self.lib.command_wait_for_stop(self.device_ids[device_id], interval)
        return repr(result)

    def set_current(self, device_id, curr_value):
        eng = pyximc.engine_settings_t()
        result = self.lib.get_engine_settings(self.device_ids[device_id], ctypes.byref(eng))
        eng.NomCurrent = curr_value
        result = self.lib.set_engine_settings(self.device_ids[device_id], ctypes.byref(eng))
        return repr(result)

    def set_feedback(self, device_id, feedback_type):
        feedback = pyximc.feedback_settings_t()
        self.lib.get_feedback_settings(self.device_ids[device_id], ctypes.byref(feedback))
        feedback.FeedbackType = feedback_type
        result = self.lib.set_feedback_settings(self.device_ids[device_id], ctypes.byref(feedback))
        return repr(result)

    def get_speed(self, device_id):
        # Create move settings structure
        mvst = pyximc.move_settings_t()
        # Get current move settings from controller
        result = self.lib.get_move_settings(self.device_ids[device_id], ctypes.byref(mvst))
        # Print command return status. It will be 0 if all is OK
        return mvst.Speed

    def set_speed(self, device_id, speed):
        # Create move settings structure
        mvst = pyximc.move_settings_t()
        # Get current move settings from controller
        result = self.lib.get_move_settings(self.device_ids[device_id], ctypes.byref(mvst))
        # Print command return status. It will be 0 if all is OK
        # Change current speed
        mvst.Speed = int(speed)
        # Write new move settings to controller
        result = self.lib.set_move_settings(self.device_ids[device_id], ctypes.byref(mvst))
        # Print command return status. It will be 0 if all is OK
        return repr(result)

    def set_microstep_mode(self, device_id, mode):
        # Create engine settings structure
        eng = pyximc.engine_settings_t()
        # Get current engine settings from controller
        result = self.lib.get_engine_settings(self.device_ids[device_id], ctypes.byref(eng))

        # Print command return status. It will be 0 if all is OK
        # Change MicrostepMode parameter to MICROSTEP_MODE_FRAC_256
        # (use MICROSTEP_MODE_FRAC_128, MICROSTEP_MODE_FRAC_64 ... for other microstep modes)
        eng.MicrostepMode = mode
        # Write new engine settings to controller
        result = self.lib.set_engine_settings(self.device_ids[device_id], ctypes.byref(eng))
        # Print command return status. It will be 0 if all is OK
        return repr(result)

    def get_lib_version(self):
        sbuf = ctypes.create_string_buffer(64)
        self.lib.ximc_version(sbuf)
        return sbuf.raw.decode().rstrip("\0")

    def get_info(self, device_id):
        x_device_information = pyximc.device_information_t()
        result = self.lib.get_device_information(self.device_ids[device_id], ctypes.byref(x_device_information))
        if result == pyximc.Result.Ok:
            return {
                " Manufacturer": repr(ctypes.string_at(x_device_information.Manufacturer).decode()),
                " ManufacturerId: ":  repr(ctypes.string_at(x_device_information.ManufacturerId).decode()),
                " ProductDescription: ":  repr(ctypes.string_at(x_device_information.ProductDescription).decode()),
                " Major: ": repr(x_device_information.Major),
                " Minor: ": repr(x_device_information.Minor),
                " Release: ": repr(x_device_information.Release)
            }

        return None

    def get_status(self, device_id):
        x_status = pyximc.status_t()
        result = self.lib.get_status(self.device_ids[device_id], ctypes.byref(x_status))
        if result == pyximc.Result.Ok:
            return {
                "Status.Ipwr: " + repr(x_status.Ipwr),
                "Status.Upwr: " + repr(x_status.Upwr),
                "Status.Iusb: " + repr(x_status.Iusb),
                "Status.Flags: " + repr(hex(x_status.Flags))
            }
        return None

    def get_serial(self, device_id):
        x_serial = ctypes.c_uint()
        result = self.lib.get_serial_number(self.device_ids[device_id], ctypes.byref(x_serial))
        if result == pyximc.Result.Ok:
            return repr(x_serial.value)
        return None
