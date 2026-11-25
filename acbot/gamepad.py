"""Virtual Xbox 360 controller output (needs ViGEmBus + `pip install vgamepad`), with a dry-run fallback."""


class DryRunPad:
    def __init__(self):
        self.last = None

    def send(self, steer, throttle, brake):
        self.last = (steer, throttle, brake)

    def close(self):
        pass


class VirtualPad:
    def __init__(self):
        import vgamepad as vg
        self._pad = vg.VX360Gamepad()

    def send(self, steer, throttle, brake):
        self._pad.left_joystick_float(x_value_float=max(-1.0, min(1.0, steer)), y_value_float=0.0)
        self._pad.right_trigger_float(value_float=max(0.0, min(1.0, throttle)))
        self._pad.left_trigger_float(value_float=max(0.0, min(1.0, brake)))
        self._pad.update()

    def close(self):
        self.send(0.0, 0.0, 1.0)


def make_pad(dry_run=False):
    if dry_run:
        return DryRunPad()
    return VirtualPad()
