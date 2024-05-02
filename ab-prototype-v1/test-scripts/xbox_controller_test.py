"""
A simple application to test a connected input device (in this case XBOX Controller)
"""
import evdev

EVENT = 4

for event in evdev.InputDevice(f"/dev/input/event{EVENT}").read_loop():
    if event.type == evdev.ecodes.EV_ABS:
        if event.code == evdev.ecodes.ABS_X:
            print(f"Left Analog Stick: {event.value}")


# BLUE XBOX REMOTE - C8:3F:26:AC:D5:24
