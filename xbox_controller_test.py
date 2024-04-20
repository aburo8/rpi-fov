"""
A simple application to test a connected input device (in this case XBOX Controller)
"""
import evdev

for event in evdev.InputDevice("/dev/input/event2").read_loop():
    if event.type == evdev.ecodes.EV_ABS:
        if event.code == evdev.ecodes.ABS_X:
            print(f"Left Analog Stick: {event.value}")


