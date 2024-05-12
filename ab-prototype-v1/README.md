# FOV Expansion Device

FOV Expansion Device Prototype Developed by Alexander Burovanov.

## Device User Manual (General Purpose)

This is a user operating manual for the FOV Expansion System developed by the STARS NRM FOV engineering team. The developed device is a type of camera controlled mirror system which expands the FOV of the user.

### 1. Getting Started

This project is an FoV expansion prototype for BIOE6901. To get started first ensure you have the following hardware ready:

TODO: add photo's

- Mirror Arm (with wiring harness attached)
- Raspberry Pi Control Unit + Attached Raspberry Pi Camera Module
- Rear Control Panel + Attached Camera Mount (with wiring harness attached)
- System Wiring Harness (Attached to Mirror Arm and Camera Mount)
- 2 x Mirror Arm Base Mounting Straps
- 2 x Control Panel Mounting Straps
- 1 x XBOX One Bluetooth Controller
- 1 x USB-C Power Cable

*Note: these instructions assume that the latest version of the rpi-fov controller firmware have already been flashed to the device. For help flashing new firmware, see xxxx.*

### 2. Assembly/Mounting

To assemble the device and mounting it to the chair follow the following steps:

1. Take the Mirror Arm Base Plate and secure it to the chair armrest or rail using the two provided arm mounting straps.
2. Insert the Mirror Arm into the base plate. *NOTE: this mirror arm has the mirror and the wiring harness attached to it, it should be handled with caution.*
3. Place the rear chair control panel board over the back rest of the wheelchair. *Note: that this control panel has the camera mount and controller mount already pre-installed.*
4. Using the provided rear panel mounting straps, secure the rear control panel to the chair. Note, depending on the type of chair being mounted to you may not require both straps.
5. Mount the raspberry pi control unit to the rear control panel by sliding it into the rails located on the left of the control panel.
6. Take the camera module (attached to the Raspberry Pi control unit) and slide it into the white housing unit located at the top of the camera mount. *Note: the direction matters here, please ensure the black ribbon connector is protruding from the back of the camera mount!*
7. Connect the wiring harness (attached to the mirror arm and the camera mount) to the raspberry pi control unit if it is not already connected. If the device is already connected please **DOUBLE CHECK** that the device connections match the following image.
8. Connect the USB-C power cable to the Raspberry Pi Control Unit.
9. Unfortunately, due to the custom installation nature of this device, we have had to allow for additional slack within the wiring harness so it can be mounted on different types of wheelchairs. Use the provided reusable cable ties to improve the wire management and securely mount the wiring harness to the device.

At this stage the device is now securely mounted to the device and is ready for Power on, see [3. Powering On The Device](#3-powering-on-the-device).

### 3. Powering on the Device

In order to power on the device follow the following steps:

1. Power on the Xbox One Bluetooth Controller by pressing the XBOX logo for 1-2 seconds. The device logo will then begin to flash.
2. Power on the FOV Expansion device by pressing the power button on the USB-C power cord. Ensure that power has been provided to the device. To check that the device is powered on successfully, a solid red light status indicator light will appear on the top right of the Raspberry Pi Control Unit.
3. Wait for the device to boot and the XBOX Controller to connect. This process usually takes 20-60 seconds. Once the controller is connected the XBOX Logo light will stop flashing and remain on.
4. If the controller did not connect successfully or the device did not boot, please repeat steps 1-3 before proceeding.
5. The device is now ready for operation. See [4. Operating the Device](#4-operating-the-device).

### 4. Operating the Device

The FOV Expansion device is operated by the Xbox One Bluetooth Controller. The device has two operation modes manual mode and automatic mode.

#### Manual Mode

When first powering on the FOV Device, by default you are put into manual mode. Manual mode allows you to control the position of the mirror using the Left Joystick on the Xbox One Controller. When manually controlling the mirror there are two movement options `Increment` and `Position`. By default the device will be in `Increment` mode, meaning that when you move the joystick the mirror moves in increments and remains in a fixed position. `Position` mode is the opposite of this where the mirror moves to the current position of the joystick at all times. This means that if you move the joystick to the left the mirror will rotate left, then when you release the joystick back into the centre the mirror will follow it back tot he centre. You can press `B` on the controller, to toggle between `Increment` and `Position` movement controls.

In the current configuration of the device, there is not much merit to being able to move the camera system, however to ensure compatibility with other output systems such as a screen, the user also has the ability to move the position of the camera system using the joystick. To toggle between moving the Camera system and the Mirror, press `Y` on the controller.

*Note: the movement mode selected will persist when toggling between Camera and Mirror control.*

#### Automatic Mode

The FOV Expansion device also has an automatic control mode, this is where the camera system mounted to the top of the chair can be used to automatically control the position of the mirrors. This is done by tracking the faces of individuals you are interacting with. To toggle between `Automatic` and `Manual` operation modes press the `A` button on the controller. Please note, once you are in `Automatic` mode, if you move the joystick, you will automatically be placed back into `Manual` mode.

*WARNING: This is a prototype, automatic mode is not perfect. Please use it under advisement and revert to manual mode as required.*

### 5. Powering off the Device

To power off the device follow the below steps:

1. Press the power button on the USB-C Power Cable.
2. Once the device is fully powered off, the red status indicator on the Raspberry Pi will be off.
3. Note, that the XBOX Controller will automatically power itself off now that it is no longer connected to the control unit.

### 6. Dismounting & Storing the device

To dismount and store the FOV Expansion device please follow these instructions -

*WARNING: in the current prototype, various measures have been taken to ensure the portability of the device. However, it is still a prototype so extreme caution should be taken when dismounting the device.*

1. Enure that the device has been successfully powered off. See [5. Powering off the Device](#5-powering-off-the-device).
2. Disconnect the USB-C Cable power cable from the device.
3. Here you have the option to (A) store the control panel, control unit & mirror arm together or (B) store the control panel & mirror arm together and the control unit and camera separately.

    For Option (A):

    1. Detach the control panel mounting straps from the chair.
    2. Remove the mirror arm from the base (done by pulling it out from the front of the chair) and collapse it using the 3 adjustable screw dials on the arm. *Note: the wiring harness is still attached to this arm so this must be done in close proximity to the chair.*
    3. Remove the Rear Control Panel from the chair. *Note, that this should have the camera, control unit, and wiring harness attached to it.*

    For Option (B):

    1. Disconnect the wiring harness from the raspberry pi control unit.
    2. Disconnect the camera from the camera mount by sliding the green camera module board out of the top of the white enclosure.
    3. Remove the Raspberry Pi control unit from the control panel by sliding it out of the rail mount.
    4. The Raspberry Pi control unit and the camera module can now be stored.
    5. Detach the control panel mounting straps from the chair.
    6. Remove the mirror arm from the base (done by pulling it out from the front of the chair) and collapse it using the 3 adjustable screw dials on the arm. *Note: the wiring harness is still attached to this arm so this must be done in close proximity to the chair.*
    7. Remove the Rear Control Panel from the chair. *Note, that this should have wiring harness attached to it.*
4. Detach the mirror arm base from the chair by loosening the two mounting straps.
5. Store the device in the medium of your choosing.

## Software User Manual (Engineers Only)

This manual explains the firmware architecture of the FOV Device. It is not exhaustive and is for the use of the developers only. Only perform these actions under the advisement of a STARS NRM FOV engineer.

## Running the Code

TODO: complete this section

## Raspberry Pi Control Unit Setup

TODO: complete this section
