### Changes this version:
- Only activate brake resistor if vint and vext are >6.5V. Prevents board from activating resistor if only usb powered and a fault reset loop
- Changed behaviour of direction enable and axis enable bits in set_effect report to always apply direction vector
    - Fix for Forza Motorsport

### Changes since v1.14.0
- Save TMC space vector PWM mode in flash. Should be usually on for BLDC motors if the star point is isolated.
- Allow using the motors flux component to dissipate energy with the TMC4671 instead of the brake resistor. May cause noticable braking in the motor but takes stress off the resistor.
- Axis speed limiter usable and saved in flash.
- Removed unused hall direction flash setting.
- Added local button pulse mode