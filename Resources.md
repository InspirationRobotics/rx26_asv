# General information
This project is for the USV for RobotX 2026 challenge. 
RobotX website: https://robotx.org/programs/2026/
RobotX rulebook: https://robonation.gitbook.io/robotx-2026-team-handbook 

Please read through the rulebook. We are aiming for disruptive tier for all mission, add remember everything that's relevant to UUV. 
# Hardware information
Transmitter RadioMaster Pocket: https://radiomasterrc.com/products/pocket-radio-controller-m2
Reserver RP3 Rx: https://radiomasterrc.com/products/rp3-expresslrs-2-4ghz-nano-receiver?variant=46486353674432
Pixhawk: https://www.amazon.com/HAWKS-WORK-Pixhawk-Controller-Absorber/dp/B0CTZTJD4J?th=1
Jetson Orin: https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/
GPS Module: https://store.beitian.com/products/beitian-built-in-zed-f9p-navigation-surveying-positioning-precision-agriculture-centimeter-level-rtk-gnss-module?variant=44850918097183
Beitian BT-800S Survey Antenna:https://www.beitian.com/en/sys-pd/1138.html
ESC: https://bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/
Analog Digital Converter: https://www.pololu.com/product/2801/specs
Bullet AC: https://store.ui.com/us/en/products/bulletac-ip67
MID-360 LiDar: https://www.livoxtech.com/mid-360
OAKD-LR Stereo Camera: https://shop.luxonis.com/products/oak-d-lr
Arduino Nano: https://store.arduino.cc/products/arduino-nano
Circuit Breaker: https://www.amazon.com/dp/B06Y4172LP?smid=ALHXFZX1L0DAX&th=1
Relay: https://www.amazon.com/IRHAPSODY-120Amp-Continuous-Terminal-Starter/dp/B07T35K8S2?th=1&psc=1
Emergency Stop Red Button: https://www.mcmaster.com/6785K21/

# Documentations
Operating the boat: https://docs.google.com/document/d/1ZIznDeolk2HxLp1HP5z8Sk3bXnA-rvEsOOBvd9w-0cg/edit?usp=sharing

# Safety System
There's a relay that controls the power to ESCs and it requires the magnetic coil to be powered to have the power deliverd. On the powerline for the magnetic coil, we have the following to cut the powerline: 
1. Emergency Stop Red Button
2. CH5 from pixhawk --> Analog to Digital Converter --> Relay, we are using the relay as a remote emergency stop
Either will cut the power to ESCs

# Device information
Jetson WIFI IP: 192.168.100.109
Jetson ETH IP: 192.168.1.5
LiDar IP: 192.168.1.166

# Mounting Information
LiDar(LiDar is mounted upside down, so its +z axis is downward instead of updward):
    Orientation:
        +x = forward
        +y = left
        +z = downward
    Displacement from vehicle geometry center:
        x = 32 cm
        y = 5 cm
    Height above ground:
        z = 52 cm
OAK D LR Camera:
    Orientation:
        +x = forward
        +y = left
        +z = upward
    Displacement from vehicle geometry center:
        x = 37 cm
        y = 0
    Height above ground:
        z = 65 cm



