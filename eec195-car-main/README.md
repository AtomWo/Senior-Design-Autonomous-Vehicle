# EEC 195 Senior Design Project

## TODO:

-   [ ] Normalize errors (from -1 to 1 ideally, then scale proportionally when determining steering values)
-   [ ] Control speed based on error: Closer to 0 = More Speed, Closer to 1/-1 = Less Speed
-   [ ] Fine-tune PID values, error weights (i.e. line-center error vs. angle error)
    -   [Good guide](https://thecodingfun.com/2020/06/16/lego-mindstorms-ev3-pid-line-follower-code-by-using-micropython-2-0/) on how to determine values

EEC 195: Autonomous Vehicle Design Final Report
Team 15
Donggeon Kim, Joseph Melman, Hanson Nguyen, Adam Wong


Table of Contents
Project Objective 
3
System Block Diagram
3-5
Hardware Documentation
6-10
Software Documentation
10-12
Design and Performance Summary
12-14
References / Acknowledgement of Sources
15













Project Objective

The goal of this project is to design, fabricate, and test a high-performance printed circuit board (PCB) for an autonomous racecar using Altium Designer. We want to develop a lane-following and speed-control algorithm using the OpenMV microcontroller and camera to ensure optimal speed, reliability, and sensor integration for the racecar. The project will involve schematic capture, PCB layout, signal integrity analysis, and manufacturing preparation, integrated in the assembly and testing of the board in a competitive racing environment. 

System Block Diagram

Part 1/3 of System Block Diagram

Part 2/3 of System Block Diagram

Part 3/3 of System Block Diagram





Hardware Documentation

Altium Circuit Schematic




Altium PCB Design and Layout

The different control mechanisms of the car are separated into three parts on our PCB: the DC motor control at the top of the board, servo motor control in the middle, and the OpenMV interfacing at the bottom of the board. Each of these sections has its own ground plane and all three ground planes are joined together at a star ground at the battery terminal. The goal of this layout is to reduce cross-board wiring and to minimize noise between the signals sent to the DC and servo motors. 

Mechanical Design Layout of 3D Arm and Platform
Bambuu Lab Layout for 3D Printing

Implementation of 3D Arm with OpenMV Camera
Mechanically, we designed and iterated on the 3D-printed arm that holds our OpenMV camera. To streamline debugging and field testing, we added a removable slot module, so we could easily take the camera on and off without disassembling the vehicle. We also integrated screw-in adjustable hinges that let us fine-tune the camera angle and tighten it into position. That added both reliability and stability, ensuring peak performance during movement. Finally, we also revised the original open-source arm design to optimize the overhead visibility of the track, which improved how well the camera could detect line features — especially at curves or intersections.
Software Documentation
Core Algorithm:
The autonomous vehicle employs a computer vision–based lane-following algorithm that integrates line detection with PID control for precise navigation. The main modules incorporated in our design are linear regression, speed control, and PID control. The OpenMV camera captures grayscale frames in its field of view, converts the pixels to a binary black or white, and uses two predefined regions of interest (ROIs) to identify lane boundaries via linear regression. When both lane lines are detected, their endpoints are averaged to derive a “middle line” that defines the desired trajectory; if only one line is visible, the middle line is estimated by offsetting from the single detected boundary. The system then computes two error metrics—lateral deviation from the image center and deflection angle of the middle line (calculated in functions line_center_error and deflection_angle_error). These error values are normalized and combined into a single control error in the function combined_error. The two metrics have different effects on the overall error depending on the weight they’re assigned. Optimizing these error weights was one of the main parts of our testing process and we found that emphasizing the error of the line center significantly improved the vehicle’s responsiveness.

Error Calculation

Steering and speed control based on error

Fine-tuned parameters for PID control for steering and speed
PID:
The PID control of the car was another of our main responsibilities; tuning the PID controller directly affected how smoothly and accurately our car followed the track. Within our algorithm modules, we adjusted three key parameters: KP for proportional error, KI for accumulated error over time, and KD for how quickly we respond to changes. These values are used in the steer_pid function which determines how much to adjust the PWM of the servo.
Design and Performance Summary
Initially, our car was either too twitchy or would make accidental turns on well-defined angles, so we went through multiple iterations of these values. We ran controlled test loops, logged steering output, and observed the car’s reaction to both smooth curves and sharp deviations. From our testing we also realized that our regions of interest for blob detection were too far ahead in vision, given the wide-lens angle that created a linear perspective. In linear perspective, parallel lines appear to converge at a single point on the horizon line called the vanishing point. We therefore had to crop the OpenMV’s field of vision to focus solely on the overhead view of the lane. Ultimately, from many revisions of both the hardware and software sides of the car, we achieved a very favorable result. From this, you can see from the leaderboard, we were able to attain the fastest time out of 20 teams to earn 1st place in the competition.

Achieved #1 in Fastest Time for 2 Laps

From our findings, we refined KP to give just enough correction without overshooting, kept KI low to avoid buildup, and adjusted KD to dampen sudden movements.
This tuning process drastically improved performance — especially during high-speed turns where precision matters most.







References / Acknowledgement of Sources
Open Source 3D Arm - https://www.printables.com/model/1153921-multifunctional-arm-phone-holder/files#preview.file.HSOaE
H-Bridge Motor Control Reference Circuit - 
https://www.pololu.com/product/1451


