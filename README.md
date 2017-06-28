
# robot_hoover
DIY Robotic Hoover, made from CNC plywood, Arduino and cheap DC geared motors.

Developing project based on Arduino as the Controller, cnc cut plywood parts, 2x geared 6v DC motors and some 3d printed parts

Using an old PC CPU fan for the suction fan, best option here is to use as high an air flow fan as possible.

#  Overview

So I like hacking things together, and have built many projects as a learning and experimenting experience. This project came after seeing a friends very expensive Dyson Robot hoover. I thought "how hard can it be" to make one... 

I luckily have access to my own CNC and 3d Printing machines which mean I can whip up parts myself, but anyone with some basic hand tools could make this in their shed or garage. (or back bedroom as I do)

So what are the goals, make a small autonomous hoover that will collect the cat hair and small dust particles in my lounge and kitchen.

Key parts 

1. Build a platform, easy enough with some old 6mm plywood
2. Arduino based control, have loads of these laying around
3. Battery powered, using some RC model LiPo batteries i have
4. Suction, this is the biggest hack part. I bought a fan off ebay, but it was useless. Old PC CPU fan works best
5. Motors, 2x cheap geared dc motors off well known online shopping site.
6. Avoid getting stuck in the corner.

The key is handling the robot movement to avoid objects, and handle the situation where it has become stuck.
This is done in combination with 3 sensor inputs and some basic counting in the code.

Firstly there are 2 x HC-SR04 Sonar detectors, to be honest 3 would work better as there is a dead spot in the middle. But this is handled by the third sensor. The wheel has an encoder, which checks that when forward movement is requested that the wheel is actually turning. So if the unit gets stuck, the robot can react and try and unstick itself. 
Finally if the robot had tried all its tricks but the wheels are still not rotating then shut down so as not to burn out the gears or flatten the battery.

The unit actually works pretty well, I use it on hardwood floor, but its does move around on carpet areas ok.

Plenty more code improvements to be made as I continue to test the unit in action.

# CREDITS 

Uses the NewPing.h library from http://playground.arduino.cc/Code/NewPing (copy under source folder)
