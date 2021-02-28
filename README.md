# SDIR 2020 Controler
This code repository consists of two parts: the ctrl and the sim part. The ctrl folder contains all necessary code files for your controler. The sim folder contains the [VREP](https://www.coppeliarobotics.com/) .ttt file with all necessary implementation for simulation.

In this file I developed forward kinematics, inverse kinematics and lin movement file. If you run this code you will see lin movement perfecly working on KUKA robot. For observing the robot movement please open vrep and from vrep please open the scene file from sim folder. Open the ctrl folder on Microsoft visual studio or any other C++ ide which support memory management, otherwise you can get some build error.

After run the main.cpp from Visual studio, one GUI will be open from vrep where you have to press apply first then select lin movement and press move robot/run.


#Software requirement:

* Microsoft visual studio
Software download link: https://visualstudio.microsoft.com/downloads/

* vrep(Coppelia Robotics)
Software download link: https://www.coppeliarobotics.com/previousVersions




