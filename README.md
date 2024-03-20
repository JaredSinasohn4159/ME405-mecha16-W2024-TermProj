# ME405-mecha16-W2024-TermProj
This project was done as a final project for ME 405 at Cal Poly. The purpose of the project was to create a turret which would incorporate cotasking in order to read thermal signatures, aim a nerf gun, and fire. The turret would then be used to duel other students in a epic battle for extra credit. An image of our project can be seen below:

![IMG_8767](https://github.com/JaredSinasohn4159/ME405-mecha16-W2024-TermProj/assets/156977553/ce617cda-8093-448f-819f-440e21c61487)

  Our design consisted of a 3D printed base. The panning motor slides in to the side and face mounts to secure the position/ tighten the belt. The panning axis was created by a machined aluminum shaft. It has a step down from .75 to .5 inches and was hollowed out to reduce mass. The shaftis supported buy two oil embedded thrust washers. These are both flanged and pressed into the base. This ensured that there was minimal play in the panning axis, the most crtitical for accurate aiming. 36 tooth GT2 pulleys were made in CAD and press fit to both the yaw motor and aluminum shaft.The pitch axis (the uppy downy aim) attaches to the base shaft by the a 3D printed shaft collar. It feature a 1 to 7 gear ratio to drive the pitch. Additionally, the axis features stops so that we can zero off of them for turret startup. We also have a servo which pulls the gun turret. The mlx camera 90640 is depicted in the back, which was used for thermal imaging. It also has a simple 3D mount

  Coding was done using a cotasking. The tasks were broken up as camera reading, yaw motor, pitch motor, and trigger servo. Furthur details can be seen in the doxygen:
doxygen link: 
Additionally a high level task diagram can be seen below, showing the interactions between the camera, both motors, and servo.

![image](https://github.com/JaredSinasohn4159/ME405-mecha16-W2024-TermProj/assets/156977553/b2dc6569-9172-4f0b-9dd7-2af982d5c07e)

Each of these tasks had their own finite state machines.  These finite state machines can be found below.

![Notes_240319_212721](https://github.com/JaredSinasohn4159/ME405-mecha16-W2024-TermProj/assets/81637028/d604ddaa-7ede-40e4-bc78-3fd56f4f26e5)


Reflection:
  One of our biggest issues was allignment between the camera and the panning axis. If we were to repeat the project, we would mount directly to the base of the turret rather than having a seperate mount/ unit. Hardware implementation was also a big takeaway. Having all mechanical components ready to go as fast as possible is necessary for success. Any delays in these parts means less time to implement electronics and debug because there WILL be issues. The use of a voltage regulator is also quite handy and cheap solution for stepping down voltages from a singular source. We used it to get the 12 V DC power supply to a 6.8 V source for our servo since we needed the 12V for both our pitch and panning motors. One thing that worked really well was the machined aluminum panning axis. Having such a high precision part seemed like overkill, but many people had a lot of slopiness in their system which made whatever optimizations in coding useless. Mechanical componenet compliance should be minimized for implementing high precision control systems. 

