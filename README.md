# teleoperation_ABB_interface

This repository is aimed to control throught a configurable controller, a robotic arm of yhe brand ABB via their own comunication metho EGM (External Guided Motion).

In these files you will find 3/4 main modules that can work separately, but they are designed to work together. Those are (Controller reader / controller display), (data traductor to EGM) and (Data process {RAPID})

(Controller reader / controller display):
These modules are aimed to collect in real time the input data of controllers and the subsecuent display of them to give the user some feedback; in this progect a space navigator and an xbox controller are used. You could use other controllers as long as you configurate properly its code to read and the sub-libraries to be sent to other modules.

(data traductor to EGM):
The traductor utilizes the sub-libraries of the previous stage to transform the into a operation file via the python libraries: abb_robot_client.egm // abb_motion_program_exec. The way it all works allows to operate the robotic arm via PoseMode (given a position the robot moves to it, so as we alter this position in real time the robot follows as well) and via JointMode (we altrt the angle value of the joints in a similar way)

(Data process {RAPID}):
The code of processing is developec fully on RAPID and in the RobotStudio environment of ABB, with a robotic controller IRC5 and a robotic arm IRB-12000. Meanwile other robotica arms with a simiar movemente architecture (DoF 6) could be suitable for the operation, the proyect have not been proven in other robotic controllers.
If, at some point, the arm passes under a singularity or it is set beyond its geometry or motor force, the program stops to prevent any mishap. The you are forced to reset the modules.

(Images and instructions on the setup yet to be setted)

WARNING: This way of operation (EGM) deactivates may active security protocols of the robot and the RobotStudio Software; if you are interested consider in adding some pasive security protocols tha fit your needs. So it is to be used in a physical environment with the appropriate equipment and safety procedures, and at your own risk.
