# 2021-season
The repository for our 2021 season code. This document outlines our code and explains the more complex processes in depth where it is preferred over in-code documentation, as well as giving a general overview of the project.

## Subsystems
This section outlines the subsystems of our robot, what control systems they have, and the commands which use them.

### Drivetrain
This subsystem outlines our robot's four-motor drivetrain. We use four Falcon 500 motors with integrated TalonFX motor controllers. These controllers are declared in the _Drivetrain_ class and bundled into two _SpeedControllerGroup_ objects: _left_ and _right_. These objects bind the two motors on each side together to prevent fighting between the motors in each gearbox. The two groups are then connected into a _DifferentialDrive_ object which handles the calculations used to translate joystick input to motor output.

### Index/Intake


### Turret


## Commands
This section outlines the commands of our robot, the subsystems they use, and the control logic they implement to create efficient function.

### DriveWithJoystick
This command controls the drivetrain subsystem by inputting the x and y values of the left joystick on our primary Xbox controller and passing them into the _arcadeDrive_ method of the _DifferentialDrive_ object. The input values are also multiplied by a coefficient slightly less that 1.0 which slows the drivetrain slightly to avoid jittering and jerky control.

### RunIndex


### RunTurret

