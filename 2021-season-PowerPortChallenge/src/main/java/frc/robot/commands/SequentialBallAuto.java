/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SequentialBallAuto extends SequentialCommandGroup {
  /**
   * Creates a new FiveBallAuto.
   */
  public SequentialBallAuto(Turret p_turret, Flywheel p_flywheel, Index p_index, Drivetrain p_drivetrain) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
     // new DriveForwardTimed(p_drivetrain, 2, 0.6),
      new AutomaticDrive(p_drivetrain, 4.0, p_turret, p_flywheel),
      new ShootUpclose(p_turret, p_flywheel, p_index),
      new DriveForwardTimed(p_drivetrain, 3, -0.6)
    );
  }
}
