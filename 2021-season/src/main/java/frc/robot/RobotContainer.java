/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.commands.DriveForwardTimed;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.FiveBallAuto;
//import frc.robot.commands.RunClimber;
//import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.commands.RunIndex;
import frc.robot.subsystems.Index;
import frc.robot.commands.RunTurret;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final Index index = new Index();
  private final Turret turret = new Turret();
  private final Flywheel flywheel = new Flywheel();
  //private final Climber climber = new Climber();

  private final DriveWithJoystick driveWithJoystick = new DriveWithJoystick(drivetrain);
  private final RunIndex runIndex = new RunIndex(index);
  private final RunTurret runTurret = new RunTurret(turret, flywheel);
  //private final RunClimber runClimber = new RunClimber(climber);

  private NetworkTableEntry autoSelect = Shuffleboard.getTab("Default").add("Shoot during auto?", true).getEntry();


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drivetrain.setDefaultCommand(driveWithJoystick);
    index.setDefaultCommand(runIndex);
    turret.setDefaultCommand(runTurret);
    //climber.setDefaultCommand(runClimber);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    /*if(autoSelect.getBoolean(true)) {
      return new FiveBallAuto(turret, flywheel, index, drivetrain);
    } else {
      */
      return new DriveForwardTimed(drivetrain, 1.0, -0.6);
    //}
  }
}
