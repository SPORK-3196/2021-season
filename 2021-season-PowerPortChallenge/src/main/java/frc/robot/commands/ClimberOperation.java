/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Climber;
import com.ctre.phoenix.motorcontrol.InvertType;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class ClimberOperation extends CommandBase {

  Climber climber;

  /**
   * Creates a new RunFlywheel.
   */
  public ClimberOperation(Climber p_climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = p_climber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //climber.rightClimberMotor.follow(climber.leftClimberMotor);
    //climber.leftClimberMotor.setInverted(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ClimberArmUp = Robot.controllerDrive.getTriggerAxis(Hand.kLeft);
    double ClimberArmDown = Robot.controllerDrive.getTriggerAxis(Hand.kRight);

    int POV = Robot.controllerSecondary.getPOV();

    boolean RaiseClimberPistons = Robot.controllerDrive.getAButton();
    boolean LowerClimberPistons = Robot.controllerDrive.getBButton();
    
    boolean LeftClimberUp = Robot.controllerDrive.getBumper(Hand.kRight);
    boolean RightClimberUp = Robot.controllerDrive.getBumper(Hand.kLeft);
    

    if (ClimberArmUp > 0.3) {
      climber.runBothMotorsForward(0.5);
    }
    else if (ClimberArmDown > 0.3) {
      climber.runBothMotorsBackward(0.5);
    }
    else if (RaiseClimberPistons) {
      climber.RaiseArms();
    }
    else if (LowerClimberPistons) {
      climber.LowerArms();
    }
    else if (LeftClimberUp) {
      climber.runLeftMotorBackward(0.5);
    }
    else if (RightClimberUp) {
      climber.runRightMotorBackward(0.5);
    }
    else if (POV == 0) {
      climber.runRightMotorForward(0.5);
    }
    else if (POV == 180) {
      climber.runLeftMotorForward(0.5);
    }
    else {
      climber.stopMotors();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
