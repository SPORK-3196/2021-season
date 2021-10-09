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
    double LeftClimberArmUp = Robot.controllerDrive.getTriggerAxis(Hand.kLeft);
    double RightClimberArmUp = Robot.controllerDrive.getTriggerAxis(Hand.kRight);

    boolean RaiseClimberPistons = Robot.controllerDrive.getAButton();
    boolean LowerClimberPistons = Robot.controllerDrive.getBButton();
    
    boolean LeftClimberDown = Robot.controllerDrive.getBumper(Hand.kLeft);
    boolean RightClimberDown = Robot.controllerDrive.getBumper(Hand.kRight);
    

    if (RightClimberArmUp > 0.3) {
      climber.runRightMotorForward(0.5);
    }
    else if (LeftClimberArmUp > 0.3) {
      climber.runLeftMotorForward(0.5);
    }
    else if (LeftClimberDown) {
      climber.runLeftMotorBackward(0.5);
    }
    else if (RightClimberDown) {
      climber.runRightMotorBackward(0.5);
    }
    else if (RaiseClimberPistons) {
      climber.RaiseArms();
    }
    else if (LowerClimberPistons) {
      climber.LowerArms();
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
