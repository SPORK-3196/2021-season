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
import com.ctre.phoenix.motorcontrol.InvertType;

public class ClimberOperation extends CommandBase {

  Climber climber;

  /**
   * Creates a new RunFlywheel.
   */
  public ClimberOperation(Climber p_climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = p_clibmer;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.rightClimberMotor.follow(flywheel.leftClimberMotor);
    climber.leftClimberMotor.setInverted(true);
    climber.rightClimberMotor.setInverted(InvertType.FollowMaster);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double climberWinchForward = Robot.controllerSecondary.getTriggerAxis(Hand.kLeft);
    double climberWinchBackward = ;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
