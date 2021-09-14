/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Turret;
import java.lang.Math;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

public class RunTurret extends CommandBase {

  private final Turret turret;
  private final Flywheel flywheel;

  /**
   * Creates a new RunTurret.
   */
  public RunTurret(Turret newTurret, Flywheel newFlywheel) {
    // Use addRequirements() here to declare subsystem dependencies.
    turret = newTurret;
    flywheel = newFlywheel;
    addRequirements(turret);
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*flywheel.flywheel2.follow(flywheel.flywheel1);
    flywheel.flywheel1.setInverted(true);
    flywheel.flywheel2.setInverted(InvertType.FollowMaster);

    //turret.setSetpoint(1640);

    turret.hoodPID.setP(0.03);
    turret.hoodPID.setI(0.0);
    turret.hoodPID.setD(0.0);
    turret.hoodPID.setOutputRange(-0.3, 0.3);*/
    turret.disable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double rightY = Robot.controllerSecondary.getY(Hand.kRight);
  
    double hoodValue = turret.hood.getEncoder().getPosition();
    turret.hoodPos.setDouble(hoodValue);
    /*turret.hoodPID.setP(turret.hoodP.getDouble(0.0));
    turret.hoodPID.setI(turret.hoodI.getDouble(0.0));
    turret.hoodPID.setD(turret.hoodD.getDouble(0.0));*/

    Robot.flywheelVel = (int) (flywheel.flywheel1.getSelectedSensorVelocity());

    boolean spinupFlywheel = Robot.controllerSecondary.getBumper(Hand.kLeft);

    if(spinupFlywheel || Robot.shooting) {
      //flywheel.setSetpoint(Flywheel.velocityTarget);
      //flywheel.enable();
      //flywheel.flywheel1.set(0.8);

      if(Robot.flywheelVel < Flywheel.velocityTarget) {
        flywheel.flywheel1.set(1.0);
      } else {
        flywheel.flywheel1.set(0.95);
      }
    } else {
      //flywheel.disable();
      flywheel.flywheel1.set(0.0);
    }

    boolean shootAgainstWall = Robot.controllerSecondary.getAButton();
    boolean shootFromLine = Robot.controllerSecondary.getXButton();
    boolean shootFromTrench = Robot.controllerSecondary.getYButton();

    int pov = Robot.controllerSecondary.getPOV();
    System.out.println(pov);
   /* if (pov == 0)
    {
      turret.hoodPID.setReference(9.3, ControlType.kPosition);
      Robot.shooting = true;
      System.out.println("pov = 0");
    } else if (pov == 2) {
      turret.hoodPID.setReference(9.3, ControlType.kPosition);
      Robot.shooting = true;
      System.out.println("pov = 2");
    }
    */
    if(pov == 0 && Robot.lastPOV != 0) {
      Robot.manualHoodOffset -= 0.2;
    } else if(pov == 180 && Robot.lastPOV != 180) {
      Robot.manualHoodOffset += 0.2;
    }
    //Robot.hoodTarget = (-0.0129 * Robot.camY) + 13.6 - 1.8 + Robot.manualHoodOffset;
    //Robot.hoodTarget = 9.4 + Robot.manualHoodOffset;

    if(shootAgainstWall) {
      turret.hoodPID.setReference(1.8, ControlType.kPosition);
      Robot.shooting = true;
    } else if(shootFromLine) {
      // 15 is max hood value
      turret.hoodPID.setReference(9.3, ControlType.kPosition);
      Robot.shooting = true;
    } else if(shootFromTrench) {
      // 15 is max hood value  11.0
      turret.hoodPID.setReference(9.0 + Robot.manualHoodOffset, ControlType.kPosition);
      Robot.shooting = true;
    } else {
      if(hoodValue > 0.25) {
        turret.hoodPID.setReference(0.0, ControlType.kPosition);
      } else {
        turret.hood.set(-0.025);
      }

      //turret.hood.set(-0.3*rightY);
      //turret.disable();
      Robot.shooting = false;
    }

    if(turret.hood.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen).get()) {
      turret.hood.getEncoder().setPosition(0.0);
    }
    
    Robot.deltaFlywheelVel = Robot.flywheelVel - Robot.lastFlywheelVel;
    Robot.lastFlywheelVel = Robot.flywheelVel;
    Robot.lastPOV = pov;
    turret.turretEncoderDashboard.setDouble(turret.getPWMPosition());

    double turretInput = Robot.controllerSecondary.getX(Hand.kLeft);
    double adjustedTurretInput = 0.0;

    if(turretInput > 0.03) {
      adjustedTurretInput = (0.4 * Math.log(turretInput)) + 1.0;
      if(adjustedTurretInput < 0.0) adjustedTurretInput = 0.0;
    } else if(turretInput < -0.03) {
      adjustedTurretInput = -((0.3 * Math.log(-turretInput)) + 1);
      if(adjustedTurretInput > 0.0) adjustedTurretInput = 0.0;
    }

    if(shootAgainstWall) {
      if(!turret.isEnabled()) {
        turret.setSetpoint(1640);
        turret.enable();
      }
      adjustedTurretInput = turret.lastTurretOutput;
    } else {
      if(turret.isEnabled()) {
        turret.disable();
      }
    }

    Robot.turretError = (int)turret.getController().getPositionError();
    //System.out.println();

    turret.turretInputDashboard.setDouble(turretInput);
    turret.adjustedInputDashboard.setDouble(adjustedTurretInput);

    turret.turret.set(adjustedTurretInput * 0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
