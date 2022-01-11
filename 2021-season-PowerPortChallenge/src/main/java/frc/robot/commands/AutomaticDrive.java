/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.ControlType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Turret;

public class AutomaticDrive extends CommandBase {

  private final Drivetrain drivetrain;
  private final Turret turret;
  private final Flywheel flywheel;


  public Timer autoTimer = new Timer();
  public double time = 5.0;

  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTable robotTable = NetworkTableInstance.getDefault().getTable("Default");


  
  boolean shootDuringAuto = robotTable.getEntry("Shoot during auto?").getBoolean(false);


  /**
   * Creates a new DriveWithJoystick.
   */
  public AutomaticDrive(Drivetrain p_drivetrain, double p_time, Turret newTurret, Flywheel newFlywheel) {
    // Use addRequirements() here to declare subsystem dependencies.
    time = p_time;
    drivetrain = p_drivetrain;
    turret = newTurret;
    flywheel = newFlywheel;
    addRequirements(drivetrain);
    addRequirements(turret);
    addRequirements(flywheel);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.drivetrain.setDeadband(0.1);

    drivetrain.frontRight.setNeutralMode(NeutralMode.Coast);
    drivetrain.rearRight.setNeutralMode(NeutralMode.Coast);
    drivetrain.frontLeft.setNeutralMode(NeutralMode.Coast);
    drivetrain.rearLeft.setNeutralMode(NeutralMode.Coast);
    autoTimer.reset();
    autoTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double tx = limelightTable.getEntry("tx").getDouble(0.0);
    double ty = limelightTable.getEntry("ty").getDouble(0.0);
    double ta = limelightTable.getEntry("ta").getDouble(0.0);
    double tv = limelightTable.getEntry("tv").getDouble(0.0);
   
    double aimControlConstant = -0.1;
    double distanceControlConstant = -0.1;
    double min_aim_command = 0.1;

    System.out.println(tv);
    
    
    /*
    boolean cool = Robot.controllerDrive.getXButton();
    Drivetrain.driveCooler.set(cool);
    */

    if (tv == 0)
    {
      drivetrain.drivetrain.arcadeDrive(0, 0.15);
      
    } else {

        double heading_error = -1 * tx;
        double steering_adjust = 0.0;
  
        if (tx > 1.0)
        {
          steering_adjust = aimControlConstant * heading_error - min_aim_command;
      }
        else if (tx < 1.0)
        {
          steering_adjust = aimControlConstant * heading_error + min_aim_command;
          //turret.hoodPID.setReference(9.5, ControlType.kPosition); //1.8 originally, changed to 2.0, changed to 2.2, changed 2.3, changed 2.5, changed 2.8, changed 3.0
          //Robot.shooting = true;
        }
        
        double leftInput = 0 + steering_adjust;
        double rightInput = 0 - steering_adjust;

        drivetrain.drivetrain.tankDrive(leftInput, rightInput);
    }

    if(Robot.shooting) {
        if(Robot.flywheelVel < Flywheel.velocityTarget) {
          flywheel.flywheel1.set(1.0);
        } else {
          flywheel.flywheel1.set(0.925);//og 0.95
        }
        turret.hoodPID.setReference(9.5, ControlType.kPosition); //1.8 originally, changed to 2.0, changed to 2.2, changed 2.3, changed 2.5, changed 2.8, changed 3.0
        Robot.shooting = true;
    } else {
        flywheel.flywheel1.set(0.0);
    }

    
    Drivetrain.falconTempDashboard[0].setDouble(drivetrain.frontLeft.getTemperature());
    Drivetrain.falconTempDashboard[1].setDouble(drivetrain.rearLeft.getTemperature());
    Drivetrain.falconTempDashboard[2].setDouble(drivetrain.frontRight.getTemperature());
    Drivetrain.falconTempDashboard[3].setDouble(drivetrain.rearRight.getTemperature());

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return autoTimer.get() >= time;
  }
}
