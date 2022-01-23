/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static XboxController controllerDrive = new XboxController(0);
  public static XboxController controllerSecondary = new XboxController(1);

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public Compressor compressor = new Compressor(50);

  //private UsbCamera cam0;
  //private SerialPort cam0_ser;

  public static double camX = 320.0;
  public static double camY = 0.0;

  public static int flywheelVel = 0;
  public static int lastFlywheelVel = 0;
  public static int deltaFlywheelVel = 0;

  public static boolean shooting = false;

  public static double hoodTarget = 0.0;
  public static double manualHoodOffset = 0.0;

  public static int lastPOV = -1;

  public static int turretError = 100;

  private HttpCamera LLFeed;	

  //public NetworkTableEntry camXDashboard = Shuffleboard.getTab("Default").add("Camera X", 160.0).getEntry();
  //public NetworkTableEntry camYDashboard = Shuffleboard.getTab("Default").add("Camera Y", 0.0).getEntry();

  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = limelightTable.getEntry("tx");
  NetworkTableEntry ty = limelightTable.getEntry("ty");
  NetworkTableEntry ta = limelightTable.getEntry("ta");
  NetworkTableEntry tv = limelightTable.getEntry("tv");


  public NetworkTableEntry camXDashboard = Shuffleboard.getTab("Default").add("Camera X", 0.0).getEntry();
  public NetworkTableEntry camYDashboard = Shuffleboard.getTab("Default").add("Camera Y", 0.0).getEntry();
  public NetworkTableEntry camADashboard = Shuffleboard.getTab("Default").add("Camera Area", 0.0).getEntry();
  public NetworkTableEntry camVDashboard = Shuffleboard.getTab("Default").add("Visible Target", 0.0).getEntry();

  public NetworkTableEntry RightBumperPrimary = Shuffleboard.getTab("Default").add("RightBumper - Primary", false).getEntry();
  public NetworkTableEntry LeftBumperPrimary = Shuffleboard.getTab("Default").add("LeftBumper - Primary", false).getEntry();
  public NetworkTableEntry RightBumperSecondary = Shuffleboard.getTab("Default").add("RightBumper - Secondary", false).getEntry();
  public NetworkTableEntry LeftBumperSecondary = Shuffleboard.getTab("Default").add("LeftBumper - Secondary", false).getEntry();

  public NetworkTableEntry RightTriggerPrimary = Shuffleboard.getTab("Default").add("RightTrigger - Primary", 0.0).getEntry();
  public NetworkTableEntry LeftTriggerPrimary = Shuffleboard.getTab("Default").add("Leftrigger - Primary", 0.0).getEntry();

  public NetworkTableEntry RightTriggerSecondary = Shuffleboard.getTab("Default").add("RightTrigger - Secondary", 0.0).getEntry();
  public NetworkTableEntry LeftTriggerSecondary = Shuffleboard.getTab("Default").add("Leftrigger - Secondary", 0.0).getEntry();
  

  public NetworkTableEntry flywheelVelocityDashboard = Shuffleboard.getTab("Default").add("Flywheel Velocity", 0.0).getEntry();

  public NetworkTableEntry hoodTargetDashboard = Shuffleboard.getTab("Default").add("Hood Target", 0.0).getEntry();
  public NetworkTableEntry manualHoodOffsetDashboard = Shuffleboard.getTab("Default").add("Manual Hood Offset", 0.0).getEntry();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboaard.
    m_robotContainer = new RobotContainer();

    ShuffleboardTab dashboardTab = Shuffleboard.getTab("Dash");	
    LLFeed = new HttpCamera("limelight", "http://10.31.96.11:5800/stream.mjpg");	
    dashboardTab.add("LimeLight", LLFeed);	
    
    /*

    //cam0 = CameraServer.getInstance().startAutomaticCapture();

    try {
     // cam0_ser = new SerialPort(115200, SerialPort.Port.kUSB);
    } catch(Exception e) {
      //System.out.println(e.toString());
    }
    */

    compressor.clearAllPCMStickyFaults();
    //compressor.getCompressorCurrent();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    double LimelightX = tx.getDouble(0.0);
    double LimelightY = ty.getDouble(0.0);
    double LimelightArea = ta.getDouble(0.0);

    
    /*String data = "";
    if(cam0_ser != null) {
      data = cam0_ser.readString();
    }
    if(!data.equals("")) {
      //System.out.println(data);
      try {
        String numData = data.substring(32, data.length() - 2);

        int commaIndex = numData.indexOf(",");
        String xStr = numData.substring(0,commaIndex);
        String yStr = numData.substring(commaIndex+1);

        //camX = Double.parseDouble(xStr);
        //camY = Double.parseDouble(yStr);
        //System.out.println(camX);
      } catch(Exception e) {
        System.out.println(e);
      }
    }
    */

    camXDashboard.setDouble(LimelightX);	
    camYDashboard.setDouble(LimelightY);	
    camADashboard.setDouble(LimelightArea);

    RightBumperPrimary.setBoolean(Robot.controllerSecondary.getBumper(Hand.kLeft));
    LeftBumperPrimary.setBoolean(Robot.controllerSecondary.getBumper(Hand.kLeft));

    RightBumperSecondary.setBoolean(Robot.controllerSecondary.getBumper(Hand.kLeft));
    LeftBumperSecondary.setBoolean(Robot.controllerSecondary.getBumper(Hand.kLeft));

    RightTriggerPrimary.setDouble(Robot.controllerSecondary.getTriggerAxis(Hand.kLeft));
    LeftTriggerPrimary.setDouble(Robot.controllerSecondary.getTriggerAxis(Hand.kLeft));
    RightTriggerSecondary.setDouble(Robot.controllerSecondary.getTriggerAxis(Hand.kLeft));
    LeftTriggerSecondary.setDouble(Robot.controllerSecondary.getTriggerAxis(Hand.kLeft));

    
    flywheelVelocityDashboard.setDouble(flywheelVel);

    hoodTargetDashboard.setDouble(hoodTarget);
    manualHoodOffsetDashboard.setDouble(manualHoodOffset);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    limelightTable.getEntry("camMode").setNumber(0);
    limelightTable.getEntry("ledMode").setNumber(0);
  }

  @Override
  public void disabledPeriodic() {
    limelightTable.getEntry("ledMode").setNumber(0);
    limelightTable.getEntry("camMode").setNumber(0);
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    limelightTable.getEntry("camMode").setNumber(0);
    limelightTable.getEntry("ledMode").setNumber(0);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    limelightTable.getEntry("camMode").setNumber(1);
    limelightTable.getEntry("ledMode").setNumber(1);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    //System.out.println("Teleop Works");
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    limelightTable.getEntry("camMode").setNumber(0);
    limelightTable.getEntry("ledMode").setNumber(2);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
