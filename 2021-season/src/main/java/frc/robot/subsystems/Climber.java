package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
    public CANSparkMax leftClimberMotor = new CANSparkMax(5, MotorType.kBrushless);
    public CANSparkMax rightClimberMotor = new CANSparkMax(6, MotorType.kBrushless);

    public static Solenoid climberSolenoid = new Solenoid(50, 4);

    public void runMotorsForward(double fowardPower) {
        leftClimberMotor.set(fowardPower);
        rightClimberMotor.set(fowardPower);
    }
    public void runMotorsBackward(double backwardPower) {
        leftClimberMotor.set(-1 * backwardPower);
        rightClimberMotor.set(-1* backwardPower);
    }
    public void stopMotors() {
        leftClimberMotor.set(0.0);
        rightClimberMotor.set(0.0);
    }

    public void spinClockwise(double spinClockwisePower) {
        boolean clockwiseSpin = Robot.controllerSecondary.getBumper(Hand.kRight);
        if(clockwiseSpin) {
            runMotorsForward(spinClockwisePower);
        }
      }

    public void spinCounterClockwise(double spinCounterClockwisePower) {
        boolean counterclockSpin = Robot.controllerSecondary.getBumper(Hand.kLeft);
        if(counterclockSpin) {
            runMotorsBackward(spinCounterClockwisePower);
        }
      }
}