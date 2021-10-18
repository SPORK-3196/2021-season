package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import org.ejml.equation.IntegerSequence.For;

// import frc.robot.Robot;
import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
    public CANSparkMax leftClimberMotor = new CANSparkMax(14, MotorType.kBrushless);
    public CANSparkMax rightClimberMotor = new CANSparkMax(13, MotorType.kBrushless);

    public static Solenoid climberSolenoid = new Solenoid(50, 4);

    public Climber() {
      Climber.climberSolenoid.set(false);
    }
    

    public void runRightMotorForward(double forwardPower) {
      rightClimberMotor.set(forwardPower);
    }
    public void runLeftMotorForward(double forwardPower) {
      leftClimberMotor.set(forwardPower);
    }
    public void runLeftMotorBackward(double backwardPower) {
      leftClimberMotor.set(-1 * backwardPower);
    }
    public void runRightMotorBackward(double backwardPower) {
      rightClimberMotor.set(-1 * backwardPower);
    }
    public void stopMotors() {
        leftClimberMotor.set(0.0);
        rightClimberMotor.set(0.0);
    }

    public void runBothMotorsForward(double forwardPower) {
      leftClimberMotor.set(forwardPower);
      rightClimberMotor.set(forwardPower);
    }

    public void runBothMotorsBackward(double backwardPower) {
      leftClimberMotor.set(-1 * backwardPower);
      rightClimberMotor.set(-1 * backwardPower);
    }


    public void RaiseArms(){
      climberSolenoid.set(true);
    }

    public void LowerArms(){
      climberSolenoid.set(false);
    }

}