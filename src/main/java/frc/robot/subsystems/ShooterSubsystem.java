// package frc.robot.subsystems;
// import com.revrobotics.CANSparkFlex;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.PWM;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotConstants;

// public class ShooterSubsystem extends SubsystemBase {
    
//     CANSparkFlex shooterMotor1;
//     CANSparkFlex shooterMotor2;
//     PWM shooterServo;
//     public DigitalInput sensor;


//     public ShooterSubsystem()    {
//         shooterMotor1 = new CANSparkFlex(RobotConstants.shooter1CANid, MotorType.kBrushless);
//         shooterMotor2 = new CANSparkFlex(RobotConstants.shooter2CANid, MotorType.kBrushless);
//         shooterServo = new PWM(RobotConstants.shooterServoPWMid);
//         sensor = new DigitalInput(RobotConstants.shooterSensorDIOid);
//     }


//     public void shooterOn() {
//         shooterMotor1.set(RobotConstants.shooterspeed);
//         shooterMotor2.set(RobotConstants.shooterspeed); 
//     }

//     public void stop()    {
//         shooterMotor1.stopMotor();
//         shooterMotor2.stopMotor();
//     }

//     public void shoot()   {
//         shooterServo.setSpeed(RobotConstants.shooterServospeed);
//     }

//     public void unstuck()   {
//         shooterServo.setSpeed(RobotConstants.shooterServoReversespeed);
//     }

//     public void resetServo()  {
//         shooterServo.setSpeed(RobotConstants.shooterServoOffspeed);
//     }

//     @Override
//     public void periodic()  {
//         SmartDashboard.putBoolean("Sensor", sensor.get());
//     }
// }