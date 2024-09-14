package frc.robot.subsystems;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.ctre.phoenix6.hardware.Pigeon2;

public class NewShooterSubsystem extends SubsystemBase {
    
    CANSparkFlex shooterMotor3;
    CANSparkFlex shooterMotor4;
    CANSparkFlex shootertilt;
    CANSparkFlex indexer;
    DutyCycleEncoder shooterEncoder;
    public DigitalInput sensor;
    Pigeon2 gyro;

    private final double shooterencoderOffset = RobotConstants.shooterencoderOffset;
    private final double shooterrangeOffset = RobotConstants.shooterrangeOffset;


    public NewShooterSubsystem()    {
        shooterMotor3 = new CANSparkFlex(RobotConstants.shooter3CANid, MotorType.kBrushless);
        shooterMotor4 = new CANSparkFlex(RobotConstants.shooter4CANid, MotorType.kBrushless);
        shootertilt = new CANSparkFlex(RobotConstants.shootertiltCANid, MotorType.kBrushless);
        indexer = new CANSparkFlex(RobotConstants.indexerCANid, MotorType.kBrushless);

        shooterEncoder = new DutyCycleEncoder(RobotConstants.shooterEncoderDIOid);

        sensor = new DigitalInput(RobotConstants.shooterSensorDIOid);

        gyro = new Pigeon2(0, "drivetrain");
    }

    public void goToshot(double degrees)  {
        
        var position = ((shooterEncoder.getAbsolutePosition() + shooterencoderOffset) % 1);
        
        if (position > (degrees + shooterrangeOffset + shooterencoderOffset) % 1) {
            this.tiltUp(RobotConstants.tiltUppower);
        }
        else if (position < (degrees - shooterrangeOffset + shooterencoderOffset) % 1) {
            this.tiltDown(RobotConstants.tiltDownpower);
        }
        else    {
            this.tiltstop();
        }
    }

    public boolean wentTo(double degrees)  {

        var pos = (shooterEncoder.getAbsolutePosition() + shooterencoderOffset) % 1;
        var maxtarget = (degrees + shooterrangeOffset + shooterencoderOffset) % 1;
        var mintarget = (degrees - shooterrangeOffset + shooterencoderOffset) % 1;

        if (pos > maxtarget) {
            this.tiltUp(RobotConstants.tiltUppower);
            return false;
        }
        else if (pos < mintarget) {
            this.tiltDown(RobotConstants.tiltDownpower);
            return false;
        }
        else    {
            this.tiltstop();
            return true;
        }
    }

    public void tiltUp(double speed){
        shootertilt.set(speed);
    }

    public void tiltDown(double speed){
        shootertilt.set(-speed);
    }

    public void tiltstop(){
        shootertilt.stopMotor();
    }

    public void newshooterOn() {
        shooterMotor3.set(RobotConstants.shooterspeed);
        shooterMotor4.set(RobotConstants.shooterspeed); 
    }

    public void indexOn(){
        indexer.set(RobotConstants.indexerspeed);
    }

    public void indexOnMax(){
        indexer.set(RobotConstants.indexerMaxspeed);
    }

    public void newstopshooter()    {
        shooterMotor3.stopMotor();
        shooterMotor4.stopMotor();
    }

    public void indexOut(){
        indexer.set(RobotConstants.indexerspeedOut);
    }

    public void indexOff(){
        indexer.stopMotor();
    }

    public void gyroreset(){
        gyro.reset();
    }

    public void indexAuton(boolean forward)  {
        if (forward)    {
            indexer.set(RobotConstants.indexerspeed);
        }
        else {
            indexer.set(RobotConstants.indexerspeedOut);
        }
    }

    public void indexMaxAuton(boolean forward)  {
        if (forward)    {
            indexer.set(RobotConstants.indexerMaxspeed);
        }
        else {
            indexer.set(RobotConstants.indexerspeedOut);
        }
    }
    
    @Override
    public void periodic()  {
        SmartDashboard.putNumber("Shooter Encoder", shooterEncoder.get());
        SmartDashboard.putBoolean("Shooter Sensor", sensor.get());
    }
}