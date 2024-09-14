package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class ClimberSubsystem extends SubsystemBase {

    private TalonFX climberMotor;
    private final double encoderOffset = RobotConstants.climberencoderOffset;
    private final double rangeOffset = RobotConstants.climberrangeOffset;

    public ClimberSubsystem(){
        climberMotor = new TalonFX(RobotConstants.climberCANid);

        //TalonFXConfiguration climberPID = new TalonFXConfiguration();
        var climberPID = new Slot0Configs();
        climberPID.kP = RobotConstants.climbermotorP;
        climberPID.kI = RobotConstants.climbermotorI;
        climberPID.kD = RobotConstants.climbermotorD;
        climberPID.kV = RobotConstants.climbermotorFF;

        climberMotor.getConfigurator().apply(climberPID);
    }

    public void goTo(double encoderGoal, double extremaValue)  {

        double position = getEncoder() + encoderOffset;
        double target = encoderGoal + encoderOffset;

        if (encoderGoal > extremaValue && position - rangeOffset < target)   {
            this.goUp();
        }
        else if (encoderGoal < extremaValue && position + rangeOffset > target)  {
            this.goDown();
        }
        else    {
            this.stop();
        }
    }

    public double getEncoder() {
        climberMotor.getPosition().refresh();
        return climberMotor.getPosition().getValue();
    }
    
    public void goUp()  {
        climberMotor.set(RobotConstants.climberUpspeed);
    }
    
    public void goDown(){
        climberMotor.set(RobotConstants.climberDownspeed);
    }

    public void stop()    {
        climberMotor.set(RobotConstants.climberOffspeed);
    }

    @Override
    public void periodic()  {
        SmartDashboard.putNumber("Climber Encoder", (getEncoder()));
    }

}