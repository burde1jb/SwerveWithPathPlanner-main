package frc.robot.subsystems;

//import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class WristSubsystem extends SubsystemBase   {
    CANSparkFlex wristMotor;
    DutyCycleEncoder wristEncoder;
    private final double rangeOffset = RobotConstants.wristrangeOffset;
    private final double encoderOffset = RobotConstants.wristencoderOffset;

    public WristSubsystem() {
        wristMotor = new CANSparkFlex(RobotConstants.wristmotorCANid, MotorType.kBrushless);
        wristEncoder = new DutyCycleEncoder(RobotConstants.wristEncoderDIOid);

        SparkPIDController wristmotorPID = wristMotor.getPIDController();
        wristmotorPID.setP(RobotConstants.wristmotorP);
        wristmotorPID.setI(RobotConstants.wristmotorI);
        wristmotorPID.setD(RobotConstants.wristmotorD);
        wristmotorPID.setFF(RobotConstants.wristmotorFF);

    }

    public void goTo(double encoderGoal, double extremaValue)  {
        if ((encoderGoal + encoderOffset) % 1 > (extremaValue + encoderOffset) % 1 && (wristEncoder.get() + encoderOffset) % 1 < (encoderGoal - rangeOffset + encoderOffset) % 1)   {
            this.goDown();
        }
        else if ((encoderGoal + encoderOffset) % 1 > (extremaValue + encoderOffset) % 1 && (wristEncoder.get() + encoderOffset) % 1 > (encoderGoal + rangeOffset + encoderOffset) % 1)    {
            this.goUp();
        }
        else if ((encoderGoal + encoderOffset) % 1 < (extremaValue + encoderOffset) % 1 && (wristEncoder.get() + encoderOffset) % 1 > (encoderGoal + rangeOffset + encoderOffset) % 1)  {
            this.goUp();
        }
        else if ((encoderGoal + encoderOffset) % 1 < (extremaValue + encoderOffset) % 1 && (wristEncoder.get() + encoderOffset) % 1 < (encoderGoal - rangeOffset + encoderOffset) % 1)  {
            this.goDown();
        }
        else    {
            this.stop();
        }
    }

    public boolean wentTo(double encoderGoal, double extremaValue)  {
        if ((encoderGoal + encoderOffset) % 1 > (extremaValue + encoderOffset) % 1 && (wristEncoder.get() + encoderOffset) % 1 < (encoderGoal - rangeOffset + encoderOffset) % 1)   {
            this.goDown();
            return false;
        }
        else if ((encoderGoal + encoderOffset) % 1 > (extremaValue + encoderOffset) % 1 && (wristEncoder.get() + encoderOffset) % 1 > (encoderGoal + rangeOffset + encoderOffset) % 1)    {
            this.goUp();
            return false;
        }
        else if ((encoderGoal + encoderOffset) % 1 < (extremaValue + encoderOffset) % 1 && (wristEncoder.get() + encoderOffset) % 1 > (encoderGoal + rangeOffset + encoderOffset) % 1)  {
            this.goUp();
            return false;
        }
        else if ((encoderGoal + encoderOffset) % 1 < (extremaValue + encoderOffset) % 1 && (wristEncoder.get() + encoderOffset) % 1 < (encoderGoal - rangeOffset + encoderOffset) % 1)  {
            this.goDown();
            return false;
        }
        else    {
            this.stop();
            return true;
        }
    }

    public void goUp()   {
        wristMotor.set(RobotConstants.wristgoUPpower);
    }

    public void goDown()    {
        wristMotor.set(RobotConstants.wristgoDOWNpower);
    }

    public void stop()  {
        wristMotor.stopMotor();
    }

    public boolean encoderCheck(double distance)    {
        if (wristEncoder.getAbsolutePosition() == distance)  {
            return true;
        }
        return false;
    }

    @Override
    public void periodic()  {
        SmartDashboard.putNumber("Wrist Encoder", (wristEncoder.getAbsolutePosition()));
    }
}