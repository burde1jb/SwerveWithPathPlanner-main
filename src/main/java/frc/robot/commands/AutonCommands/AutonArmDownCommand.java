package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.RotationArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AutonArmDownCommand extends Command    {
    
    RotationArmSubsystem rotationArmSubsystem;
    WristSubsystem wristSubsystem;
    boolean isItFinished;
    boolean wristFinished;
    boolean armFinished;

    public AutonArmDownCommand(RotationArmSubsystem rotationArmSubsystem, WristSubsystem wristSubsystem) {
        this.rotationArmSubsystem = rotationArmSubsystem;
        this.wristSubsystem = wristSubsystem;
        addRequirements(rotationArmSubsystem);
        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize()    {
        isItFinished = false;
        armFinished = false;
        wristFinished = false;
    }

    @Override
    public void execute()   {
        if (!armFinished && rotationArmSubsystem.wentTo(RobotConstants.groundArmValue)) {
            rotationArmSubsystem.stop();
            armFinished = true;
        }
        if (!wristFinished && wristSubsystem.wentTo(RobotConstants.groundWristValue, RobotConstants.groundWristExtreme)) {
            wristSubsystem.stop();
            wristFinished = true;
        }
        if (armFinished && wristFinished) {
            isItFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isItFinished;
    }

}