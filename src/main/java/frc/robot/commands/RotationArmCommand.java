package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.RotationArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class RotationArmCommand extends Command {
    
    RotationArmSubsystem rotationArmSubsystem;
    WristSubsystem wristSubsystem;
    XboxController controller2;


    public RotationArmCommand(RotationArmSubsystem rotationArmSubsystem, WristSubsystem wristSubsystem, XboxController controller2)    {
        this.rotationArmSubsystem = rotationArmSubsystem;
        this.wristSubsystem= wristSubsystem;
        this.controller2 = controller2;
        addRequirements(rotationArmSubsystem);
        addRequirements(wristSubsystem);
    }


    @Override
    public void execute()   {
        if (controller2.getXButton())   {
            wristSubsystem.goUp();
        }
        else if (controller2.getBButton())  {
            wristSubsystem.goDown();
        }
        else if (controller2.getAButton())   {  // Dump
            rotationArmSubsystem.goTo(RobotConstants.dumpArmValue);
            wristSubsystem.goTo(RobotConstants.dumpWristValue, RobotConstants.dumpWristExtreme);
        }
        else if (controller2.getYButton()) {    // Amp
            rotationArmSubsystem.goTo(RobotConstants.ampArmValue);
            wristSubsystem.goTo(RobotConstants.ampWristValue, RobotConstants.ampWristExtreme);
        }
        else if (controller2.getLeftBumper())   {   // Ground
            rotationArmSubsystem.goTo(RobotConstants.groundArmValue); 
            wristSubsystem.goTo(RobotConstants.groundWristValue, RobotConstants.groundWristExtreme);
        }
        else if (controller2.getStartButton())  {   // Source
            rotationArmSubsystem.goTo(RobotConstants.sourceArmValue);
            wristSubsystem.goTo(RobotConstants.sourceWristValue, RobotConstants.sourceWristExtreme);
        }
        else    {
            rotationArmSubsystem.stop();
            wristSubsystem.stop();
        }
    }
    
}