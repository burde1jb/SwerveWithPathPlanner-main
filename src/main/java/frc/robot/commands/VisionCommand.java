package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class VisionCommand extends CommandBase {
    VisionSubsystem visionSubsystem;
    //Swerve swerve;
    XboxController controller2;

    public VisionCommand(VisionSubsystem visionSubsystem, XboxController controller2)    {
        this.visionSubsystem = visionSubsystem;
       // this.swerve = swerve;
        this.controller2 = controller2;
        addRequirements(visionSubsystem);
       // addRequirements(swerve);
    }

    public void autoAim()   {
       // swerve.rotateTo(visionSubsystem.x);
        //swerve.xStance();
    }

}
