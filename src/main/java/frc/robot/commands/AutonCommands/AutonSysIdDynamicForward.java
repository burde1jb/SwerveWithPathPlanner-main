package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.CommandSwerveDrivetrain;

public class AutonSysIdDynamicForward extends Command  {
    
    CommandSwerveDrivetrain swerve; 

    public AutonSysIdDynamicForward(CommandSwerveDrivetrain commandSwerveDrivetrain)  {
        this.swerve = commandSwerveDrivetrain;
    }

    @Override
    public void execute() {
        swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward);
    }

}
