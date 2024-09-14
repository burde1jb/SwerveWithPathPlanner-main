package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutonIntakeOffCommand extends Command   {
    
    IntakeSubsystem intakeSubsystem;

    public AutonIntakeOffCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }
    
    @Override
    public void execute()   {
        intakeSubsystem.intakeOff();
    }

}
