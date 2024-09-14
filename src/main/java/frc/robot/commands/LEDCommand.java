package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.NewShooterSubsystem;
import frc.robot.RobotConstants;

public class LEDCommand extends Command {
    LEDSubsystem ledSubsystem;
    NewShooterSubsystem shooterSubsystem;
    IntakeSubsystem intakeSubsystem;
    XboxController controller2;

    public LEDCommand(LEDSubsystem ledSubsystem, XboxController controller2, NewShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem)    {
        this.ledSubsystem = ledSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.controller2 = controller2;
        addRequirements(ledSubsystem);
    }
    
    @Override
    public void execute()   {
        if (!intakeSubsystem.sensor.get()) {
            ledSubsystem.set(RobotConstants.LEDintakesensor);
        }
        else if (!shooterSubsystem.sensor.get()) {
            ledSubsystem.set(RobotConstants.LEDshootersensor);
        }
        else    {
            ledSubsystem.set(RobotConstants.LEDdefault);
        }
    }
}