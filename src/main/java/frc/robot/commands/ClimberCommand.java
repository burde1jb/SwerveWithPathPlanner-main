package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;


public class ClimberCommand extends Command {

    ClimberSubsystem climberSubsystem;
    XboxController controller2;
    final double initialEncoderPos;

    public ClimberCommand(ClimberSubsystem climberSubsystem,XboxController controller2){
        this.climberSubsystem = climberSubsystem;
        this.controller2 = controller2;
        addRequirements(climberSubsystem);
        initialEncoderPos = climberSubsystem.getEncoder();
    }

    @Override
    public void execute(){
        if (controller2.getRightY() < -0.2) {
            climberSubsystem.goUp();
        }
        else if (controller2.getRightY() > 0.2){
            climberSubsystem.goDown();
        }
        else {
            climberSubsystem.stop();
        }
    }


}