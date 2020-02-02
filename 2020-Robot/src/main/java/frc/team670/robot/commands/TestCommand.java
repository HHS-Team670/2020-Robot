package frc.team670.robot.commands;

import java.util.Map;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.subsystems.Test;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.utils.Logger;

public class TestCommand extends CommandBase implements MustangCommand {

    private Test system;

    public TestCommand(Test system) {
        this.system = system;
    }

    public void initialize() {
        Logger.consoleLog("Test subsystem started");
    }

    @Override
    public void execute() {
        system.valueTest();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

}