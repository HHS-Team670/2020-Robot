package frc.team670.robot.commands;

import frc.team670.robot.subsystems.MustangSubsystemBase;

public class TestSubsystem extends MustangSubsystemBase{

    private int c;

    public TestSubsystem(int c){
        this.c = c;
    }

    public HealthState checkHealth(){
        if (c%4 == 1) return HealthState.GREEN;
        else if (c%4 == 2) return HealthState.YELLOW;
        else return HealthState.RED;
    }

	@Override
	public void zeroSensors() {
		// TODO Auto-generated method stub
		
	}

}

