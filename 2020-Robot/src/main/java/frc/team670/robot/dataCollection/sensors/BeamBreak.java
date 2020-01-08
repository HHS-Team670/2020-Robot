package frc.team670.robot.dataCollection.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BeamBreak{

    private DigitalInput m_BeamBreak;

    public BeamBreak(int channel){
        m_BeamBreak = new DigitalInput(channel);
    }

    public boolean getBeamBreakOutput(){
          return m_BeamBreak.get(); 
      }


    public void sendBeamBreakDataToDashboard() {
          SmartDashboard.putString("Beam Break Sensor:", getBeamBreakOutput() + "");
        }
}