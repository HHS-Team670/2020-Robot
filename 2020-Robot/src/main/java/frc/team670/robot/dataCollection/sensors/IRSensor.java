package frc.team670.robot.dataCollection.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IRSensor{

    private DigitalInput m_IR;

    public IRSensor(int channel){
        m_IR = new DigitalInput(channel);
    }

    public boolean getIRSensorOutput(){
          return m_IR.get(); 
      }


    public void sendIRDataToDashboard() {
          SmartDashboard.putString("IR Sensor:", getIRSensorOutput() + "");
        }
}