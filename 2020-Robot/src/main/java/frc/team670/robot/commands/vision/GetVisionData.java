package frc.team670.robot.commands.vision;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.utils.functions.MathUtils;

public class GetVisionData extends CommandBase implements MustangCommand {

    private MustangCoprocessor coprocessor;

    private long startTime;

    private boolean runVisionOnce;

    private static final double MAX_TIME_TO_RUN = 2500; // Max time to run this in ms

    public GetVisionData(MustangCoprocessor coprocessor) {
        this.coprocessor = coprocessor;
    }

    @Override
    public void initialize() {
        runVisionOnce =  false;
        coprocessor.turnOnLEDs();
        // SmartDashboard.putNumberArray(coprocessor.VISION_RETURN_NETWORK_KEY, new double[] { RobotConstants.VISION_ERROR_CODE,
        //         RobotConstants.VISION_ERROR_CODE, RobotConstants.VISION_ERROR_CODE }); // Clears vision data so we don't
        //         
        // coprocessor.clearLastValues();  
        // NetworkTableInstance.getDefault().flush();        // use old data accidentally
        coprocessor.enableVision(true);
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        if(!runVisionOnce && System.currentTimeMillis()-startTime > 3000){
            coprocessor.getLatestVisionData();
            runVisionOnce = true;
        }
    }

    @Override
    public boolean isFinished() {
        long time = System.currentTimeMillis();
        return (!MathUtils.doublesEqual(SmartDashboard.getNumberArray("reflect_tape_vision_data",
                new double[] { RobotConstants.VISION_ERROR_CODE, RobotConstants.VISION_ERROR_CODE,
                        RobotConstants.VISION_ERROR_CODE })[2],
                RobotConstants.VISION_ERROR_CODE) && time > startTime + 100 || time > startTime + MAX_TIME_TO_RUN);
    }

    @Override
    public void end(boolean interrupted) {
        coprocessor.turnOffLEDs();
        coprocessor.enableVision(false);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

}