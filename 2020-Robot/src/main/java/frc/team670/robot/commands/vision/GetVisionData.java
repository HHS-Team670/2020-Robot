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

    private double[] visionData;
    private long startTime;

    private static final double MAX_TIME_TO_RUN = 2500; // Max time to run this in ms

    public GetVisionData(double[] visionData, MustangCoprocessor coprocessor) {
        this.visionData = visionData;
        this.coprocessor = coprocessor;
    }

    @Override
    public void initialize() {
        coprocessor.turnOnLEDs();
        SmartDashboard.putNumberArray("reflect_tape_vision_data", new double[] { RobotConstants.VISION_ERROR_CODE,
                RobotConstants.VISION_ERROR_CODE, RobotConstants.VISION_ERROR_CODE }); // Clears vision data so we don't
                                                                                       // use old data accidentally
        coprocessor.enableVision(true);
        NetworkTableInstance.getDefault().flush();
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
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
        SmartDashboard.putString("vision-enabled", "disabled");
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

}