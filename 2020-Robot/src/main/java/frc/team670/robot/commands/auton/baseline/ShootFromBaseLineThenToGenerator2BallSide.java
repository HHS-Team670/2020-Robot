package frc.team670.robot.commands.auton.baseline;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.paths.Path;
import frc.team670.paths.center.Center2BS;
import frc.team670.paths.left.Left2BS;
import frc.team670.paths.right.Right2BS;
import frc.team670.robot.commands.auton.AutoSelector.StartPosition;
import frc.team670.robot.commands.indexer.ShootAllBalls;
import frc.team670.robot.commands.shooter.Shoot;
import frc.team670.robot.commands.shooter.StartShooterByDistance;
import frc.team670.robot.commands.turret.RotateToAngle;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.Vision;

/**
 * Autonomous routine starting with shooting from the initiation line (facing
 * towards your driver station), ending at (and hopefully intaking and indexing)
 * 2 Power Cells under the generator near your trench.
 * 
 * @author ctychen, meganchoy
 */
public class ShootFromBaseLineThenToGenerator2BallSide extends SequentialCommandGroup implements MustangCommand {

        private Path trajectory;
        private Map<MustangSubsystemBase, HealthState> healthReqs;
        private DriveBase driveBase;
        private Vision coprocessor;

        /**
         * Initializes this command from the given parameters
         * 
         * @param startPosition the position of the robot at the beginning of the game
         * @param driveBase the drive base
         * @param intake the intake
         * @param conveyor the conveyor
         * @param shooter the shooter
         * @param indexer the indexer
         * @param turret the turret
         * @param coprocessor the coprocessor
         */
        public ShootFromBaseLineThenToGenerator2BallSide(StartPosition startPosition, DriveBase driveBase, Intake intake, 
        Conveyor conveyor, Shooter shooter, Indexer indexer, Turret turret, Vision coprocessor) {
                this.driveBase = driveBase;
                this.coprocessor = coprocessor;
                double turretAng = 0;
                if (startPosition == StartPosition.LEFT) {
                        trajectory = new Left2BS(driveBase);
                        turretAng = RobotConstants.leftTurretAng;
                }
                if (startPosition == StartPosition.CENTER)
                        trajectory = new Center2BS(driveBase);
                if (startPosition == StartPosition.RIGHT) {
                        trajectory = new Right2BS(driveBase);
                        turretAng = RobotConstants.rightTurretAng;
                }
                healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
                healthReqs.put(driveBase, HealthState.GREEN);
                healthReqs.put(shooter, HealthState.GREEN);
                healthReqs.put(intake, HealthState.GREEN);
                healthReqs.put(conveyor, HealthState.GREEN);
                healthReqs.put(indexer, HealthState.GREEN);
                healthReqs.put(turret, HealthState.GREEN);

                // Sets current position to the starting point of the path which is where we should be at init
                driveBase.resetOdometry(trajectory.getStartingPose());

                addCommands(
                        new StartShooterByDistance(shooter, driveBase),
                        // new RotateTurret(turret, driveBase, coprocessor),
                        new RotateToAngle(turret, turretAng),
                        new Shoot(shooter),
                        new ShootAllBalls(indexer),
                        new ParallelCommandGroup (
                                getTrajectoryFollowerCommand(trajectory, driveBase),
                                new ShootAllBalls(indexer)       
                        )
                );
        }

        @Override 
        public void end(boolean interrupted){
        //     if (!interrupted){
        //         addCommands(
        //             new GetVisionData(coprocessor),
        //             // should check here that vision didn't return an error code before updating
        //             new UpdatePoseFromVision(driveBase, coprocessor)
        //         );
        //     }
        }

        @Override
        public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
                // maybe also check that NavX is there
                return healthReqs;
        }
}