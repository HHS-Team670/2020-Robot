/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.auton.baseline;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.paths.Path;
import frc.team670.paths.center.Center3BS;
import frc.team670.paths.left.Left3BS;
import frc.team670.paths.right.Right3BS;
import frc.team670.robot.commands.auton.AutoSelector.StartPosition;
import frc.team670.robot.commands.indexer.RunIndexer;
import frc.team670.robot.commands.shooter.Shoot;
import frc.team670.robot.commands.shooter.StartShooterByDistance;
import frc.team670.robot.commands.turret.RotateTurret;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
// import frc.team670.robot.subsystems.MustangSubsystemBase;
// import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.Vision;

/**
 * Autonomous routine starting with shooting from the initiation line (facing
 * towards your driver station), ending at (and hopefully intaking and indexing)
 * 3 Power Cells under the generator.
 * 
 * @author ctychen, meganchoy
 */
public class ShootFromBaseLineThenToGenerator3BallMid extends SequentialCommandGroup implements MustangCommand {

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
        public ShootFromBaseLineThenToGenerator3BallMid(StartPosition startPosition, DriveBase driveBase, Intake intake,
            Conveyor conveyor, Shooter shooter, Indexer indexer, Turret turret, Vision coprocessor) {
          this.driveBase = driveBase;
          this.coprocessor = coprocessor;
          if (startPosition == StartPosition.LEFT)
              trajectory = new Left3BS(driveBase);
          if (startPosition == StartPosition.CENTER)
              trajectory = new Center3BS(driveBase);
          if (startPosition == StartPosition.RIGHT)
              trajectory = new Right3BS(driveBase);
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

              new RotateTurret(turret, driveBase, coprocessor),
              
              new ParallelCommandGroup (
              new RunIndexer(indexer), 
              new Shoot(shooter)        
            ),
            
              new ParallelCommandGroup (
              getTrajectoryFollowerCommand(trajectory, driveBase),
              new RunIndexer(indexer)       
            )
          );
        }

        @Override 
        public void end(boolean interrupted){
            // if (!interrupted){
            //     addCommands(
            //         new GetVisionData(coprocessor),
            //         new UpdatePoseFromVision(driveBase, coprocessor)
            //     );
            // }
        }

        @Override
        public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
          // maybe also check that NavX is there
          return healthReqs;
        }
      }