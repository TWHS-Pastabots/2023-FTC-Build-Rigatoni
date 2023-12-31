package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class BlueIntakeAutonSequences {

    SampleMecanumDrive drive;
    AutonUtilities utilities;

    // Servo Positions (0.35 straight)
    final double BLUE_HIGH_SERVO = 0.40;

    final double BLUE_MID_SERVO = 0.42; // Done
    final double BLUE_LOW_SERVO = 0.36; // Done

    // Flywheel speeds
    final double BLUE_HIGH_VELOCITY = 350;
    final double BLUE_MID_VELOCITY = 350;
    final double BLUE_LOW_VELOCITY = 340;

    // Trajectories
    Trajectory blueFirstShootTrajectory;
    Trajectory blueFirstShoot1Trajectory;
    Trajectory blueFirstShoot2Trajectory;
    Trajectory blueFirstShoot3Trajectory;
    Trajectory blueRing1PickupTrajectory;
    Trajectory blueRing2PickupTrajectory;
    Trajectory blueRing3PickupTrajectory;
    Trajectory blueTripleShootTrajectory;
    Trajectory bluePark1Trajectory;
    Trajectory bluePark2Trajectory;
    Trajectory bluePark3Trajectory;

    // Pose + Vector2d
    Pose2d blueStartPose = new Pose2d(-64, -48, Math.toRadians(0));
    Vector2d blueStartVector = new Vector2d(-64, -48);
    Vector2d blueShoot = new Vector2d(-24, -18);
    Vector2d blue1Shoot = new Vector2d(-24, -18);
    Vector2d blue2Shoot = new Vector2d(-24, -18);
    Vector2d blue3Shoot = new Vector2d(-24, -18);
    Vector2d blueTowardsFirstRing = new Vector2d(-48, -36);
    Vector2d blueFirstRing = new Vector2d(-48, -72);
    Vector2d blueSecondRing = new Vector2d(-24, -72);
    Vector2d blueThirdRing = new Vector2d(42, -72);
    Vector2d bluePark1 = new Vector2d(32, -36); // Done
    Vector2d bluePark2 = new Vector2d(8, -58); // Done
    Vector2d bluePark3 = new Vector2d(-12, -36);

    public BlueIntakeAutonSequences(HardwareMap hardwareMap, AutonUtilities utilities)
    {
        this.utilities = utilities;
        drive = new SampleMecanumDrive(hardwareMap);

        // Trajectories
        blueFirstShootTrajectory = drive.trajectoryBuilder(blueStartPose)
                .splineTo(blueShoot, Math.toRadians(0))
                .build();
        blueFirstShoot1Trajectory = drive.trajectoryBuilder(blueStartPose)
                .splineTo(blue1Shoot, Math.toRadians(20))
                .build();
        blueFirstShoot2Trajectory = drive.trajectoryBuilder(blueStartPose)
                .splineTo(blue2Shoot, Math.toRadians(0))
                .build();
        blueFirstShoot3Trajectory = drive.trajectoryBuilder(blueStartPose)
                .splineTo(blue3Shoot, Math.toRadians(0))
                .build();
        blueRing1PickupTrajectory = drive.trajectoryBuilder(new Pose2d(blueStartVector, Math.toRadians(0)))
                .splineTo(blueTowardsFirstRing, Math.toRadians(-90))
                .splineTo(blueFirstRing, Math.toRadians(-90))
                .build();
        blueRing2PickupTrajectory = drive.trajectoryBuilder(new Pose2d(blueFirstRing, Math.toRadians(-90)))
                .splineTo(blueSecondRing, Math.toRadians(0))
                .build();
        blueRing3PickupTrajectory = drive.trajectoryBuilder(new Pose2d(blueSecondRing, Math.toRadians(0)))
                .splineTo(blueThirdRing, Math.toRadians(0))
                .build();
        blueTripleShootTrajectory = drive.trajectoryBuilder(new Pose2d(blueThirdRing, Math.toRadians(0)))
                .splineTo(blueShoot, Math.toRadians(0))
                .build();
        bluePark1Trajectory = drive.trajectoryBuilder(new Pose2d(blueShoot, Math.toRadians(15)))
                .splineTo(bluePark1, Math.toRadians(-90))
                .build();
        bluePark2Trajectory = drive.trajectoryBuilder(new Pose2d(blueShoot, Math.toRadians(0)))
                .splineTo(bluePark2, Math.toRadians(-90))
                .build();
        bluePark3Trajectory = drive.trajectoryBuilder(new Pose2d(blueShoot, Math.toRadians(0)))
                .splineTo(bluePark3, Math.toRadians(-90))
                .build();
    }

    public void blueIntake()
    {
        drive.setPoseEstimate(blueStartPose);
        utilities.deployIntakeFully();
        blueIntakeSequences();

        // Park
        drive.followTrajectory(bluePark1Trajectory);
        utilities.retractIntake();
    }
    public void blueIntakeSequences()
    {
        // Pick up three rings
        utilities.intake(true);
        drive.followTrajectory(blueRing1PickupTrajectory);
//        utilities.wait(500);
        drive.followTrajectory(blueRing2PickupTrajectory);
//        utilities.wait(500);
        drive.followTrajectory(blueRing3PickupTrajectory);
//        utilities.wait(500);

        // Aim ring
        utilities.aimLauncher(BLUE_HIGH_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(BLUE_HIGH_VELOCITY);

        drive.followTrajectory(blueTripleShootTrajectory);
        // Turn off intake
        utilities.intake(false);
        utilities.retractIntake();

        // Shoot three rings into high goal
        for(int i = 0; i<3; i++)
        {
            utilities.shoot();
            utilities.wait(1000);
        }
        utilities.flywheelOff();
    }
}
