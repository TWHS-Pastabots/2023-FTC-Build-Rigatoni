package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class RedAutonSequences {

    SampleMecanumDrive drive;
    AutonUtilities utilities;

    // Servo Positions (0.35 straight)
    final double RED_HIGH_SERVO = 0.55;
    final double RED_MID_SERVO = 0.40;
    final double RED_LOW_SERVO = 0.50;

    // Flywheel speeds
    final double RED_HIGH_VELOCITY = 340;
    final double RED_MID_VELOCITY = 340;
    final double RED_LOW_VELOCITY = 340;

    final int FIRST_SHOT_FLYWHEEL_DURATION = 5000;
    final int MULTI_SHOT_FLYWHEEL_DURATION = 9000;

    // Trajectories
    Trajectory redFirstShootTrajectory;
    Trajectory redFirstShoot1Trajectory;
    Trajectory redFirstShoot2Trajectory;
    Trajectory redFirstShoot3Trajectory;
    Trajectory redRing1PickupTrajectory;
    Trajectory redRing2PickupTrajectory;
    Trajectory redRing3PickupTrajectory;
    Trajectory redTripleShootTrajectory;
    Trajectory redPark1Trajectory;
    Trajectory redPark2Trajectory;
    Trajectory redPark3Trajectory;

    // Red Pose + Vector2d
    Pose2d redStartPose = new Pose2d(-64, 48, Math.toRadians(0));
    Vector2d redShoot = new Vector2d(-24, 18);
    Vector2d red1Shoot = new Vector2d(-24, 18);
    Vector2d red2Shoot = new Vector2d(-24, 18);
    Vector2d red3Shoot = new Vector2d(-24, 18);
    Vector2d redTowardsFirstRing = new Vector2d(-48, 36);
    Vector2d redFirstRing = new Vector2d(-48, 72);
    Vector2d redSecondRing = new Vector2d(-24, 72);
    Vector2d redThirdRing = new Vector2d(42, 72);
    Vector2d redPark1 = new Vector2d(32, 36);
    Vector2d redPark2 = new Vector2d(8, 58);
    Vector2d redPark3 = new Vector2d(-6, 36);


    public RedAutonSequences(HardwareMap hardwareMap, AutonUtilities utilities)
    {
        this.utilities = utilities;
        drive = new SampleMecanumDrive(hardwareMap);

        // Trajectories
        redFirstShootTrajectory = drive.trajectoryBuilder(redStartPose)
                .splineTo(redShoot, Math.toRadians(0))
                .build();
        redFirstShoot1Trajectory = drive.trajectoryBuilder(redStartPose)
                .splineTo(red1Shoot, Math.toRadians(0))
                .build();
        redFirstShoot2Trajectory = drive.trajectoryBuilder(redStartPose)
                .splineTo(red2Shoot, Math.toRadians(0))
                .build();
        redFirstShoot3Trajectory = drive.trajectoryBuilder(redStartPose)
                .splineTo(red3Shoot, Math.toRadians(0))
                .build();
        redRing1PickupTrajectory = drive.trajectoryBuilder(new Pose2d(redShoot, Math.toRadians(0)))
                .splineTo(redTowardsFirstRing, Math.toRadians(90))
                .splineTo(redFirstRing, Math.toRadians(90))
                .build();
        redRing2PickupTrajectory = drive.trajectoryBuilder(new Pose2d(redFirstRing, Math.toRadians(90)))
                .splineTo(redSecondRing, Math.toRadians(0))
                .build();
        redRing3PickupTrajectory = drive.trajectoryBuilder(new Pose2d(redSecondRing, Math.toRadians(0)))
                .splineTo(redThirdRing, Math.toRadians(0))
                .build();
        redTripleShootTrajectory = drive.trajectoryBuilder(new Pose2d(redThirdRing, Math.toRadians(0)))
                .splineTo(redShoot, Math.toRadians(0))
                .build();
        redPark1Trajectory = drive.trajectoryBuilder(new Pose2d(redShoot, Math.toRadians(0)))
                .splineTo(redPark1, Math.toRadians(90))
                .build();
        redPark2Trajectory = drive.trajectoryBuilder(new Pose2d(redShoot, Math.toRadians(0)))
                .splineTo(redPark2, Math.toRadians(90))
                .build();
        redPark3Trajectory = drive.trajectoryBuilder(new Pose2d(redShoot, Math.toRadians(0)))
                .splineTo(redPark3, Math.toRadians(90))
                .build();
    }

    public void redOneFull()
    {
        drive.setPoseEstimate(redStartPose);

        // Aim first ring
        utilities.aimLauncher(RED_MID_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(RED_MID_VELOCITY, FIRST_SHOT_FLYWHEEL_DURATION);

        // Trajectory to shooting position
        drive.followTrajectory(redFirstShoot1Trajectory);

        // Shoot first ring
        utilities.shoot();
        utilities.wait(1000);

        // Pick up and shoot other rings
        redIntake();

        // Park
        drive.followTrajectory(redPark1Trajectory);
        utilities.retractIntake();
    }
    public void redTwoFull()
    {
        drive.setPoseEstimate(redStartPose);

        // Aim first ring
        utilities.aimLauncher(RED_LOW_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(RED_LOW_VELOCITY, FIRST_SHOT_FLYWHEEL_DURATION);

        // Trajectory to shooting position
        drive.followTrajectory(redFirstShoot2Trajectory);

        // Shoot first ring
        utilities.shoot();
        utilities.wait(1000);

        // Pick up and shoot other rings
        redIntake();

        // Park
        drive.followTrajectory(redPark2Trajectory);
        utilities.retractIntake();
    }
    public void redThreeFull()
    {
        drive.setPoseEstimate(redStartPose);

        // Aim first ring
        utilities.aimLauncher(RED_HIGH_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(RED_HIGH_VELOCITY, FIRST_SHOT_FLYWHEEL_DURATION);

        // Trajectory to shooting position
        drive.followTrajectory(redFirstShoot3Trajectory);

        // Shoot first ring
        utilities.shoot();
        utilities.wait(1000);

        // Pick up and shoot other rings
        redIntake();

        // Park
        drive.followTrajectory(redPark3Trajectory);
        utilities.retractIntake();
    }

    public void redIntake()
    {
        // Pick up three rings
        utilities.intake(true);
        drive.followTrajectory(redRing1PickupTrajectory);
//        utilities.wait(1000);
        drive.followTrajectory(redRing2PickupTrajectory);
//        utilities.wait(1000);
        drive.followTrajectory(redRing3PickupTrajectory);


        // Aim ring
        utilities.aimLauncher(RED_HIGH_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(RED_HIGH_VELOCITY);

        drive.followTrajectory(redTripleShootTrajectory);
        // Turn off intake
        utilities.intake(false);
        utilities.retractIntake();

        // Shoot three rings into high goal
        for(int i = 0; i<3; i++)
        {
            utilities.shoot();
            utilities.wait(1000);
        }
    }

    public void redOnePart()
    {
        drive.setPoseEstimate(redStartPose);

        // Aim first ring
        utilities.aimLauncher(RED_MID_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(RED_MID_VELOCITY);

        // Trajectory to shooting position
        drive.followTrajectory(redFirstShoot1Trajectory);

        // Shoot first ring
        utilities.shoot();
        utilities.wait(1000);
        utilities.flywheelOff();

        // Park
        drive.followTrajectory(redPark1Trajectory);
        utilities.retractIntake();
    }
    public void redTwoPart()
    {
        drive.setPoseEstimate(redStartPose);

        // Aim first ring
        utilities.aimLauncher(RED_LOW_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(RED_LOW_VELOCITY);

        // Trajectory to shooting position
        drive.followTrajectory(redFirstShoot2Trajectory);

        // Shoot first ring
        utilities.shoot();
        utilities.wait(1000);
        utilities.flywheelOff();

        // Park
        drive.followTrajectory(redPark2Trajectory);
        utilities.retractIntake();
    }
    public void redThreePart()
    {
        drive.setPoseEstimate(redStartPose);

        // Aim first ring
        utilities.aimLauncher(RED_HIGH_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(RED_HIGH_VELOCITY);

        // Trajectory to shooting position
        drive.followTrajectory(redFirstShoot3Trajectory);

        // Shoot first ring
        utilities.shoot();
        utilities.wait(1000);
        utilities.flywheelOff();

        // Park
        drive.followTrajectory(redPark3Trajectory);
        utilities.retractIntake();
    }
}
