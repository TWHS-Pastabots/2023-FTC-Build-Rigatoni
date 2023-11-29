package org.firstinspires.ftc.teamcode.autonomous;
import org.firstinspires.ftc.teamcode.drive.*;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutonSequences {

    SampleMecanumDrive drive;
    AutonUtilities utilities;

    // Servo Positions (0.35 straight)
    final double BLUE_HIGH_SERVO = 0.60;
    final double BLUE_MID_SERVO = 0.43;
    final double BLUE_LOW_SERVO = 0.30;
    final double RED_HIGH_SERVO = 0.60;
    final double RED_MID_SERVO = 0.40;
    final double RED_LOW_SERVO = 0.50;

    // Flywheel speeds
    final double BLUE_HIGH_VELOCITY = 340;
    final double BLUE_MID_VELOCITY = 280;
    final double BLUE_LOW_VELOCITY = 340;
    final double RED_HIGH_VELOCITY = 340;
    final double RED_MID_VELOCITY = 340;
    final double RED_LOW_VELOCITY = 340;

    final int FIRST_SHOT_FLYWHEEL_DURATION = 5000;
    final int MULTI_SHOT_FLYWHEEL_DURATION = 9000;

    // Trajectories

    // Blue Trajectories
    Trajectory blueFirstShootTrajectory;
    Trajectory blueRing1PickupTrajectory;
    Trajectory blueRing2PickupTrajectory;
    Trajectory blueRing3PickupTrajectory;
    Trajectory blueTripleShootTrajectory;
    Trajectory bluePark1Trajectory;
    Trajectory bluePark2Trajectory;
    Trajectory bluePark3Trajectory;
    // Red Trajectories
    Trajectory redFirstShootTrajectory;
    Trajectory redRing1PickupTrajectory;
    Trajectory redRing2PickupTrajectory;
    Trajectory redRing3PickupTrajectory;
    Trajectory redTripleShootTrajectory;
    Trajectory redPark1Trajectory;
    Trajectory redPark2Trajectory;
    Trajectory redPark3Trajectory;

    // Blue Pose + Vector2d
    Pose2d blueStartPose = new Pose2d(-64, -48, Math.toRadians(0));
    Vector2d blueShoot = new Vector2d(-24, -18);
    Vector2d blueTowardsFirstRing = new Vector2d(-48, -36);
    Vector2d blueFirstRing = new Vector2d(-48, -72);
    Vector2d blueSecondRing = new Vector2d(-24, -72);
    Vector2d blueThirdRing = new Vector2d(48, -72);
    Vector2d bluePark1 = new Vector2d(36, -36);
    Vector2d bluePark2 = new Vector2d(12, -60);
    Vector2d bluePark3 = new Vector2d(-12, -36);

    // Red Pose + Vector2d
    Pose2d redStartPose = new Pose2d(-64, 48, Math.toRadians(0));
    Vector2d redShoot = new Vector2d(-24, 12);
    Vector2d redTowardsFirstRing = new Vector2d(-48, 36);
    Vector2d redFirstRing = new Vector2d(-48, 72);
    Vector2d redSecondRing = new Vector2d(-24, 72);
    Vector2d redThirdRing = new Vector2d(48, 72);
    Vector2d redPark1 = new Vector2d(36, 36);
    Vector2d redPark2 = new Vector2d(12, 60);
    Vector2d redPark3 = new Vector2d(-12, 36);


    public AutonSequences(HardwareMap hardwareMap, AutonUtilities utilities)
    {
        this.utilities = utilities;
        drive = new SampleMecanumDrive(hardwareMap);

        // Blue trajectories
        blueFirstShootTrajectory = drive.trajectoryBuilder(blueStartPose)
                .splineTo(blueShoot, Math.toRadians(0))
                .build();
        blueRing1PickupTrajectory = drive.trajectoryBuilder(new Pose2d(blueShoot, Math.toRadians(0)))
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
        bluePark1Trajectory = drive.trajectoryBuilder(new Pose2d(blueShoot, Math.toRadians(0)))
                .splineTo(bluePark1, Math.toRadians(-90))
                .build();
        bluePark2Trajectory = drive.trajectoryBuilder(new Pose2d(blueShoot, Math.toRadians(0)))
                .splineTo(bluePark2, Math.toRadians(-90))
                .build();
        bluePark3Trajectory = drive.trajectoryBuilder(new Pose2d(blueShoot, Math.toRadians(0)))
                .splineTo(bluePark3, Math.toRadians(-90))
                .build();

        // Red trajectories
        redFirstShootTrajectory = drive.trajectoryBuilder(redStartPose)
                .splineTo(redShoot, Math.toRadians(0))
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

    public void blueOneFull()
    {
        drive.setPoseEstimate(blueStartPose);

        // Aim first ring
        utilities.aimLauncher(BLUE_MID_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(BLUE_MID_VELOCITY, FIRST_SHOT_FLYWHEEL_DURATION);

        // Trajectory to shooting position
        drive.followTrajectory(blueFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();
        utilities.wait(1000);

        // Pick up and shoot other rings
        blueIntake();

        // Park
        drive.followTrajectory(bluePark1Trajectory);
    }

    public void blueTwoFull()
    {
        drive.setPoseEstimate(blueStartPose);

        // Aim first ring
        utilities.aimLauncher(BLUE_LOW_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(BLUE_LOW_VELOCITY, FIRST_SHOT_FLYWHEEL_DURATION);

        // Trajectory to shooting position
        drive.followTrajectory(blueFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();
        utilities.wait(1000);

        // Pick up and shoot other rings
        blueIntake();

        // Park
        drive.followTrajectory(bluePark2Trajectory);
    }

    public void blueThreeFull()
    {
        drive.setPoseEstimate(blueStartPose);

        // Aim first ring
        utilities.aimLauncher(BLUE_HIGH_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(BLUE_HIGH_VELOCITY, FIRST_SHOT_FLYWHEEL_DURATION);

        // Trajectory to shooting position
        drive.followTrajectory(blueFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();
        utilities.wait(1000);

        // Pick up and shoot other rings
        blueIntake();

        // Park
        drive.followTrajectory(bluePark3Trajectory);
    }

    public void blueIntake()
    {
        // Pick up three rings
        utilities.intake(true);
        drive.followTrajectory(blueRing1PickupTrajectory);
        utilities.wait(1000);
        drive.followTrajectory(blueRing2PickupTrajectory);
        utilities.wait(1000);
        drive.followTrajectory(blueRing3PickupTrajectory);
        utilities.intake(false);

        // Aim ring
        utilities.aimLauncher(BLUE_HIGH_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(BLUE_HIGH_VELOCITY, MULTI_SHOT_FLYWHEEL_DURATION);

        drive.followTrajectory(blueTripleShootTrajectory);

        // Shoot three rings into high goal
        for(int i = 0; i<3; i++)
        {
            utilities.shoot();
            utilities.wait(1000);
        }
    }

    public void redOneFull()
    {
        drive.setPoseEstimate(redStartPose);

        // Aim first ring
        utilities.aimLauncher(RED_MID_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(RED_MID_VELOCITY, FIRST_SHOT_FLYWHEEL_DURATION);

        // Trajectory to shooting position
        drive.followTrajectory(redFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();
        utilities.wait(1000);

        // Pick up and shoot other rings
        redIntake();

        // Park
        drive.followTrajectory(redPark1Trajectory);
    }
    public void redTwoFull()
    {
        drive.setPoseEstimate(redStartPose);

        // Aim first ring
        utilities.aimLauncher(RED_LOW_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(RED_LOW_VELOCITY, FIRST_SHOT_FLYWHEEL_DURATION);

        // Trajectory to shooting position
        drive.followTrajectory(redFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();
        utilities.wait(1000);

        // Pick up and shoot other rings
        redIntake();

        // Park
        drive.followTrajectory(redPark2Trajectory);
    }
    public void redThreeFull()
    {
        drive.setPoseEstimate(redStartPose);

        // Aim first ring
        utilities.aimLauncher(RED_HIGH_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(RED_HIGH_VELOCITY, FIRST_SHOT_FLYWHEEL_DURATION);

        // Trajectory to shooting position
        drive.followTrajectory(redFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();
        utilities.wait(1000);

        // Pick up and shoot other rings
        redIntake();

        // Park
        drive.followTrajectory(redPark3Trajectory);
    }

    public void redIntake()
    {
        // Pick up three rings
        utilities.intake(true);
        drive.followTrajectory(redRing1PickupTrajectory);
        utilities.wait(1000);
        drive.followTrajectory(redRing2PickupTrajectory);
        utilities.wait(1000);
        drive.followTrajectory(redRing3PickupTrajectory);
        utilities.intake(false);

        // Aim ring
        utilities.aimLauncher(RED_HIGH_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(RED_HIGH_VELOCITY, MULTI_SHOT_FLYWHEEL_DURATION);

        drive.followTrajectory(redTripleShootTrajectory);

        // Shoot three rings into high goal
        for(int i = 0; i<3; i++)
        {
            utilities.shoot();
            utilities.wait(1000);
        }
    }

    public void blueOnePart()
    {
        drive.setPoseEstimate(blueStartPose);

        // Aim first ring
        utilities.aimLauncher(BLUE_MID_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(BLUE_MID_VELOCITY, FIRST_SHOT_FLYWHEEL_DURATION);

        // Trajectory to shooting position
        drive.followTrajectory(blueFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();
        utilities.wait(1000);

        // Park
        drive.followTrajectory(bluePark1Trajectory);
    }
    public void blueTwoPart()
    {
        drive.setPoseEstimate(blueStartPose);

        // Aim first ring
        utilities.aimLauncher(BLUE_LOW_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(BLUE_LOW_VELOCITY, FIRST_SHOT_FLYWHEEL_DURATION);

        // Trajectory to shooting position
        drive.followTrajectory(blueFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();
        utilities.wait(1000);

        // Park
        drive.followTrajectory(bluePark2Trajectory);
    }
    public void blueThreePart()
    {
        drive.setPoseEstimate(blueStartPose);

        // Aim first ring
        utilities.aimLauncher(BLUE_HIGH_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(BLUE_HIGH_VELOCITY, FIRST_SHOT_FLYWHEEL_DURATION);

        // Trajectory to shooting position
        drive.followTrajectory(blueFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();
        utilities.wait(1000);

        // Park
        drive.followTrajectory(bluePark3Trajectory);
    }

    public void redOnePart()
    {
        drive.setPoseEstimate(redStartPose);

        // Aim first ring
        utilities.aimLauncher(RED_MID_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(RED_MID_VELOCITY, FIRST_SHOT_FLYWHEEL_DURATION);

        // Trajectory to shooting position
        drive.followTrajectory(redFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();
        utilities.wait(1000);

        // Park
        drive.followTrajectory(redPark1Trajectory);
    }
    public void redTwoPart()
    {
        drive.setPoseEstimate(redStartPose);

        // Aim first ring
        utilities.aimLauncher(RED_LOW_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(RED_LOW_VELOCITY, FIRST_SHOT_FLYWHEEL_DURATION);

        // Trajectory to shooting position
        drive.followTrajectory(redFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();
        utilities.wait(1000);

        // Park
        drive.followTrajectory(redPark2Trajectory);
    }
    public void redThreePart()
    {
        drive.setPoseEstimate(redStartPose);

        // Aim first ring
        utilities.aimLauncher(RED_HIGH_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(RED_HIGH_VELOCITY, FIRST_SHOT_FLYWHEEL_DURATION);

        // Trajectory to shooting position
        drive.followTrajectory(redFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();
        utilities.wait(1000);

        // Park
        drive.followTrajectory(redPark3Trajectory);
    }
}
