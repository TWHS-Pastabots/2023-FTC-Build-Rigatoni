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

    final double BLUE_HIGH_SERVO = 0;
    final double BLUE_MID_SERVO = 0;
    final double BLUE_LOW_SERVO = 0;
    final double RED_HIGH_SERVO = 0;
    final double RED_MID_SERVO = 0;
    final double RED_LOW_SERVO = 0;

    final double BLUE_HIGH_VELOCITY = 0;
    final double BLUE_MID_VELOCITY = 0;
    final double BLUE_LOW_VELOCITY = 0;
    final double RED_HIGH_VELOCITY = 0;
    final double RED_MID_VELOCITY = 0;
    final double RED_LOW_VELOCITY = 0;

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
    Vector2d blueShoot = new Vector2d(-24, -12);
    Vector2d blueTowardsFirstRing = new Vector2d(-48, -36);
    Vector2d blueFirstRing = new Vector2d(-48, -72);
    Vector2d blueSecondRing = new Vector2d(-24, -72);
    Vector2d blueThirdRing = new Vector2d(48, -72);
    Vector2d bluePark1 = new Vector2d(36, -36);
    Vector2d bluePark2 = new Vector2d(12, -60);
    Vector2d bluePark3 = new Vector2d(-12, -36);



    // Red Pose
    Pose2d redStartPose = new Pose2d(10, -8, Math.toRadians(90));

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
                .splineTo(bluePark1, Math.toRadians(90))
                .build();
        bluePark2Trajectory = drive.trajectoryBuilder(new Pose2d(blueShoot, Math.toRadians(0)))
                .splineTo(bluePark2, Math.toRadians(90))
                .build();
        bluePark3Trajectory = drive.trajectoryBuilder(new Pose2d(blueShoot, Math.toRadians(0)))
                .splineTo(bluePark3, Math.toRadians(90))
                .build();


    }

    public void blueOneFull()
    {
        drive.setPoseEstimate(blueStartPose);

        // Aim first ring
        utilities.aimLauncher(BLUE_MID_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(BLUE_MID_VELOCITY);

        // Trajectory to shooting position
        drive.followTrajectory(blueFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();

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
        utilities.flywheelVelocity(BLUE_LOW_VELOCITY);

        // Trajectory to shooting position
        drive.followTrajectory(blueFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();

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
        utilities.flywheelVelocity(BLUE_HIGH_VELOCITY);

        // Trajectory to shooting position
        drive.followTrajectory(blueFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();

        // Pick up and shoot other rings
        blueIntake();

        // Park
        drive.followTrajectory(bluePark3Trajectory);
    }

    public void blueIntake()
    {
        // Pick up three rings
        drive.followTrajectory(blueRing1PickupTrajectory);
        utilities.intake(4000);
        drive.followTrajectory(blueRing2PickupTrajectory);
        utilities.intake(4000);
        drive.followTrajectory(blueRing3PickupTrajectory);
        utilities.intake(4000);
        drive.followTrajectory(blueTripleShootTrajectory);

        // Aim

        // Shoot three rings into high goal
        for(int i = 0; i<3; i++)
        {
            utilities.shoot();
        }
    }

    public void redOneFull()
    {
        drive.setPoseEstimate(redStartPose);

        // Aim first ring
        utilities.aimLauncher(RED_MID_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(RED_MID_VELOCITY);

        // Trajectory to shooting position
        drive.followTrajectory(redFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();

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
        utilities.flywheelVelocity(RED_LOW_VELOCITY);

        // Trajectory to shooting position
        drive.followTrajectory(redFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();

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
        utilities.flywheelVelocity(RED_HIGH_VELOCITY);

        // Trajectory to shooting position
        drive.followTrajectory(redFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();

        // Pick up and shoot other rings
        redIntake();

        // Park
        drive.followTrajectory(redPark3Trajectory);
    }

    public void redIntake()
    {
        // Pick up three rings
        drive.followTrajectory(redRing1PickupTrajectory);
        utilities.intake(4000);
        drive.followTrajectory(redRing2PickupTrajectory);
        utilities.intake(4000);
        drive.followTrajectory(redRing3PickupTrajectory);
        utilities.intake(4000);
        drive.followTrajectory(redTripleShootTrajectory);

        // Aim

        // Shoot three rings into high goal
        for(int i = 0; i<3; i++)
        {
            utilities.shoot();
        }
    }

    public void blueOnePart()
    {
        drive.setPoseEstimate(blueStartPose);

        // Aim first ring
        utilities.aimLauncher(BLUE_MID_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(BLUE_MID_VELOCITY);

        // Trajectory to shooting position
        drive.followTrajectory(blueFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();

        // Park
        drive.followTrajectory(bluePark1Trajectory);
    }
    public void blueTwoPart()
    {
        drive.setPoseEstimate(blueStartPose);

        // Aim first ring
        utilities.aimLauncher(BLUE_LOW_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(BLUE_LOW_VELOCITY);

        // Trajectory to shooting position
        drive.followTrajectory(blueFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();

        // Park
        drive.followTrajectory(bluePark2Trajectory);
    }
    public void blueThreePart()
    {
        drive.setPoseEstimate(blueStartPose);

        // Aim first ring
        utilities.aimLauncher(BLUE_HIGH_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(BLUE_HIGH_VELOCITY);

        // Trajectory to shooting position
        drive.followTrajectory(blueFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();

        // Park
        drive.followTrajectory(bluePark3Trajectory);
    }

    public void redOnePart()
    {
        drive.setPoseEstimate(redStartPose);

        // Aim first ring
        utilities.aimLauncher(RED_MID_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(RED_MID_VELOCITY);

        // Trajectory to shooting position
        drive.followTrajectory(redFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();

        // Park
        drive.followTrajectory(redPark1Trajectory);
    }
    public void redTwoPart()
    {
        drive.setPoseEstimate(redStartPose);

        // Aim first ring
        utilities.aimLauncher(RED_LOW_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(RED_LOW_VELOCITY);

        // Trajectory to shooting position
        drive.followTrajectory(redFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();

        // Park
        drive.followTrajectory(redPark2Trajectory);
    }
    public void redThreePart()
    {
        drive.setPoseEstimate(redStartPose);

        // Aim first ring
        utilities.aimLauncher(RED_HIGH_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(RED_HIGH_VELOCITY);

        // Trajectory to shooting position
        drive.followTrajectory(redFirstShootTrajectory);

        // Shoot first ring
        utilities.shoot();

        // Park
        drive.followTrajectory(redPark3Trajectory);
    }
}
