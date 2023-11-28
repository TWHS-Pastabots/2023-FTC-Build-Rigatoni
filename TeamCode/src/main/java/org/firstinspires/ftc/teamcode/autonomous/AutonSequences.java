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
    Trajectory blueFullPark1Trajectory;
    Trajectory blueFullPark2Trajectory;
    Trajectory blueFullPark3Trajectory;
    Trajectory bluePartPark1Trajectory;
    Trajectory bluePartPark2Trajectory;
    Trajectory bluePartPark3Trajectory;
    // Red Trajectories
    Trajectory redFirstShootTrajectory;
    Trajectory redRing1PickupTrajectory;
    Trajectory redRing2PickupTrajectory;
    Trajectory redRing3PickupTrajectory;
    Trajectory redTripleShootTrajectory;
    Trajectory redFullPark1Trajectory;
    Trajectory redFullPark2Trajectory;
    Trajectory redFullPark3Trajectory;
    Trajectory redPartPark1Trajectory;
    Trajectory redPartPark2Trajectory;
    Trajectory redPartPark3Trajectory;

    // Blue Pose + Vector2d
    Pose2d blueStartPose = new Pose2d(10, -8, Math.toRadians(90));
    Vector2d blueHighShoot = new Vector2d(10, -8);
    Vector2d blueMidShoot = new Vector2d(10, -8);
    Vector2d blueLowShoot = new Vector2d(10, -8);
    Vector2d blueFirstRing = new Vector2d(10, -8);
    Vector2d blueSecondRing = new Vector2d(10, -8);
    Vector2d blueThirdRing = new Vector2d(10, -8);



    // Red Pose
    Pose2d redStartPose = new Pose2d(10, -8, Math.toRadians(90));

    public AutonSequences(HardwareMap hardwareMap, AutonUtilities utilities)
    {
        this.utilities = utilities;
        drive = new SampleMecanumDrive(hardwareMap);

        blueRing1PickupTrajectory = drive.trajectoryBuilder(blueStartPose)
                .splineTo(blueFirstRing, Math.toRadians(90))
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
        drive.followTrajectory(blueFullPark1Trajectory);
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
        drive.followTrajectory(blueFullPark2Trajectory);
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
        drive.followTrajectory(blueFullPark3Trajectory);
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
        drive.followTrajectory(redFullPark1Trajectory);
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
        drive.followTrajectory(redFullPark2Trajectory);
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
        drive.followTrajectory(redFullPark3Trajectory);
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
        drive.followTrajectory(bluePartPark1Trajectory);
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
        drive.followTrajectory(bluePartPark2Trajectory);
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
        drive.followTrajectory(bluePartPark3Trajectory);
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
        drive.followTrajectory(redPartPark1Trajectory);
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
        drive.followTrajectory(redPartPark2Trajectory);
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
        drive.followTrajectory(redPartPark3Trajectory);
    }
}
