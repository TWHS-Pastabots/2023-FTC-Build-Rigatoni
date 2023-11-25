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
    Trajectory blueRing1PickupTrajectory;
    Trajectory blueRing2PickupTrajectory;
    Trajectory blueRing3PickupTrajectory;
    Trajectory blueTripleShootTrajectory;
    Trajectory blueFullPark1Trajectory;
    Trajectory blueFullPark2Trajectory;
    Trajectory blueFullPark3Trajectory;
    Trajectory blueSinglePark1Trajectory;
    Trajectory blueSinglePark2Trajectory;
    Trajectory blueSinglePark3Trajectory;


    Trajectory redRingPickupTrajectory;

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
        // Aim first ring
        utilities.aimLauncher(BLUE_MID_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(BLUE_MID_VELOCITY);
        // Shoot first ring
        utilities.shoot();

        // Pick up and shoot other rings
        blueIntake();

        // Park
        drive.followTrajectory(blueFullPark1Trajectory);
    }

    public void blueTwoFull()
    {
        // Aim first ring
        utilities.aimLauncher(BLUE_LOW_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(BLUE_LOW_VELOCITY);
        // Shoot first ring
        utilities.shoot();

        // Pick up and shoot other rings
        blueIntake();

        // Park
        drive.followTrajectory(blueFullPark2Trajectory);
    }

    public void blueThreeFull()
    {
        // Aim first ring
        utilities.aimLauncher(BLUE_HIGH_SERVO);
        // Set flywheel power
        utilities.flywheelVelocity(BLUE_HIGH_VELOCITY);
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
        drive.setPoseEstimate(blueStartPose);
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

}
