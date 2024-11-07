package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name = "Red2Auto",group = "robot")
public class Red2Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Pose2d startPose = new Pose2d(60, 12, Math.toRadians( 180));
        drive.setPoseEstimate(startPose);

        Trajectory traj2 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(24, 56, Math.toRadians(180)))
                .build();
        drive.followTrajectory(traj2);

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), true)
                .lineToLinearHeading(new Pose2d(60, -60, Math.toRadians(-45)))
                .build();
        drive.followTrajectory(traj3);

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end(), true)
                .lineToLinearHeading(new Pose2d(24, 64, Math.toRadians(180)))
                .build();
        drive.followTrajectory(traj4);

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end(), true)
                .lineToLinearHeading(new Pose2d(60, -60, Math.toRadians(-45)))
                .build();
        drive.followTrajectory(traj5);

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end(), true)
                .lineToLinearHeading(new Pose2d(24, 72, Math.toRadians(180)))
                .build();
        drive.followTrajectory(traj6);

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end(), true)
                .lineToLinearHeading(new Pose2d(60, -60 , Math.toRadians(-45)))
                .build();
        drive.followTrajectory(traj7);
    }
}