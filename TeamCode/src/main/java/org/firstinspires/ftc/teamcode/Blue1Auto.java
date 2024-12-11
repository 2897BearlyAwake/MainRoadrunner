package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name = "Blue1Auto",group = "robot")
@Disabled
public class Blue1Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Pose2d startPose = new Pose2d(-60, -12, Math.toRadians(360));
        drive.setPoseEstimate(startPose);

        Trajectory traj = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(135)))
                .build();
        drive.followTrajectory(traj);

        Trajectory traj2 = drive.trajectoryBuilder(traj.end(), true)
                .lineToLinearHeading(new Pose2d(-42, -46, Math.toRadians(360)))
                .build();
        drive.followTrajectory(traj2);

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), true)
                .lineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(135)))
                .build();
        drive.followTrajectory(traj3);

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end(), true)
                .lineToLinearHeading(new Pose2d(-42, -60, Math.toRadians(360)))
                .build();
        drive.followTrajectory(traj4);

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end(), true)
                .lineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(135)))
                .build();
        drive.followTrajectory(traj5);

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end(), true)
                .lineToLinearHeading(new Pose2d(-38, -55, Math.toRadians(-45)))
                .build();
        drive.followTrajectory(traj6);

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end(), true)
                .lineToLinearHeading(new Pose2d(-60, 60 , Math.toRadians(135)))
                .build();
        drive.followTrajectory(traj7);

    }
}
