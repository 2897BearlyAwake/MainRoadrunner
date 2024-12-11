package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;



@Autonomous(name = "Red1Auto",group = "robot")
@Disabled
public class Red1Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Pose2d startPose = new Pose2d(60, -12, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        Trajectory traj = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(60, -60, Math.toRadians(-45)))
                .build();
        drive.followTrajectory(traj);

        Trajectory traj2 = drive.trajectoryBuilder(traj.end(), true)
                .lineToLinearHeading(new Pose2d(42, -46, Math.toRadians(180)))
                .build();
        drive.followTrajectory(traj2);

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), true)
                .lineToLinearHeading(new Pose2d(60, -60, Math.toRadians(-45)))
                .build();
        drive.followTrajectory(traj3);

        Trajectory traj4= drive.trajectoryBuilder(traj3.end(), true)
                .lineToLinearHeading(new Pose2d(42, -60, Math.toRadians(180)))
                .build();
        drive.followTrajectory(traj4);

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end(), true)
                .lineToLinearHeading(new Pose2d(60, -58, Math.toRadians(-45)))
                .build();
        drive.followTrajectory(traj5);

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end(), true)
                .lineToLinearHeading(new Pose2d(38, -55, Math.toRadians(225)))
                .build();
        drive.followTrajectory(traj6);

        Trajectory traj7= drive.trajectoryBuilder(traj6.end(), true)
                .lineToLinearHeading(new Pose2d(60, -58, Math.toRadians(-45)))
                .build();
        drive.followTrajectory(traj7);

        //The code above works really well now!! -Kelly


        //*GUYS LISTEN UP!!!!!! The next bit of code DOES NOT WORK but the premise is to get the blocks from the centre and then push them into the end-zone*
        //*PLLEEEEEEASEEEEEEEEEEEE CONTINUE THIS FOR ME - MAIA*



        /*Trajectory traj8 =  drive.trajectoryBuilder(traj7.end(), true)
                .lineToLinearHeading(new Pose2d(70, 0, Math.toRadians(180)))
                .build();
        drive.followTrajectory(traj8);

        Trajectory traj9 = drive.trajectoryBuilder(traj8.end(), true)
                .lineToLinearHeading(new Pose2d(12,0,Math.toRadians(180)))
                .build();
        drive.followTrajectory(traj9);

        Trajectory traj10 =  drive.trajectoryBuilder(traj9.end(), true)
                .lineToLinearHeading(new Pose2d(70, 0, Math.toRadians(180)))
                .build();
        drive.followTrajectory(traj10);

        Trajectory traj11 =  drive.trajectoryBuilder(traj10.end(), true)
                .lineToLinearHeading(new Pose2d(70, 0, Math.toRadians(180)))
                .build();
        drive.followTrajectory(traj11);

        Trajectory traj12 = drive.trajectoryBuilder(traj11.end(), true)
                .lineToLinearHeading(new Pose2d(12,0,Math.toRadians(180)))
                .build();
        drive.followTrajectory(traj12);



        Trajectory traj1= drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(24, -48, Math.toRadians(180)))
                .build();
        drive.followTrajectory(traj);

        Trajectory traj = drive.trajectoryBuilder(startPose)
              .splineTo(new Vector2d(56, 18), Math.toRadians(0))
            .build();
        drive.followTrajectory(traj);

        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
              .turn(Math.toRadians(90))
            .build();
        drive.followTrajectorySequence(ts);

        Trajectory traj2 = drive.trajectoryBuilder(traj.end(), true)
              .lineTo(new Vector2d(80, 95))
            .build();
        drive.followTrajectory(traj2);

        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
              .turn(Math.toRadians(180))
            .build();
        drive.followTrajectorySequence(ts);

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), true)
              .lineTo(new Vector2d(0, 0))
            .build();
        drive.followTrajectory(traj3);

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end(), true)
              .splineTo(new Vector2d(20, -20), Math.toRadians(180))
            .build();
        drive.followTrajectory(traj4);

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end(), true)
              .lineTo(new Vector2d(10, -30))
            .build();
        drive.followTrajectory(traj5);
        */
    }
}

