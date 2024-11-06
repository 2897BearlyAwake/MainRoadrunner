package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "drive")
public class TestClass extends LinearOpMode {
    //public DcMotor arm = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Position of the arm when it's lifted
        int armUpPosition = 1000;

        // Position of the arm when it's down
        int armDownPosition = 0;

        // Find a motor in the hardware map named "Arm Motor"
        DcMotor armMotor = hardwareMap.dcMotor.get("arm");

        // Reset the motor encoder so that it reads zero ticks
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Sets the starting position of the arm to the down position
        armMotor.setTargetPosition(armDownPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();


        if (isStopRequested()) return;

        armMotor.setTargetPosition(armUpPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);

        // Get the current position of the armMotor
        double position = armMotor.getCurrentPosition();

        // Get the target position of the armMotor
        double desiredPosition = armMotor.getTargetPosition();

        // Show the position of the armMotor on telemetry
        telemetry.addData("Encoder Position", position);

        // Show the target position of the armMotor on telemetry
        telemetry.addData("Desired Position", desiredPosition);

        telemetry.update();

        sleep(10000);
        //sleepy

        armMotor.setTargetPosition(armDownPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);

        // Get the current position of the armMotor
        position = armMotor.getCurrentPosition();

        // Get the target position of the armMotor
        desiredPosition = armMotor.getTargetPosition();

        // Show the position of the armMotor on telemetry
        telemetry.addData("Encoder Position", position);

        // Show the target position of the armMotor on telemetry
        telemetry.addData("Desired Position", desiredPosition);

        telemetry.update();

        sleep(2000);

        /*Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        drive.followTrajectory(traj);

        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );*/
    }
}
