package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "LiftCode",group = "robot")
public class LiftCode extends LinearOpMode {
    //public DcMotor arm = null;
    private CRServo intake = null;
    private Servo claw = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Position of the arm when it's lifted
        int armUpPosition = 1270;
        int slideoutPosition = 950;

        // Position of the arm when it's down
        int armDownPosition = 0;
        int slideinPosition = 3;

        int wristUpPosition = 0;
        int wristDownPosition = 400;

        // Find a motor in the hardware map named "Arm Motor"
        DcMotor armMotor = hardwareMap.dcMotor.get("arm");
        DcMotor slideMotor = hardwareMap.dcMotor.get("slide");
        DcMotor wristMotor = hardwareMap.dcMotor.get("wrist");
        intake = hardwareMap.get(CRServo.class, "intake");
        claw = hardwareMap.get(Servo.class, "claw");

        // Reset the motor encoder so that it reads zero ticks
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Sets the starting position of the arm to the down position
//        armMotor.setTargetPosition(armDownPosition);
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();


        if (isStopRequested()) return;
        //Lift Arm parallel to floor
        armMotor.setTargetPosition(armUpPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);

        sleep(2050);

        //Slide out slide fully
        slideMotor.setTargetPosition(slideoutPosition);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(1);

        sleep(4000);
        // Get the current position of the armMotor
        //double position = slideMotor.getCurrentPosition();


        // Show the position of the armMotor on telemetry
        //telemetry.addData("Encoder Position", position);
        //telemetry.update();

        //Lift Arm fully up
        armMotor.setTargetPosition(3000);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);

        sleep(5000);

        //Bend Wrist Down
        wristMotor.setTargetPosition(wristDownPosition);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setPower(.5);

        sleep(6000);

        intake.setPower(-1);
        sleep(3000);
        intake.setPower(0);

        //next
        wristMotor.setTargetPosition(wristUpPosition);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setPower(.5);

        sleep(4000);

        armMotor.setTargetPosition(armUpPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);

        sleep(2050);

        slideMotor.setTargetPosition(slideinPosition);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(1);

        // Show the position of the armMotor on telemetry
        //telemetry.addData("Encoder Position", position);
        //telemetry.update();

        sleep(5000);

        armMotor.setTargetPosition(armDownPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        /*
        slideMotor.setTargetPosition(slideoutPosition);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(1);

        sleep(4000);

        wristMotor.setTargetPosition(wristDownPosition);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setPower(.8);

        sleep(3000);
        intake.setPower(0.5);
        sleep(3000);

        intake.setPower(0);

        wristMotor.setTargetPosition(wristUpPosition);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setPower(.5);

        sleep(2000);
        claw.setPosition(0.55);
        claw.setPosition(1);

        slideMotor.setTargetPosition(slideinPosition);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(1);

        sleep(5000);

        slideMotor.setPower(0);

        // Get the current position of the armMotor
        double position = armMotor.getCurrentPosition();

        // Get the target position of the armMotor
        double desiredPosition = armMotor.getTargetPosition();

        // Show the position of the armMotor on telemetry
        telemetry.addData("Encoder Position", position);

        // Show the target position of the armMotor on telemetry
        telemetry.addData("Desired Position", desiredPosition);

        telemetry.update();

        sleep(1000);
        //sleepy

        armMotor.setTargetPosition(armDownPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);

        // Get the current position of the armMotor
        position = armMotor.getCurrentPosition();

        // Get the target position of the armMotor
        desiredPosition = armMotor.getTargetPosition();

        // Show the position of the armMotor on telemetry
        telemetry.addData("Encoder Position", position);

        // Show the target position of the armMotor on telemetry
        telemetry.addData("Desired Position", desiredPosition);

        telemetry.update();

        sleep(2000);*/

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
