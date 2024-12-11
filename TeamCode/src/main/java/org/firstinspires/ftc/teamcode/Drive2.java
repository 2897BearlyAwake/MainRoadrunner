package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Drive2", group="Robot")

public class Drive2 extends LinearOpMode {
    private DcMotor armMotor = null;
    private DcMotor slideMotor = null;
    private DcMotor wristMotor = null;
    private Servo claw = null;

    @Override
    public void runOpMode() throws InterruptedException {
        //Establish MecanumDrive for RoadRunner
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Add Arm Motor to Hardware Map
        armMotor = hardwareMap.dcMotor.get("arm");

        //Add Slide Motor to Hardware Map
        slideMotor = hardwareMap.dcMotor.get("slide");

        //Add Wrist Motor to Hardware Map
        wristMotor = hardwareMap.dcMotor.get("wrist");

        //Add Claw to Hardware Map
        claw = hardwareMap.get(Servo.class, "claw");

        //Arm Position Constants
        int armDrivePosition = 200;
        int armUpPosition = 1270;
        int armAllUpPosition = 3000;

        //Linear Slide Constants
        int slideoutPosition = 950;

        //Wrist Position Constants
        int wristDownPosition = 380;


        // Claw Positions
        double clawClosed = 1;
        double clawOpen = 0.55;

        waitForStart();

        if (isStopRequested()) return;

        // Close Claw
        claw.setPosition(clawClosed);

        //set encoder readings to 0
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Lift Arm Motor For Driving
        armMotor.setTargetPosition(armDrivePosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        sleep(1000);

        Pose2d startPose = new Pose2d(36, -48, Math.toRadians(45));
        drive.setPoseEstimate(startPose);

        Pose2d currentPose = null;
        while (opModeIsActive()) {

            // Basic Drive Code
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            //If gamepad2 x button is pressed position by the goal
            if (gamepad2.x){
                currentPose = drive.getPoseEstimate();
                Trajectory netTraj = drive.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(new Pose2d(-39, -49, Math.toRadians(225)))
                        .build();
                drive.followTrajectory(netTraj);
            }

            //If gamepad2 y button is pressed score a sample
            if (gamepad2.y){
                //Lift Arm parallel to floor
                armMotor.setTargetPosition(armUpPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);
                sleep(1000);

                //Slide out slide fully
                slideMotor.setTargetPosition(slideoutPosition);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(1);
                sleep(3000);

                //Lift Arm fully up
                armMotor.setTargetPosition(armAllUpPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);
                sleep(2000);

                //Roll Forward
                currentPose = drive.getPoseEstimate();
                Trajectory trajrf = drive.trajectoryBuilder(currentPose)
                        .forward(21.5)
                        .build();
                drive.followTrajectory(trajrf);

                //Bend Wrist Down
                wristMotor.setTargetPosition(wristDownPosition);
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(.8);
                sleep(3000);
            }

            // Failsafe if arm doesn't lift at start
            if (gamepad2.right_bumper){
                armMotor.setTargetPosition(armDrivePosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);
                sleep(1000);
            }


            //Holding Code for Later Information
            //Pose2d poseEstimate = drive.getPoseEstimate();
            //telemetry.addData("x", poseEstimate.getX());
            //telemetry.addData("y", poseEstimate.getY());
            //telemetry.addData("heading", poseEstimate.getHeading());
            //telemetry.update();

        }
    }
}
