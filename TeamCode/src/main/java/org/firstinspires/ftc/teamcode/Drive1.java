package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Drive1", group="Robot")

public class Drive1 extends LinearOpMode {
    private DcMotor armMotor = null;
    private DcMotor slideMotor = null;
    private DcMotor wristMotor = null;
    private Servo claw = null;
    private CRServo intake = null;

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

        //Add Intake to Hardware Map
        intake = hardwareMap.get(CRServo.class, "intake");

        //Arm Position Constants and variables.
        int armDrivePosition = 175;
        int armUpPosition = 1270;
        int armAllUpPosition = 3000;
        int armEncoder = 0;

        //Linear Slide Constants
        int slideoutPosition = 950;
        int slideinPosition = -3;
        int slideEncoder = 0;

        //Wrist Position Constants
        int wristDownPosition = 380;
        int wristFloorPosition = 250;
        int wristUpPosition = 0;
        int wristEncoder = 0;

        // Claw Positions
        double clawClosed = 1;
        double clawOpen = 0.55;

        boolean Extension = false;

        // Drive Factor
        double driveFactor = 1.0;
        boolean Slow = false;

        // Wrist down to floor
        boolean wristDown = false;

        waitForStart();

        if (isStopRequested()) return;

        // Close Claw
        claw.setPosition(clawClosed);

        //set encoder readings to 0
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Lift Arm Motor For Driving
        armMotor.setTargetPosition(armDrivePosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        sleep(1000);

        Pose2d startPose = new Pose2d(-36, -48, Math.toRadians(-225));
        drive.setPoseEstimate(startPose);

        Pose2d currentPose = null;
        while (opModeIsActive()) {

            // Basic Drive Code
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y/driveFactor,
                            -gamepad1.left_stick_x/driveFactor,
                            -gamepad1.right_stick_x/driveFactor
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
            if (gamepad2.y && !Extension){
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
                Extension = true;
            }
            if (gamepad2.y && Extension){
                //Retract Wrist
                wristMotor.setTargetPosition(wristUpPosition);
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(.8);
                sleep(2000);

                //Slide back from net zone
                currentPose = drive.getPoseEstimate();
                Trajectory trajrb = drive.trajectoryBuilder(currentPose)
                        .forward(-21.5)
                        .build();
                drive.followTrajectory(trajrb);

                //Lower Arm
                armMotor.setTargetPosition(armUpPosition+50);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.8);
                sleep(2050);

                //Bring in slide
                slideMotor.setTargetPosition(slideinPosition);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(1);
                sleep(4000);

                //Bring Down Arm
                armMotor.setTargetPosition(armDrivePosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.8);

                Extension = false;
            }

            // Failsafe if arm doesn't lift at start
            if (gamepad2.right_bumper){
                armMotor.setTargetPosition(armDrivePosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);
                sleep(1000);
            }

            // Control Intake on Stick 1
            if (gamepad1.left_trigger>0.1){
                intake.setPower(-1.0);
            }
            else if (gamepad1.right_trigger>0.1){
                intake.setPower(1.0);
            }
            else{
                intake.setPower(0);
            }

            if(gamepad1.dpad_up){
                armEncoder = armMotor.getCurrentPosition();
                armMotor.setTargetPosition(armEncoder+100);
                if(armEncoder>armAllUpPosition){
                    armMotor.setTargetPosition(armAllUpPosition);
                }
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);
            }
            if(gamepad1.dpad_down){
                armEncoder = armMotor.getCurrentPosition();
                armMotor.setTargetPosition(armEncoder-100);
                if(armEncoder<armDrivePosition){
                    armMotor.setTargetPosition(armDrivePosition);
                }
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);
            }

            if(gamepad1.dpad_left){
                slideEncoder = slideMotor.getCurrentPosition();
                slideMotor.setTargetPosition(slideEncoder+50);
                if(slideEncoder<slideinPosition){
                    slideMotor.setTargetPosition(slideinPosition);
                }
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(1);
            }
            if(gamepad1.dpad_right){
                slideEncoder = slideMotor.getCurrentPosition();
                slideMotor.setTargetPosition(slideEncoder-50);
                if(slideEncoder>slideoutPosition){
                    slideMotor.setTargetPosition(slideoutPosition);
                }
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(1);
            }

            // Change Drive Speed
            if (gamepad1.x && !Slow){
                driveFactor = 4.0;
                Slow = true;
            }
            else if (gamepad1.x && Slow){
                driveFactor = 1.0;
                Slow = false;
            }

            if (gamepad1.y  && !wristDown){
                wristMotor.setTargetPosition(wristFloorPosition);
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(.8);
                sleep(3000);
                wristDown = true;
            }
            else if (gamepad1.y && wristDown){
                wristMotor.setTargetPosition(wristUpPosition);
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(.8);
                sleep(3000);
                wristDown = false;
            }

            if(gamepad1.right_stick_y < -0.5){
                wristEncoder = wristMotor.getCurrentPosition();
                wristMotor.setTargetPosition(wristEncoder-5);
                if(wristEncoder<wristDownPosition){
                    wristMotor.setTargetPosition(wristDownPosition);
                }
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(0.8);
            }
            else if(gamepad1.right_stick_y > 0.5){
                wristEncoder = wristMotor.getCurrentPosition();
                wristMotor.setTargetPosition(wristEncoder+5);
                if(wristEncoder>wristUpPosition){
                    wristMotor.setTargetPosition(wristUpPosition);
                }
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(0.8);
            }
            else{
                wristMotor.setPower(0.0);
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