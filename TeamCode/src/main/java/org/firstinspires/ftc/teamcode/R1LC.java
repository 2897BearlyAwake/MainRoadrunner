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

@Autonomous(name = "R1LC",group = "robot")
public class R1LC extends LinearOpMode {

    //Declare Arms, Wrist and Slide Motors
    private DcMotor armMotor = null;
    private DcMotor slideMotor = null;
    private DcMotor wristMotor = null;

    //Declare intake and claw servos
    private CRServo intake = null;
    private Servo claw = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup Hardware Profile
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        armMotor = hardwareMap.dcMotor.get("arm");
        slideMotor = hardwareMap.dcMotor.get("slide");
        wristMotor = hardwareMap.dcMotor.get("wrist");
        intake = hardwareMap.get(CRServo.class, "intake");
        claw = hardwareMap.get(Servo.class, "claw");

        // Constants Involved in ArmMotor
        int armDrivePosition = 200;
        int armUpPosition = 1270;
        int armDownPosition = 0;
        int armAllUpPosition = 3000;

        // Linear Slide Constants
        int slideoutPosition = 950;
        int slideinPosition = -3;

        // Wrist Constants
        int wristUpPosition = 0;
        int wristDownPosition = 330;

        // Reset the motor encoder so that it reads zero ticks
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        if (isStopRequested()) return;

        // Initial pose for Red Side Position 1
        Pose2d startPose = new Pose2d(-24, -66, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        // Raise Arm To Height Required to Drive
        armMotor.setTargetPosition(armDrivePosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        sleep(1000);

        // Push forward away from the wall
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .forward(12)
                .build();
        drive.followTrajectory(traj1);

        // Drive to position facing the net zone
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(),true)
                .lineToLinearHeading(new Pose2d(-40, -51, Math.toRadians(232)))
                .build();
        drive.followTrajectory(traj2);

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

        // Slide forward to net zone
        Trajectory trajdf = drive.trajectoryBuilder(traj2.end(),true)
                .forward(22)
                .build();
        drive.followTrajectory(trajdf);

        //Bend Wrist Down
        wristMotor.setTargetPosition(wristDownPosition);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setPower(.8);
        sleep(3000);

        // Eject Sample
        intake.setPower(-1);
        sleep(2000);
        intake.setPower(0);

        //Retract Wrist
        wristMotor.setTargetPosition(wristUpPosition);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setPower(.8);
        sleep(2000);

        //Slide back from net zone
        Trajectory trajdb = drive.trajectoryBuilder(trajdf.end(),true)
                .forward(-24)
                .build();
        drive.followTrajectory(trajdb);

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
        armMotor.setTargetPosition(armDownPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);

        //Slide to Ending Position
        Trajectory trajfin = drive.trajectoryBuilder(trajdb.end(),true)
                .lineToLinearHeading(new Pose2d(-36, -48, Math.toRadians(-225)))
                .build();
        drive.followTrajectory(trajfin);


    }
}

