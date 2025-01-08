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

@Autonomous(name = "R2LC",group = "robot")
public class R2LC extends LinearOpMode {

    //Declare Arms, Wrist and Slide Motors
    private DcMotor armMotor = null;
    private DcMotor slideMotor = null;
    private DcMotor wristMotor = null;

    //Declare intake and claw servos
    private CRServo intake = null;
    private Servo claw = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        armMotor = hardwareMap.dcMotor.get("arm");
        slideMotor = hardwareMap.dcMotor.get("slide");
        wristMotor = hardwareMap.dcMotor.get("wrist");
        intake = hardwareMap.get(CRServo.class, "intake");
        claw = hardwareMap.get(Servo.class, "claw");

        // Constants Involved in ArmMotor
        int armDrivePosition = 200;
        int armUpPosition = 1270;
        int armToBar = 2450;
        int armDownPosition = 0;
        int armAllUpPosition = 3000;

        // Linear Slide Constants (prob don't need)
        //int slideoutPosition = 300;
        //int slideinPosition = -3;

        // Wrist Constants
        int wristUpPosition = 0;
        int wristDownPosition = 400;
        int wristGroundIntakePosition = 300;

        // Claw Positions
        double clawClosed = 1;
        double clawOpen = 0.55;

        // Reset the motor encoder so that it reads zero ticks
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        // Set Initial pose
        Pose2d startPose = new Pose2d(24, -66, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        // Close Claw
        claw.setPosition(clawClosed);

        // Lift arm into the drive position
        armMotor.setTargetPosition(armDrivePosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        sleep(1000);


        // Roll forward off the wall
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .forward(12)
                .build();
        drive.followTrajectory(traj1);

        // Roll to face the submursible
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(),true)
                .lineToLinearHeading(new Pose2d(-4, -48, Math.toRadians(90)))
                .build();
        drive.followTrajectory(traj2);

        //Lift Arm Above the Bar
        armMotor.setTargetPosition(armToBar);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        sleep(1000);

        //Extend arm
//        slideMotor.setTargetPosition(slideoutPosition);
//        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slideMotor.setPower(1);
//        sleep(1000);

        // Roll closer to the submursible
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(),true)
                .forward(9)
                .build();
        drive.followTrajectory(traj3);

        //Hook Sample on Bar
        armMotor.setTargetPosition(armToBar + -780);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        sleep(1000);

        // Open Claw
        claw.setPosition(clawOpen);

        // Roll Back from the submursible
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end(),true)
                .forward(-10)
                .build();
        drive.followTrajectory(traj4);

        // Lift arm into the drive position
        armMotor.setTargetPosition(armDrivePosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        sleep(1000);

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end(),true)
                .lineToLinearHeading(new Pose2d(32, -48, Math.toRadians(90)))
                .build();
        drive.followTrajectory(traj5);

        //Slide to position in front of sample on ground
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end(),true)
                .lineToLinearHeading(new Pose2d(31.5, -34.5, Math.toRadians(20)))
                .build();
        drive.followTrajectory(traj6);

        //Bring Down Arm (trying to see if this code is needed or if it just conflicts)
       // armMotor.setTargetPosition(armDownPosition);
        //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armMotor.setPower(0.8);

        //Bend Wrist Down
        wristMotor.setTargetPosition(wristGroundIntakePosition);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setPower(.8);
        sleep(3000);

//        //Slide forward a little closer to the sample to intake it
//        Trajectory traj6 = drive.trajectoryBuilder(traj5.end(),true)
//                .lineToLinearHeading(new Pose2d(33, -48, Math.toRadians(30)))
//                .build();
//        drive.followTrajectory(traj6);

        // Intake Sample
        intake.setPower(0.6);
        sleep(2000);
        intake.setPower(0);

        //Retract Wrist
        wristMotor.setTargetPosition(wristUpPosition);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setPower(.8);
        sleep(2000);

        // Lift arm into the drive position
        armMotor.setTargetPosition(armDrivePosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        sleep(1000);
    }
}


