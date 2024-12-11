/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="MecanumDriveWIntake", group="Robot")
@Disabled
public class MecanumDriveWIntake extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor FrontLeft = null;
    public DcMotor FrontRight = null;
    public DcMotor BackLeft = null;
    public DcMotor BackRight = null;
    public DcMotor arm = null;
    public DcMotor wrist = null;
    public DcMotor slide = null;
    private CRServo intake = null;
    private Servo claw = null;

    // Claw Positions
    private static final double CLAW_OPEN_POSITION = 0.55;
    private static final double CLAW_CLOSED_POSITION = 1;

    // Claw toggle state
    private boolean clawOpen = true;
    private boolean lastBump = false;
    // private boolean lastHook = false;
    // private boolean lastGrab = false;

    @Override
    public void runOpMode() {
        FrontLeft  = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft  = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        slide = hardwareMap.get(DcMotor.class, "slide");
        intake = hardwareMap.get(CRServo.class, "intake");
        claw = hardwareMap.get(Servo.class, "claw");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        wrist.setDirection(DcMotor.Direction.REVERSE);
        slide.setDirection(DcMotor.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double FrontLeftPower;
        double FrontRightPower;
        double BackLeftPower;
        double BackRightPower;
        double drivePower;
        double drive;
        double turn;
        double strafe;
        drivePower = 1.0;
        double armpower;
        double wristpower;
        double slidepower;


        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            drive = gamepad1.left_stick_y;
            turn  = -gamepad1.right_stick_x;
            strafe = -gamepad1.left_stick_x;
            FrontLeftPower = -Range.clip(drive + turn + strafe,-drivePower,drivePower) ;
            FrontRightPower = Range.clip(drive - turn - strafe,-drivePower,drivePower) ;
            BackLeftPower = -Range.clip(drive + turn - strafe,-drivePower,drivePower) ;
            BackRightPower = Range.clip(drive - turn + strafe,-drivePower,drivePower) ;



            // Output the safe vales to the motor drives.
            FrontLeft.setPower(FrontLeftPower);
            FrontRight.setPower(FrontRightPower);
            BackLeft.setPower(BackLeftPower);
            BackRight.setPower(BackRightPower);

            // Control intake servo with triggerss
            if (gamepad1.right_trigger>0.1) {
                intake.setPower(1.0);
            } else if (gamepad1.left_trigger>0.1) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0);
            }

            // Toggle claw position when right_bumper is pressed
            if (gamepad1.right_bumper && !lastBump) {
                clawOpen = !clawOpen;
                if (clawOpen) {
                    claw.setPosition(CLAW_OPEN_POSITION);
                } else {
                    claw.setPosition(CLAW_CLOSED_POSITION);
                }
            }
            lastBump = gamepad1.right_bumper;

            //flex arm
            if(gamepad1.dpad_up)
            {
                armpower = 1.0;
            }
            else if(gamepad1.dpad_down)
            {
                armpower = -1.0;
            }
            else
            {
                armpower = 0.005;
            }
            //armpower = gamepad2.left_stick_y;
            arm.setPower(armpower);

            //flex wrist
            if(gamepad1.dpad_left)
            {
                wristpower = 1.0;
            }
            else if(gamepad1.dpad_right)
            {
                wristpower = -1.0;
            }
            else
            {
                wristpower = 0.005;
            }
            wrist.setPower(wristpower);

            //extend slide
            if(gamepad2.dpad_up)
            {
                slidepower = 1.0;
            }
            else if(gamepad2.dpad_down)
            {
                slidepower = -1.0;
            }
            else
            {
                slidepower = 0;
            }
            //armpower = gamepad2.left_stick_y;
            slide.setPower(slidepower);
            //wristpower = gamepad2.right_stick_y;
            //wrist.setPower(wristpower);

            // Send telemetry message to signify robot running;
            //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("FrontLeft",  "%.2f", FrontLeftPower);
            telemetry.addData("FrontRight",  "%.2f", FrontLeftPower);
            telemetry.addData("BackLeft",  "%.2f", FrontLeftPower);
            telemetry.addData("BackRight",  "%.2f", FrontLeftPower);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}


