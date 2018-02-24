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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Blue_Team_Autonomous_Linear", group="Linear OpMode")
//@Disabled
public class Blue_Team_Autonomous_Linear extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

   private ColorSensor colorSensor;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor extension = null;

    static final double     COUNTS_PER_MOTOR_REV    = 1680 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.25; /////////////////////////// test gear reduction
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     DRIVE_SPEED             = 0.6;
//    static final double     TURN_SPEED              = 0.5;

//    public Servo leftUClaw = null;
//    public Servo rightUClaw = null;
    private Servo jewelArm = null;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        colorSensor.enableLed(true);

        jewelArm = hardwareMap.get(Servo.class, "jewelArm");

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        extension = hardwareMap.get(DcMotor.class, "extension");


        telemetry.addData("Status", "Resetting Encoders");
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "StartingEncoders");
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Error", "I am Rebel");
        telemetry.update();

        jewelArm.setPosition(0.95);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // this is for moving into the safe zone // THIS WILL NEED TO CHANGE DEPENDING ON THE POSITION
        //for (int i; i=0;i<5 && opModeIsActive() ){

       // }

//            encoderDrive(DRIVE_SPEED,  32,  -32, 5.0);  // S1: move to the safe zone
//          encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: turn towards safe zone
//          encoderDrive(DRIVE_SPEED,  7,  7, 2.0);    // S3: Move into safe zone


        runtime.reset();

        armDown(2.0);
        jewelPunch(1.0);
        jewelArm.setPosition(0.95);

    //    encoderDrive(DRIVE_SPEED,  25,  25, 5.0);  // S1: move to the safe zone

//
//
//        leftDrive.setPower(0.5);
//        rightDrive.setPower(0.5);
//        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
//            telemetry.addData("Path", "Step 2: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//            runtime.reset();
//        }
//
//        // Step 2:  Spin right for 1.3 seconds
//        leftDrive.setPower(0.8);
//        rightDrive.setPower(-0.8);
//        upDrive.setPower(0.8);
//        downDrive.setPower(-0.8);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
//            telemetry.addData("Path", "Step 3: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//            runtime.reset();
//        }
//
//        // Step 3:  Drive Backwards for 1 Second
//        leftDrive.setPower(0.5);
//        rightDrive.setPower(0.5);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 0.2)) {
//            telemetry.addData("Path", "Step 4: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//            runtime.reset();
//        }
//        // Step 5: turn toward the cryptobox
//        leftDrive.setPower(-0.8);
//        rightDrive.setPower(0.8);
//        upDrive.setPower(-0.8);
//        downDrive.setPower(0.8);
//        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
//            telemetry.addData("Path", "Step 5: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//            runtime.reset();
//        }
//        // Step 6: Move towards cryptobox
//        leftDrive.setPower(0.5);
//        rightDrive.setPower(0.5);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 0.2)) {
//            telemetry.addData("Path", "Step 6: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//            runtime.reset();
//        }
//        // Step 7: Drop cube
//        leftUClaw.setPosition(0.1); // Might change direction of claw depending on tests
//        rightUClaw.setPosition(0.1);
//        while (opModeIsActive() && (runtime.seconds() < 0.2)) {
//            telemetry.addData("Path", "Step 6: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//            runtime.reset();
//        }
//        // Step 8: move away
//        leftDrive.setPower(0.5);
//        rightDrive.setPower(0.5);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 0.2)) {
//            telemetry.addData("Path", "Step 6: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//            runtime.reset();
//        }
        // Step 9:  Stop and close the claw.
        leftDrive.setPower(0);
        rightDrive.setPower(0);

//        leftUClaw.setPosition(1.0);
//        rightUClaw.setPosition(0.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
    ///////////////////////////////////////////////////////// this code below is for the red team
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftDrive.getCurrentPosition(),
                        rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    // JewelPunch + ArmDown
    //{}}{}{}{}{{}}{}}{}{}{}}{}{}{}{}{}{}{}{}{}{{}}{}{}}{{}{}}}{}{}{ JEWEL IS ONLY POSSIBLE IF THEY GIVE US A HUB + EXTENSION


    public void jewelPunch(double holdTime){

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && holdTimer.time()< holdTime) {
            if (!(colorSensor.blue() > colorSensor.red()) && holdTimer.time() < holdTime) {
                if (runtime.seconds() < 2) {
                    leftDrive.setPower(-0.6);
                    rightDrive.setPower(-0.6);

                    runtime.reset();
                }
                if (runtime.seconds() < 2) {
                    leftDrive.setPower(0.6);
                    rightDrive.setPower(0.6);

                    telemetry.addData("Jewel", "is",colorSensor.blue());
                    telemetry.update();
                    runtime.reset();
                }
            } else if (colorSensor.blue() > colorSensor.red() && holdTimer.time() < holdTime) {
                if (runtime.seconds() < 2) {
                    leftDrive.setPower(0.6);
                    rightDrive.setPower(0.6);


                    runtime.reset();
                }
                if (runtime.seconds() < 2) {
                    leftDrive.setPower(-0.6);
                    rightDrive.setPower(-0.6);

                    telemetry.addData("Jewel","is", colorSensor.red());
                    telemetry.update();
                    runtime.reset();
                }
            }
        }
    }

    public void armDown (double holdTime){

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && holdTimer.time()< holdTime){
            jewelArm.setPosition(0.15);                              // Change value // Test it // distance tests
            runtime.reset();
        }
    }
}
