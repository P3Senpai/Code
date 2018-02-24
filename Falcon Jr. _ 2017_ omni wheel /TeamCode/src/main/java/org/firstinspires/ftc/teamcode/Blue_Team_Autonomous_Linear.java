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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor upDrive = null;
    public DcMotor downDrive = null;
//    public Servo leftUClaw = null;
//    public Servo rightUClaw = null;
    public Servo jewelArm = null;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        colorSensor.enableLed(true);

        jewelArm = hardwareMap.get(Servo.class, "jewelArm");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds

        runtime.reset();
        while(opModeIsActive()){
            armDown(2.0);//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\!!!imp test out values
            jewelPunch(0.5);
        }

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
        upDrive.setPower(0);
        downDrive.setPower(0);
//        leftUClaw.setPosition(1.0);
//        rightUClaw.setPosition(0.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
    ///////////////////////////////////////////////////////// this code below is for the blue team
    public void jewelPunch(double holdTime){

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && holdTimer.time()< holdTime) { //\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\!!!imp  for red ball value
            if (colorSensor.red() > 9 && holdTimer.time() < holdTime) {
                if (runtime.seconds() < 2) {
                    leftDrive.setPower(-0.6);
                    rightDrive.setPower(0.6);
                    upDrive.setPower(-0.6);
                    downDrive.setPower(0.6);

                    runtime.reset();
                }
                if (runtime.seconds() < 2) {
                    leftDrive.setPower(0.6);
                    rightDrive.setPower(-0.6);
                    upDrive.setPower(0.6);
                    downDrive.setPower(-0.6);

                    jewelArm.setPosition(0.0);
                    runtime.reset();
                }
            } else {
                if (runtime.seconds() < 2) {
                    leftDrive.setPower(0.6);
                    rightDrive.setPower(-0.6);
                    upDrive.setPower(0.6);
                    downDrive.setPower(-0.6);

                    runtime.reset();
                }
                if (runtime.seconds() < 2) {
                    leftDrive.setPower(-0.6);
                    rightDrive.setPower(0.6);
                    upDrive.setPower(-0.6);
                    downDrive.setPower(0.6);

                    jewelArm.setPosition(0.0);
                    runtime.reset();
                }
            }
        }
    }
    public void armDown (double holdTime){

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && holdTimer.time()< holdTime){
            jewelArm.setPosition(1.0);
            runtime.reset();
        }
    }
}
