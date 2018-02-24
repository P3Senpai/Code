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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOpMode_Iterative", group="Iterative Opmode")
//@Disabled
public class TeleOpMode_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    public  DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor upDrive = null;
    public DcMotor downDrive = null;

    public Servo armPivot = null;

    public Servo leftUClaw = null;
    public Servo rightUClaw = null;

    private Servo leftDClamp = null;
    private Servo rightDClamp = null;
    private Servo expandong = null;
    private Servo contratong = null;
    double servoMINPosition = 0.1;
    double servoMAXposition = 0.85; // when changing claw values change this
    boolean clawClosed;
    int changeDowg =0;


//    if (gamepad1.a){
//        if (clawClosed == true){
//            clawClosed = false;
//        } else {
//            clawClosed = true;
//        }
//}

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        upDrive = hardwareMap.get(DcMotor.class, "up_drive");
        downDrive = hardwareMap.get(DcMotor.class, "down_drive");

        leftUClaw = hardwareMap.get(Servo.class, "left_Clamp");
        rightUClaw = hardwareMap.get(Servo.class, "right_Clamp");
        armPivot = hardwareMap.get(Servo.class, "ArmPivot");

        // Most robots need the motor on one side to be reversed to drive forward

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        upDrive.setDirection(DcMotor.Direction.FORWARD);
        downDrive.setDirection(DcMotor.Direction.REVERSE);



        // I want the servos to move to the same place every time when they initialize
        this.rightUClaw.setPosition(servoMINPosition);
        this.leftUClaw.setPosition(-servoMINPosition);
        this.clawClosed =  false;
        telemetry.addData("Servo Status","Initialized");



    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() { runtime.reset();}
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double upPower;
        double downPower;
        double armPosition;
        double clawPosition = 0.0;
        double time = runtime.milliseconds();

        double yAxis = gamepad1.right_stick_x;// This is for the movement driver
        double xAxis = gamepad1.right_stick_y;

        double yLaxis = gamepad1.left_stick_y; //\\\\\\\\\\\ Will robot be controlled by 2 people??? \\\\\\\\\\\!!!imp

        leftPower = Range.clip(xAxis, -1.0, 1.0); // independent var solves prob when all work at once
        rightPower = Range.clip(xAxis, -1.0, 1.0);
        upPower = Range.clip(yAxis, -1.0, 1.0);
        downPower = Range.clip(yAxis, -1.0, 1.0);

        armPosition = Range.clip(yLaxis, -1.0, 1.0);// armPivot calculation
        clawPosition = Range.clip(clawPosition, servoMINPosition, servoMAXposition);

        // Moves straight vertically w/ left stick // MOVEMENT DRIVER
        if (yAxis > 0.13 || yAxis < -0.13) {
            upDrive.setPower(upPower);
            downDrive.setPower(downPower);
        } else if ((!gamepad1.left_bumper) && (!gamepad1.right_bumper)) {
            upDrive.setPower(0);
            downDrive.setPower(0);
        }
        // Moves straight horizontally w/ left stick // MOVEMENT DRIVER
        if (xAxis > 0.13 || xAxis < -0.13) {
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
        } else if ((!gamepad1.left_bumper) && (!gamepad1.right_bumper)) {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }

        if (gamepad1.left_bumper) {// Rotating to the  (anti-clockwise) // MOVEMENT DRIVER
            leftDrive.setPower(0.6);
            rightDrive.setPower(-0.6);
            upDrive.setPower(0.6);
            downDrive.setPower(-0.6);
        }
        if (gamepad1.right_bumper) { // Rotatiting to the left (clockwise) // MOVEMENT DRIVER
            leftDrive.setPower(-0.6);
            rightDrive.setPower(0.6);
            upDrive.setPower(-0.6);
            downDrive.setPower(0.6);
        }

        //claw device with two servos  \\\\\\\\\\\ Will robot be controlled by 2 people??? \\\\\\\\\\\!!!imp
        if ((gamepad1.a )&& (!this.clawClosed)) { // See if i can fix claw orientation\\\\\\\\\\\\\\\\\\\\\\\\!!!imp
        rightUClaw.setPosition(servoMAXposition -clawPosition);
            leftUClaw.setPosition(-servoMAXposition + clawPosition);
            this.clawClosed = true;
            //®time
        }else {
            this.clawClosed = false;
        }
        if ((gamepad1.a)&& (this.clawClosed)){
            rightUClaw.setPosition(servoMINPosition - clawPosition); // Why is the claw position plus or minus
            leftUClaw.setPosition(-servoMINPosition + clawPosition); // It is there to display the values on the phone
            this.clawClosed = false;
        } else{
            this.clawClosed = true;
        }

        //Servo controling arm angle
        if (gamepad1.dpad_up ) { // see if this can be solved with a while loop??? \\\\\\\\\\\\\\\\\!!!imp
            armPivot.setPosition(1);
        } else if (gamepad1.dpad_down) {
            armPivot.setPosition(0);
        }

        //__________________________________________________________________________\\
        //Servo exansion arm
        //            if (gamepad1.dpad_up == true) {
//                if (servoPosition != 1) {
//                    servoPosition += 0.05;
//                }
//            }
//            expandong.setPosition(servoPosition);
//            // servo contraction arm
//            if (gamepad1.dpad_down == true) {
//                if (servoPosition != 0) {
//                    servoPosition -= 0.05;
//                }
//            }
//            contratong.setPosition(servoPosition);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f), up (%.2f), down(%.2f)", leftPower, rightPower, upPower, downPower);
        //how to show telementary data for servos
        telemetry.addData("Servo", ", claw Position (%.2f), arm Pivot (%.2f)",clawPosition, armPosition);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
