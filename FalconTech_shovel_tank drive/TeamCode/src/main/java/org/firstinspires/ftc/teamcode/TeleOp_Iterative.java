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

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

@TeleOp(name="TeleOp Iterative", group="Iterative Opmode")
//@Disabled
public class TeleOp_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private FalconTech_Jr_Basic_Bot yaoiBot = new FalconTech_Jr_Basic_Bot();  // autonomous will be called yuriBot
    // this will init the hardware
    // to get all the hardware initialized before just type "yaoiBot." and then the name of the hardware
    HardwareMap hwMap           =  null;

    /* CONTROLS FOR DRIVER */
//    Game_pad1 -
//        each stick controls the wheel side individually
//        Y button used to retract jewel arm if necessary
//        optional === triggers used to control the direction and spin of robot
//        B button will play reeses puffs
//    Game_Pad2 -
//        A button is used to reveres intake spin if necessary
//        X button is used to tilt the ramp to place cubes
//            ? (This might need one or 2 buttons depending on the code)
//        Y button will retract jewel arm if necessary
//        Dpad Up and Down used to control the linear lift
//        B button will play reeses puffs
//

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        yaoiBot.init(hwMap); // idk if hwMap will init all the instances in FalconTech_Jr_Basic_Bot \\\\\\imp!!!
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        /* DRIVING CALCULATIONS*/
        double leftPower;
        double rightPower;
        double gp1L =  gamepad1.left_stick_y; // I name the controls for driving
        double gp1R =  gamepad1.right_stick_x;
        leftPower    = Range.clip(gp1L , -1.0, 1.0) ; // sets the range on the control outputs
        rightPower   = Range.clip(gp1R , -1.0, 1.0) ;

        /* DRIVING LOOP */
        if (leftPower!=0||rightPower!=0){ // if right power or left power aren't equal to 0 the robot drives
            yaoiBot.leftDrive.setPower(leftPower);
            yaoiBot.rightDrive.setPower(rightPower); // I call the instances in the pet object
        }

        /* RAMP TILT IMPLEMENTATION */
        if (gamepad2.x){
            tiltRamp();
        }

        /* JEWEL ARM RETRACTION */
        if (gamepad1.y || gamepad2.y){
            yaoiBot.jewelArm.setPosition(0.9);
            telemetry.addLine("Shrivel Up"); // test out what the addLine  function does
            telemetry.update();
        }

        /* LINEAR LIFT IMPLEMENTATION */
        if (gamepad2.dpad_up == true && linearLimit(true)< yaoiBot.limitMAX){
            yaoiBot.linearLift.setPower(0.5); // power and motion of lift
        } else if (gamepad2.dpad_up == true && linearLimit(true)>= yaoiBot.limitMAX){
            yaoiBot.linearLift.setPower(0);
            telemetry.addLine("I can't go any higher");                 // informs driver about height limit
            telemetry.update();
        } else if (gamepad2.dpad_down == true && linearLimit(false)> yaoiBot.limitMIN){
            yaoiBot.linearLift.setPower(-0.5);
        } else if (gamepad2.dpad_down == true && linearLimit(false)<= yaoiBot.limitMIN){
            yaoiBot.linearLift.setPower(0);
            telemetry.addLine("I can't go any lower");                  // informs driver about height limit
            telemetry.update();
        }

        /* WHEEL INTAKE */
        yaoiBot.leftIntake.setPower(yaoiBot.intakeSpeed); // power adjustments must be made to both left and right intakes
        yaoiBot.rightIntake.setPower(yaoiBot.intakeSpeed);

        /* REVERSING OF WHEEL INTAKE */
        if (gamepad2.a){
            yaoiBot.leftIntake.setPower(-yaoiBot.intakeSpeed); // this might be slower than the normal intake rpm
            yaoiBot.rightIntake.setPower(-yaoiBot.intakeSpeed);
            telemetry.addLine("I'm reversing, dumbass");
            telemetry.update();
        }

        /* MEME MACHINE */
        if (gamepad1.b || gamepad2.b){
            // play reeses puffs
            // old name of file Reeses Puffs Rap (2009) w Lyrics

//            MediaPlayer ring= MediaPlayer.create(MainActivity.this,R.raw.ring);
//            ring.start();
        }
        /**Idea for the meme machine is that when you press the b button it randomly picks
         *  from a array of meme songs and plays one.
         *  -> For that i will need to create a new method that will contain the different arrays
         *  -> i can even add the telemetry data so that people can see the name of the song
         */

        /* DATA THE SHOWS ON THE PHONE */
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Lift limit", "limit (%.2f)", yaoiBot.limitValue);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    /* RAMP TILT METHOD */
    public void tiltRamp (){
        ElapsedTime boi = new ElapsedTime();
        boi.seconds();
        boi.reset();

        if (boi.time() <2) {
            yaoiBot.tilt.setPower(0.5);
        } else if (boi.time()>2 && boi.time()<4){
            yaoiBot.tilt.setPower(-0.5);
        }
         /** so this method moves the motors there for 2 seconds and back for 2 seconds
         * This means that the ramp goes up and down with one click of a button
         */
    }

    /* LIMITATION OF LINEAR LIFT MOTION METHOD */
    public int linearLimit (boolean meatBalls){

        if (meatBalls == true && yaoiBot.limitMIN < yaoiBot.limitMAX){ // true loop means that it moves up
            yaoiBot.limitValue++;
        } else if (meatBalls != true && yaoiBot.limitMAX >= yaoiBot.limitMIN){ // false loop means that it moves down
            yaoiBot.limitValue --;
        }
        return yaoiBot.limitValue;
    }

}
