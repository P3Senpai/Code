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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class FalconTech_Jr_Basic_Bot
{
    /* Public OpMode members. */
    // I will need to add the other functions for the suction and the lifting for e.g. \\\\\\\\\\\\\\imp!
    // I will need to ask if we will order another REV expansion hub this year in time for the the comp \\\ imp!!!
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  linearLift  = null;
    public Servo    jewelArm    = null;

    // these are if we have 2 REV expansion hubs
    public DcMotor  rightIntake = null;
    public DcMotor  leftIntake = null; // I will need to set intake directions  /////////////////imp !!!
    public DcMotor  tilt       = null;

    /*this is only if we have one REV expansion hub*/
//    public DcMotor  intake      = null;
//    public Servo    leftTilt    = null;
//    public Servo    rightTilt   = null;

    // I will also need to give the suction a constant power
    public static final double intakeSpeed = 0.5;


    /* VALUES FOR THE LINEAR LIFT LIMITATIONS*/
    public final int limitMIN = 0;
    public final int limitMAX = 5;// value for limit needs to be tested in order to see if you need to change it
    public int limitValue = 0; // starts at resting position with 0 but isn't final so it can be changed

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public FalconTech_Jr_Basic_Bot(){}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /* INITIALIZING OF HARDWARE COMPONENTS*/
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");

        jewelArm   = hwMap.get(Servo.class,   "jewel_arm");

        linearLift = hwMap.get(DcMotor.class, "linear_lift"); // should i set a direction for it????
        rightIntake = hwMap.get(DcMotor.class, "right_intake"); // ask daniel about the scientific discovery (ftc)
        leftIntake  = hwMap.get(DcMotor.class, "left_intake");
        tilt        = hwMap.get(DcMotor.class, "tilt");

        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        linearLift.setPower(0);
        rightIntake.setPower(0);
        leftIntake.setPower(0);
        tilt.setPower(0);
        jewelArm.setPosition(0.9);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed. \\\\\\\\\\\\\\\\\\\\\\\\\\\imp!!!
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linearLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
 }

