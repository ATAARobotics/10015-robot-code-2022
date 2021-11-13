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

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Front Left  drive motor:        "fl_drive"  Tetrix  1:60 Port 0
 * Motor channel:  Front Right drive motor:        "fr_drive" Tetrix  1:60 Port 1
 * Motor channel:  Back Left drive motor:        "bl_drive" Tetrix 1:60 Port 2
 * Motor channel:  Back Right drive motor:        "br_drive" Tetrix 1:60 Port 3
 *
 *
 * Servo channel-0:  left gripper: "left_gripper"
 * Servo channel-1:  right gripper: "right_gripper"
 *
 * Left Color sensor        : "left_color"      Hub#1   I2C     port 1
 * Right Color sensor       : "right_color"     Hub#1   I2C     port 3
 *
 * Left Side Distance sensor     :   "dis_left"  Hub#1   I2C     port 2
 * Right Side Distance sensor     :   "dis_right"  Hub#2   I2C     port 2
 * Rear Distance sensor         :      "dis_rear"   Hub#2  I2C     port 3
 * Front Distance sensor        :       "dis_front"     Hub#2   I2C   port 1
 */
public class TeamRobot_R5
{
    /* Public OpMode members. */
    public DcMotor  fl_Drive   = null;
    public DcMotor  fr_Drive  = null;
    public DcMotor  bl_Drive   = null;
    public DcMotor  br_Drive  = null;
    public DcMotor  linear_lift    = null;
    public DcMotor  l_intake = null;
    public DcMotor  r_intake = null;



    //Servos
    public Servo    left_gripper  = null;
    public Servo    right_gripper  = null;
    public Servo    arm_servo  = null;
    public Servo    left_hook = null;
    public Servo    right_hook = null;
    public Servo    linear_arm = null;
    public Servo    clamp = null;
    public static Servo  arm    = null;
    public static Servo  arm2   = null;
    public static Servo  foundation_clamp = null;



    //Limit switches
    public DigitalChannel limit_switch = null;

    //LED
    //public RevBlinkinLedDriver LED;

    //Drive options
    public enum DRIVE_OPTION {
        STRAIGHT,
        TURN,
        RIGHTSTRAFE,
        LEFTSTRAFE,
        DIAGONALSTRAFERIGHT,
        DIAGONALSTRAFELEFT,
    }

    public enum INTAKE_OPTION {
        INTAKE,
        OUTTAKE
    }

    /* local OpMode members. */
    private HardwareMap hwMap           =  null;
    private ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public TeamRobot_R5(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        //Drive motors
        fl_Drive = hwMap.get(DcMotor.class, "fl_drive");
        fr_Drive = hwMap.get(DcMotor.class, "fr_drive");
        bl_Drive = hwMap.get(DcMotor.class, "bl_drive");
        br_Drive = hwMap.get(DcMotor.class, "br_drive");
        l_intake = hwMap.get(DcMotor.class,"l_intake");
        r_intake = hwMap.get(DcMotor.class,"r_intake");
        linear_lift = hwMap.get(DcMotor.class, "linear_lift");

        linear_arm = hwMap.get(Servo.class, "arm");
        clamp = hwMap.get(Servo.class, "clamp");
        arm   = hwMap.get(Servo.class,"l_arm");
        arm2  = hwMap.get(Servo.class,"r_arm");
        foundation_clamp = hwMap.get(Servo.class,"foundation_servo");


        //linear_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fl_Drive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        bl_Drive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        fr_Drive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        br_Drive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        //arm.setDirection(DcMotor.Direction.REVERSE);
        l_intake.setDirection(DcMotor.Direction.FORWARD);
        r_intake.setDirection(DcMotor.Direction.REVERSE);

        fl_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linear_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set all motors to zero power
        fl_Drive.setPower(0);
        bl_Drive.setPower(0);
        fr_Drive.setPower(0);
        fr_Drive.setPower(0);
        linear_lift.setPower(0);
        l_intake.setPower(0);
        r_intake.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        fl_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linear_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    public void stop(){
        // Stop all motion;
        fl_Drive.setPower(0);
        bl_Drive.setPower(0);
        fr_Drive.setPower(0);
        br_Drive.setPower(0);
        l_intake.setPower(0);
        r_intake.setPower(0);
    }
    public void drive(DRIVE_OPTION option,double power){
        switch (option){
            case STRAIGHT:
                fl_Drive.setPower(power);
                bl_Drive.setPower(power);
                fr_Drive.setPower(power);
                br_Drive.setPower(power);
                break;
            case TURN:
                fl_Drive.setPower(power);
                bl_Drive.setPower(power);
                fr_Drive.setPower(-power);
                br_Drive.setPower(-power);
                break;
            case LEFTSTRAFE:
                fl_Drive.setPower(power);
                bl_Drive.setPower(-power);
                fr_Drive.setPower(-power);
                br_Drive.setPower(power);
                break;
            case RIGHTSTRAFE:
                fl_Drive.setPower(-power);
                bl_Drive.setPower(power);
                fr_Drive.setPower(power);
                br_Drive.setPower(-power);
                break;
            case DIAGONALSTRAFELEFT:
                fl_Drive.setPower(power);
                br_Drive.setPower(power);
            case DIAGONALSTRAFERIGHT:
                fr_Drive.setPower(power);
                bl_Drive.setPower(power);
        }
    }

    public void intake(TeamRobot_R5.INTAKE_OPTION option, double power) {
        switch (option) {
            case INTAKE:
                l_intake.setPower(-power);
                r_intake.setPower(-power);
            case OUTTAKE:
                l_intake.setPower(-power);
                r_intake.setPower(-power);
        }

    }

}

