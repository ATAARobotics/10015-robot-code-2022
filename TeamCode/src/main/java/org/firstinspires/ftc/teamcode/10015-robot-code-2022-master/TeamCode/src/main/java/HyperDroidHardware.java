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
 * Motor channel:  Front Left  drive motor:        "fl_drive"  Tetrix  1:40 Port 0
 * Motor channel:  Front Right drive motor:        "fr_drive" Tetrix  1:50 Port 1

 *
 *
 * Servo channel-3:  left gripper: "dumpster"
 * Servo channel-4:  right gripper: "claw"

 */
public class HyperDroidHardware
{
    /* Public OpMode members. */
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor intakeDrive = null;
    private DcMotorSimple duckDrive = null;
    private DcMotor linear_lift = null;
    private Servo dumpster = null;
    private Servo clampr = null;
    private Servo clampl = null;
    private Servo claw = null;
    private DcMotor sparkmini = null;
    private Servo arm = null;

    //Limit switches

    //Distance sensors

    //Color sensors

    //LED
    //Drive options
    public enum DRIVE_OPTION {
        STRAIGHT,
        TURN,

    }

    public enum INTAKE_OPTION {
        INTAKE,
        OUTTAKE
    }


    // The IMU sensor object
    public BNO055IMU imu = null;
    boolean gyro_ready = false;

    // State used for updating telemetry
    Orientation LastAngles;
    double globalAngle;
    double correction;

    // Motors' parameters, wheels as well
    static final double     HEADING_THRESHOLD       = 3;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.05;
    static final double     P_FOUNDATION_TURN_COEFF            = 0.15; // Larger is more responsive, but also less stable
    static final double angPerPow   = 0.005556;

    /* local OpMode members. */
    private HardwareMap hwMap           =  null;
    private ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public HyperDroidHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        //Drive motors
        leftDrive  = hwMap.get(DcMotor.class, "l_motor");
        rightDrive = hwMap.get(DcMotor.class, "r_motor");
        intakeDrive = hwMap.get(DcMotor.class, "INT");
        duckDrive = hwMap.get(DcMotorSimple.class, "duck_motor");
        clampl = hwMap.get(Servo.class, "clampl");
        clampr = hwMap.get(Servo.class, "clampr");
        arm = hwMap.get(Servo.class, "arm");
        dumpster = hwMap.get(Servo.class, "dumpster");
        claw = hwMap.get(Servo.class, "claw");
        linear_lift = hwMap.get(DcMotor.class, "linear_lift");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        linear_lift.setDirection(DcMotor.Direction.FORWARD);

        linear_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linear_lift.setTargetPosition(3000);

        linear_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear_lift.setPower(0.0);
        linear_lift.setTargetPosition(0);

        //Servos

        //color sensors

        //Distance sensors

        // Set all motors to zero power

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        // initial gyro sensor
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //Wait until the gyro sensor is calibrated
        while (!imu.isGyroCalibrated()) {
            gyro_ready = false;
        }
        gyro_ready = true;
    }

    /**
     *Gripper operation
     *
     */
    /**
    public void clamp(double angle,String motion) {
        if (motion=="open" ) {
            angle = angle*angPerPow;
            left_gripper.setPosition(angle);
            right_gripper.setPosition(angle);
        }
        if(motion=="close") {
            angle = -angle*angPerPow;
            left_gripper.setPosition(angle);
            right_gripper.setPosition(angle);
        }
    */


    public void strafe(double power,String direction){

    }

    public void stop(){
        // Stop all motion;
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);

    }
    public void drive(DRIVE_OPTION option,double power){
        switch (option){
            case STRAIGHT:
                leftDrive.setPower(power);
                rightDrive.setPower(power);
                break;
            case TURN:
                leftDrive.setPower(power);
                rightDrive.setPower(-power);
                break;

        }
    }

    public void intake(HyperDroidHardware.INTAKE_OPTION option, double power) {
        switch (option) {
            case INTAKE:
                break;
            case OUTTAKE:
                break;
        }

    }

    //**Gyro Section*************************************************//
    public double getError(double targetAngle) {
        double robotError;
        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (robotError > 180)  robotError -= 360;
        if (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    boolean onHeading(double power, double angle, double PCoeff,String option) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double  left_power = 0;
        double  right_power = 0;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            if(option == "rotate"){
                left_power  = 0;
                right_power = 0;
            }
            if (option == "straight"){
                left_power  = power;
                right_power = power;
            }
            onTarget = true;
        }
        else {
            if(option == "rotate") {
                steer = getSteer(error, PCoeff);
                left_power = -power * steer;
                right_power = -left_power;
            }
            if(option =="straight"){
                steer = 1.5 * getSteer(error, PCoeff);
                left_power  = -power * steer;
                right_power   = -left_power;
            }
        }

        // Send desired speeds to motors.
        leftDrive.setPower(left_power);
        rightDrive.setPower(right_power);

        return onTarget;
    }

    public void gyro_Turn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (!onHeading(speed, angle, P_TURN_COEFF,"rotate")) {
            // Update telemetry & Allow time for other processes to run.
            try {
                Thread.sleep(10);
            } catch(InterruptedException e) {
                System.out.println("got interrupted!");
            }
        }
    }

    public void foundation_gyro_Turn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (!onHeading(speed, angle, P_FOUNDATION_TURN_COEFF,"rotate")) {
            // Update telemetry & Allow time for other processes to run.
            try {
                Thread.sleep(10);
            } catch(InterruptedException e) {
                System.out.println("got interrupted!");
            }
        }
    }

    public void simpe_rotation (  double speed) {

        // keep looping while we are still active, and not on heading.
        leftDrive.setPower(speed);
        rightDrive.setPower(-speed);
    }

    public void drive_straight(double speed,double angle){
        while (!onHeading(speed, angle, P_TURN_COEFF,"straight")) {
            // Update telemetry & Allow time for other processes to run.
            try {
                Thread.sleep(10);
            } catch(InterruptedException e) {
                System.out.println("got interrupted!");
            }
        }
    }


    //DO NOT REMOVE OR REPLACE!!!!!!!!!!!!!!!

    /**
     * public double checkDir() {
     *         double correction,gain = .10;
     *         Orientation current_angles;
     *         double deltaAngle;
     *         current_angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
     *         deltaAngle = current_angles.firstAngle - LastAngles.firstAngle;
     *         if (deltaAngle < -180) {
     *             deltaAngle = deltaAngle + 360;
     *         } else if (deltaAngle > 180) {
     *             deltaAngle = deltaAngle - 360;
     *         }
     *         globalAngle = globalAngle + deltaAngle;
     *         LastAngles = current_angles;
     *         if (globalAngle == 0) {
     *             correction = 0;
     *         } else {
     *             correction = globalAngle;
     *         }
     *         correction = correction * gain;
     *         return correction;
     *     }
     *
     *
     *     public void choosingState(double power,String Direction) {
     *         if (Direction=="Strafe Left") {
     *             correction = checkDir();
     *             fr_Drive.setPower(power - correction);
     *             fl_Drive.setPower(-power + correction);
     *             bl_Drive.setPower((power)+ correction);
     *             br_Drive.setPower(-power - correction);
     *
     *         }
     *         else if (Direction=="Strafe Right") {
     *             correction = checkDir();
     *             fr_Drive.setPower(-power - correction);
     *             fl_Drive.setPower(power + correction);
     *             bl_Drive.setPower(-(power)+ correction);
     *             br_Drive.setPower(power - correction);
     *
     *         }
     *         else if (Direction=="Backward") {
     *             correction = checkDir();
     *             fr_Drive.setPower(+power - correction);
     *             fl_Drive.setPower(+power + correction);
     *             bl_Drive.setPower(+power + correction);
     *             br_Drive.setPower(+power - correction);
     *         }
     *         else if (Direction=="Forward") {
     *             correction = checkDir();
     *             fr_Drive.setPower(-power - correction);
     *             fl_Drive.setPower(-power + correction);
     *             bl_Drive.setPower(-power + correction);
     *             br_Drive.setPower(-power - correction);
     *         }
     * /
     * @param speed
     * @param counts
     */


    //*************************************end of gyro section****************************************//

    //Arm encoder drive
    /**
    public void arm_encoder(double speed, int counts){
        int newTarget;
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Determine new target position, and pass to motor controller
        newTarget = arm.getCurrentPosition() + counts;
        arm.setTargetPosition(newTarget);
        // Turn On RUN_TO_POSITION
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // start motion.
        arm.setPower(abs(speed));
        while (arm.isBusy()) {
        }
        // Stop all motion;
        arm.setPower(0);
        // Turn off RUN_TO_POSITION
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
     */

    //Color sensor detection
    public float color_detect(NormalizedColorSensor sensor)
    {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
        colors.red   /= max;
        colors.green /= max; //later use only
        colors.blue  /= max; //later use only
        int color = colors.toColor();


        return Color.red(color);
    }



}

