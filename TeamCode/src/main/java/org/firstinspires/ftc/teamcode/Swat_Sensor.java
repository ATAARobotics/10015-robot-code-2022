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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code shows using two different light sensors:
 *   The Primary sensor shown in this code is a legacy NXT Light sensor (called "sensor_light")
 *   Alternative "commented out" code uses a MR Optical Distance Sensor (called "sensor_ods")
 *   instead of the LEGO sensor.  Chose to use one sensor or the other.
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set half way between the light and dark values.
 *   These values can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the senso on asnd off the white line and not the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD half way between the min and max.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="TeamBot Sensor Checking", group="TeamBot")
//@Disabled
public class Swat_Sensor extends LinearOpMode {

    /* Declare OpMode members. */
    R_5_CONFIG         robot   = new R_5_CONFIG();   // Use a Pushbot's hardware


    @Override
    public void runOpMode() {

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init(hardwareMap);


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            telemetry.addData("Mode", "gyro calibrating...");
            telemetry.update();
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the white line is seen OR the driver presses STOP;
        while (opModeIsActive() && !isStopRequested()) {
            //robot.gyro_Turn(0.5,90);
           // robot.drive_straight(0.5,0);
          // robot.strafe(0.7,"left");
            /**
            robot.arm_encoder(0.7,70);
            telemetry.addData("arm Reading:",robot.arm.getCurrentPosition());
            telemetry.update();
            sleep(2000);
            robot.arm_servo.setPosition(1);//close the arm servo
            sleep(1000);
            robot.arm_encoder(1,-70);
            telemetry.addData("arm Reading:",robot.arm.getCurrentPosition());
            telemetry.update();
             */

            telemetry.addData("Distance Left:",robot.dis_left.getDistance(DistanceUnit.INCH));
            telemetry.addData("Distance Right:",robot.dis_right.getDistance(DistanceUnit.INCH));
            telemetry.addData("Distance Rear:",robot.dis_rear.getDistance(DistanceUnit.INCH));
            telemetry.addData("Distance Front_RIGHT:",robot.dis_front_right.getDistance(DistanceUnit.INCH));
            telemetry.addData("Distance Front_LEFT:",robot.dis_front_left.getDistance(DistanceUnit.INCH));
            telemetry.addData("Gyro Reading:",robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();
        }

    }
}
