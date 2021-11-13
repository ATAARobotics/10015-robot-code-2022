import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "RED_Backup",group = "Redbot")
public class RED_Backup extends LinearOpMode {
    /* Declare OpMode members. */
    //config and create objects
    R_5_CONFIG robot   = new R_5_CONFIG();   // Use a SwatBot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    skystoneDetectorClass detector = new skystoneDetectorClass();
    PID_CONTROLLER pidRotate, pidDrive,pidDrive_Long;

    //define 1,2 and 3 skyblock posiions
    static int              GLOBAL_SKYBLOCK_POS = 1;
    static int              SECOND_SKYBLOCK_POS = 0;
    static int              THIRD_SKYBLOCK_POS = 0;
    static int              FOURTH_SKYBLOCK_POS = 0;
    //ratio=power multiplier
    static double              ratio               = 0.9;
    //boolean variables
    boolean                 FOUND_SKYSTONE  = false;
    boolean                 L_ARM_DOWN = false;
    boolean                 R_ARM_DOWN = false;
    boolean                 JumpOut = false;
    //create a new lastangles to enable drive PID straight,strafe and rotate
    Orientation lastAngles = new Orientation();
    //variables for PID
    double                  globalAngle, power = .55, correction, PIDCorrection,rotation;
    //Variable for right color sensor
    int[] vals;

    @Override
    public void runOpMode() {
        /////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////
        //INIT (STAGE 0)
        //enable OPENCV
        detector.setOffset(0f / 8f, 0f / 8f);
        detector.camSetup(hardwareMap);
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        power = power * ratio;
        //make sure the imu gyro is calibrated before continuing.
        //create pidRotate and pidDrive to be used later on
        pidRotate = new PID_CONTROLLER(0, 0, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is. This is 5% power adjust
        // per degree of error.
        pidDrive = new PID_CONTROLLER(.03, 0, 0);
        pidDrive_Long = new PID_CONTROLLER(.02, 0, 0);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        //Jumpout bool to tell loop to jump out if they see skystone
        boolean JumpOut = false;

        // Wait for the game to start (driver presses PLAY)
        //init arm positions
        sample_L_servo("open");
        sample_R_servo("open");
        waitForStart();
        //define pidDrive and pidDriveLong power,output and enable them
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();
        pidDrive_Long.setSetpoint(0);
        pidDrive_Long.setOutputRange(0, power);
        pidDrive_Long.setInputRange(-90, 90);
        pidDrive_Long.enable();

        //OPENCV scanning
        /////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////
        //ENABLE OPENCV AND USE IT TO SCAN FOR SKYSTONE (STAGE 1)
        while (opModeIsActive()) {
            //keep on updating vals until we jump out
            detector.updateVals();
            vals = detector.getVals();
            //telemetry to tell which stone pos is skystone in
            telemetry.addData("Values", vals[1] + " " + vals[0] + " " + vals[2]);
            telemetry.addData("Stone Position", detector.stone_loc(102));
            telemetry.update();
            if (detector.stone_loc(102) != 0 && JumpOut == false) {
                JumpOut = true;
                GLOBAL_SKYBLOCK_POS = detector.stone_loc(102);
                telemetry.update();
            }
            while (opModeIsActive()) {
                choosingState(power,"Backward");
                sleep(300);
                break;
            }
            robot.stop();

            switch (GLOBAL_SKYBLOCK_POS) {
                case 6:
                    //sample_L_servo("open");
                    while (opModeIsActive()) {
                        if (robot.dis_right.getDistance(DistanceUnit.INCH) < 28) {
                            choosingState(power, "Strafe Left");
                        } else {
                            break;
                        }
                    }
                    THIRD_SKYBLOCK_POS=5;
                    FOURTH_SKYBLOCK_POS=4;
                    break;
                case 5:
                    //sample_R_servo("open");
                    while (opModeIsActive()) {
                        if (robot.dis_right.getDistance(DistanceUnit.INCH) < 30) {
                            choosingState(power, "Strafe Left");
                        } else {
                            break;
                        }
                    }
                    THIRD_SKYBLOCK_POS=6;
                    FOURTH_SKYBLOCK_POS=4;
                    break;
                case 4:
                    //sample_R_servo("open");
                    while (opModeIsActive()) {
                        if (robot.dis_right.getDistance(DistanceUnit.INCH) > 28) {
                            choosingState(power, "Strafe Right");
                        } else {
                            break;
                        }
                    }
                    THIRD_SKYBLOCK_POS=6;
                    FOURTH_SKYBLOCK_POS=5;
                    break;

            }
            robot.stop();
            SECOND_SKYBLOCK_POS = GLOBAL_SKYBLOCK_POS - 3;

            while (opModeIsActive()) {
                if (robot.dis_front_right.getDistance(DistanceUnit.INCH) < 27) {
                    choosingState(power, "Backward");
                } else {
                    break;
                }
            }
            robot.stop();
            switch (GLOBAL_SKYBLOCK_POS) {
                case 5:
                    sample_R_servo("close");
                    break;
                case 6:
                    sample_L_servo("close");
                    break;
                case 4:
                    sample_R_servo("close");
                    break;
            }
            while (opModeIsActive()) {
                if (robot.dis_front_right.getDistance(DistanceUnit.INCH) > 26) {
                    choosingState(power, "Forward");
                } else {
                    break;
                }
            }
            switch (GLOBAL_SKYBLOCK_POS) {
                case 5:
                    sample_L_servo("init");
                    break;
                case 6:
                    sample_R_servo("init");
                    break;
                case 4:
                    sample_L_servo("init");
                    break;
            }

            robot.stop();
            robot.gyro_Turn(-power,0);
            while (opModeIsActive()) {
                if(robot.dis_right.getDistance(DistanceUnit.INCH)<50) {
                    choosingState(power,"Strafe Left");
                }
                else {
                    break;
                }
            }
            robot.stop();
            robot.gyro_Turn(-power,0);
            runtime.reset();
            //use time as distance sensors are unreliable here in dead zone

            while (opModeIsActive()&&runtime.milliseconds()<1200) {
                choosingState(power,"Strafe Left");
            }
            robot.stop();
            switch (GLOBAL_SKYBLOCK_POS) {
                case 6:
                    sample_L_servo("open");
                    break;
                case 5:
                    sample_R_servo("open");
                    break;
                case 4:
                    sample_R_servo("open");
                    break;
            }
            while (opModeIsActive()) {
                if (robot.dis_left.getDistance(DistanceUnit.INCH)<50) {
                    choosingState(power,"Strafe Right");
                }
                else {
                    break;
                }
            }
            runtime.reset();
            //use time to bypass dead zone again
            while (opModeIsActive()&&runtime.milliseconds()<900) {
                sample_R_servo("init");
                sample_L_servo("init");
                choosingState(0.6,"Strafe Right");

            }
            robot.stop();
            //strafe away from block wall so we dont collide with stones
            //and alter their position
            while (opModeIsActive()) {
                if (robot.dis_front_right.getDistance(DistanceUnit.INCH)>24) {
                    choosingState(power,"Forward");
                }
                else {
                    break;
                }
            }
            robot.stop();
            if (opModeIsActive()) {
                robot.gyro_Turn(-power,0);
                resetAngle();
                //recorrect again to be safe
                while (opModeIsActive()) {
                    if (robot.dis_right.getDistance(DistanceUnit.INCH)>20) {
                        choosingState(power, "Strafe Right");
                    }
                    else {
                        break;
                    }
                }
            }
            robot.stop();
            switch (SECOND_SKYBLOCK_POS) {
                case 1:
                    sample_R_servo("open");
                    break;
                case 2:
                    sample_R_servo("open");
                    break;
                case 3:
                    sample_R_servo("open");
                    break;
            }
            sleep(150);
            switch (SECOND_SKYBLOCK_POS) {
                case 3:
                    while (opModeIsActive()) {
                        if (robot.dis_right.getDistance(DistanceUnit.INCH) > 20) {
                            choosingState(power, "Strafe Right");
                        } else {
                            break;
                        }
                    }
                    break;
                case 2:
                    while (opModeIsActive()) {
                        if (robot.dis_right.getDistance(DistanceUnit.INCH) > 12) {
                            choosingState(power, "Strafe Right");
                        } else {
                            break;
                        }
                    }
                    break;
                case 1:
                    while (opModeIsActive()) {
                        if (robot.dis_right.getDistance(DistanceUnit.INCH) > 1) {
                            choosingState(power, "Strafe Right");
                        } else {
                            break;
                        }
                    }
                    break;

            }
            robot.stop();
            if (opModeIsActive()) {
                robot.gyro_Turn(-0.4,0);
            }
            while (opModeIsActive()) {
                if (robot.dis_front_right.getDistance(DistanceUnit.INCH) < 27) {
                    choosingState(power, "Backward");
                } else {
                    break;
                }
            }
            robot.stop();
            switch (SECOND_SKYBLOCK_POS) {
                case 1:
                    sample_R_servo("close");
                    break;
                case 2:
                    sample_R_servo("close");
                    break;
                case 3:
                    sample_R_servo("close");
                    break;
            }
            while (opModeIsActive()) {
                if (robot.dis_front_right.getDistance(DistanceUnit.INCH) > 26) {
                    choosingState(power, "Forward");
                } else {
                    break;
                }
            }
            robot.stop();
            if (opModeIsActive()) {
                robot.gyro_Turn(-power,0);
            }
            while (opModeIsActive()) {
                if(robot.dis_right.getDistance(DistanceUnit.INCH)<50) {
                    choosingState(power,"Strafe Left");
                }
                else {
                    break;
                }
            }
            robot.stop();
            robot.gyro_Turn(-power,0);
            runtime.reset();
            //use time as distance sensors are unreliable here in dead zone

            while (opModeIsActive()&&runtime.milliseconds()<1300) {
                choosingState(power,"Strafe Left");
            }
            robot.stop();
            switch (SECOND_SKYBLOCK_POS) {
                case 1:
                    sample_R_servo("open");
                    break;
                case 2:
                    sample_R_servo("open");
                    break;
                case 3:
                    sample_R_servo("open");
                    break;
            }
            choosingState(power,"Strafe Right");
            sleep(300);
            while (opModeIsActive()) {
                if (robot.dis_left.getDistance(DistanceUnit.INCH)<50) {
                    choosingState(power,"Strafe Right");
                }
                else {
                    break;
                }
            }
            runtime.reset();
            //use time to bypass dead zone again
            while (opModeIsActive()&&runtime.milliseconds()<800) {
                sample_R_servo("init");
                sample_L_servo("init");
                choosingState(0.6,"Strafe Right");

            }
            robot.stop();
            //strafe away from block wall so we dont collide with stones
            //and alter their position
            while (opModeIsActive()) {
                if (robot.dis_right.getDistance(DistanceUnit.INCH)>50) {
                    choosingState(power,"Strafe Right");
                }
                else {
                    break;
                }
            }
            robot.stop();
            switch (THIRD_SKYBLOCK_POS) {
                case 6:
                    sample_L_servo("open");
                    break;
                case 5:
                    sample_L_servo("open");
                    break;
            }
            switch (THIRD_SKYBLOCK_POS) {
                case 6:
                    while (opModeIsActive()) {
                        if (robot.dis_right.getDistance(DistanceUnit.INCH) > 32) {
                            choosingState(power, "Strafe Right");
                        } else {
                            break;
                        }
                    }
                    break;
                case 5:
                    while (opModeIsActive()) {
                        if (robot.dis_right.getDistance(DistanceUnit.INCH) > 26) {
                            choosingState(power, "Strafe Right");
                        } else {
                            break;
                        }
                    }
                    break;
            }
            robot.stop();
            if (opModeIsActive()) {
                robot.gyro_Turn(-0.4,0);
            }
            while (opModeIsActive()) {
                if (robot.dis_front_right.getDistance(DistanceUnit.INCH) < 27) {
                    choosingState(power, "Backward");
                } else {
                    break;
                }
            }
            robot.stop();
            switch (THIRD_SKYBLOCK_POS) {
                case 6:
                    sample_L_servo("close");
                    break;
                case 5:
                    sample_L_servo("close");
                    break;
            }
            while (opModeIsActive()) {
                if (robot.dis_front_right.getDistance(DistanceUnit.INCH) > 26) {
                    choosingState(power, "Forward");
                } else {
                    break;
                }
            }
            robot.stop();
            if (opModeIsActive()) {
                robot.gyro_Turn(-power,0);
            }
            while (opModeIsActive()) {
                if(robot.dis_right.getDistance(DistanceUnit.INCH)<50) {
                    choosingState(power,"Strafe Left");
                }
                else {
                    break;
                }
            }
            robot.stop();
            robot.gyro_Turn(-power,0);
            runtime.reset();
            //use time as distance sensors are unreliable here in dead zone

            while (opModeIsActive()&&runtime.milliseconds()<1400) {
                choosingState(power,"Strafe Left");
            }
            robot.stop();
            switch (THIRD_SKYBLOCK_POS) {
                case 6:
                    sample_L_servo("open");
                    break;
                case 5:
                    sample_L_servo("open");
                    break;
            }
            while (opModeIsActive()) {
                if (robot.dis_left.getDistance(DistanceUnit.INCH)<50) {
                    choosingState(power,"Strafe Right");
                }
                else {
                    break;
                }
            }
            runtime.reset();
            //use time to bypass dead zone again
            while (opModeIsActive()&&runtime.milliseconds()<800) {
                sample_R_servo("init");
                sample_L_servo("init");
                choosingState(0.6,"Strafe Right");

            }
            robot.stop();
            //strafe away from block wall so we dont collide with stones
            //and alter their position
            while (opModeIsActive()) {
                if (robot.dis_right.getDistance(DistanceUnit.INCH)>35) {
                    choosingState(power,"Strafe Right");
                }
                else {
                    break;
                }
            }
            robot.stop();
            switch (FOURTH_SKYBLOCK_POS) {
                case 4:
                    sample_R_servo("open");
                    break;
                case 5:
                    sample_R_servo("open");
                    break;
            }
            switch (FOURTH_SKYBLOCK_POS) {
                case 4:
                    while (opModeIsActive()) {
                        if (robot.dis_right.getDistance(DistanceUnit.INCH) > 28) {
                            choosingState(power, "Strafe Right");
                        } else {
                            break;
                        }
                    }
                    break;
                case 5:
                    while (opModeIsActive()) {
                        if (robot.dis_right.getDistance(DistanceUnit.INCH) > 33) {
                            choosingState(power, "Strafe Right");
                        } else {
                            break;
                        }
                    }
                    break;
            }
            robot.stop();
            if (opModeIsActive()) {
                robot.gyro_Turn(-0.4,0);
            }
            while (opModeIsActive()) {
                if (robot.dis_front_right.getDistance(DistanceUnit.INCH) < 27) {
                    choosingState(power, "Backward");
                } else {
                    break;
                }
            }
            robot.stop();
            switch (FOURTH_SKYBLOCK_POS) {
                case 4:
                    sample_R_servo("close");
                    break;
                case 5:
                    sample_R_servo("close");
                    break;
            }
            while (opModeIsActive()) {
                if (robot.dis_front_right.getDistance(DistanceUnit.INCH) > 26) {
                    choosingState(power, "Forward");
                } else {
                    break;
                }
            }
            robot.stop();
            if (opModeIsActive()) {
                robot.gyro_Turn(-power,0);
            }
            while (opModeIsActive()) {
                if(robot.dis_right.getDistance(DistanceUnit.INCH)<50) {
                    choosingState(power,"Strafe Left");
                }
                else {
                    break;
                }
            }
            robot.stop();
            robot.gyro_Turn(-power,0);
            runtime.reset();
            //use time as distance sensors are unreliable here in dead zone

            while (opModeIsActive()&&runtime.milliseconds()<1400) {
                choosingState(power,"Strafe Left");
            }
            robot.stop();
            switch (FOURTH_SKYBLOCK_POS) {
                case 4:
                    sample_R_servo("open");
                    break;
                case 5:
                    sample_R_servo("open");
                    break;
            }
            sample_R_servo("init");
            sample_L_servo("init");
            choosingState(power,"Strafe Right");
            sleep(1000);
            robot.stop();
            break;

        }
    }
            private void resetAngle()
    {
        //update lastAngles and reset globalAngle
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
        //get the z of our imu and based on DEGREES
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //delta angle is how many angles are we off by
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        //if we exceed -180 or 180, add or subtract 360 so we still stay in the range
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        //global angle += delta angle for update
        globalAngle += deltaAngle;
        //update lastangles
        lastAngles = angles;

        return globalAngle;
    }

    //DO NOT REPLACE OR REMOVE!!!!
    public void sample_L_servo(String motion){
        if(motion == "init"){
            robot.l_arm.setPosition(0.5);
            robot.l_gripper.setPosition(0.00);
        }
        if (motion =="open"){
            robot.l_arm.setPosition(0.15);
            robot.l_gripper.setPosition(0.40);
        }
        if (motion == "close"){
            robot.l_arm.setPosition(0.08);
            robot.l_gripper.setPosition(0.08);
        }
    }
    public void sample_R_servo(String motion){
        if(motion == "init"){
            robot.r_arm.setPosition(0.66);
            robot.r_gripper.setPosition(0.70);
        }
        if (motion =="open"){
            robot.r_arm.setPosition(0.94);
            robot.r_gripper.setPosition(0.35);
        }
        if (motion == "close"){
            robot.r_arm.setPosition(1);
            robot.r_gripper.setPosition(0.56);
        }
    }



    public void choosingState(double power,String Direction){
        if(opModeIsActive()){
            if (Direction=="Strafe Left") {
                //enable pidDrive so we can get the correction/error we need to keep on driving straight
                //MAY NEED TO BE CHANGED BASED ON WHAT TASK WE ARE GOING TO DO
                //if that case, then go to pidDrive and change the parameters
                //correction multiplied by -1 or another negative as a ratio
                correction = pidDrive.performPID(getAngle());
                correction = correction * -1;
                robot.fr_Drive.setPower(-power + correction);
                robot.br_Drive.setPower(power + correction);
                robot.fl_Drive.setPower(power - correction);
                robot.bl_Drive.setPower(-power - correction);

            }
            else if (Direction=="Strafe Right") {
                //enable pidDrive so we can get the correction/error we need to keep on driving straight
                //MAY NEED TO BE CHANGED BASED ON WHAT TASK WE ARE GOING TO DO
                //if that case, then go to pidDrive and change the parameters
                //correction multiplied by -1 or another negative as a ratio
                correction = pidDrive.performPID(getAngle());
                correction = correction * -1;
                robot.fr_Drive.setPower(power + correction);
                robot.br_Drive.setPower(-power + correction);
                robot.fl_Drive.setPower(-power - correction);
                robot.bl_Drive.setPower(power - correction);

            }
            else if (Direction=="Forward") {
                //enable pidDrive so we can get the correction/error we need to keep on driving straight
                //MAY NEED TO BE CHANGED BASED ON WHAT TASK WE ARE GOING TO DO
                //if that case, then go to pidDrive and change the parameters
                //correction multiplied by -1 or another negative as a ratio
                correction = pidDrive.performPID(getAngle());
                // set power levels.
                robot.fl_Drive.setPower(-power + correction);
                robot.bl_Drive.setPower(-power + correction);
                robot.fr_Drive.setPower(-power - correction);
                robot.br_Drive.setPower(-power - correction);
            }
            else if (Direction=="Backward") {
                //enable pidDrive so we can get the correction/error we need to keep on driving straight
                //MAY NEED TO BE CHANGED BASED ON WHAT TASK WE ARE GOING TO DO
                //if that case, then go to pidDrive and change the parameters
                //correction multiplied by -1 or another negative as a ratio
                correction = pidDrive.performPID(getAngle());
                // set power levels.
                robot.fl_Drive.setPower(power + correction);
                robot.bl_Drive.setPower(power + correction);
                robot.fr_Drive.setPower(power - correction);
                robot.br_Drive.setPower(power - correction);
            }
        }
        else {
            //disable robot driving
            robot.fl_Drive.setPower(0);
            robot.bl_Drive.setPower(0);
            robot.fr_Drive.setPower(0);
            robot.br_Drive.setPower(0);
        }

    }
}
