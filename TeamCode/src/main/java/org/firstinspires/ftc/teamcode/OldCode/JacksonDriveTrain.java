/**
 Drivetrain_.java
 Created by Jackson Bremen

 Changelog:
 9/12/19
 - Created file
 - Rotation values are almost certainly meaningless


 TODO:
 Get setup for encoders, if wanted. Last year we didn't have them for drivetrain tho

 */


package org.firstinspires.ftc.teamcode.OldCode;

import static java.lang.Math.*;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Utility.ServoClaw;

public class JacksonDriveTrain{

    /**

    Variables

     **/

    DcMotor front_left_motor, front_right_motor, rear_left_motor, rear_right_motor;
    DcMotor arm;
    DcMotor slide;
    DcMotor spinner;
    DcMotor stars;
    // DcMotor rope;
    // Servo grabL, grabR;
    Servo bucket;
    double internalHeading;
    DcMotor[] motorArray = new DcMotor[4];
    ColorSensor color;
    VoltageSensor[] voltageSensors;
    // -750
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, startAngle;
    final static double DEGREES_AT_FULL_POWER = 30;
    final static double STANDARD_VOLTAGE = 13;
    LinearOpMode op;
    long wait = 500;
    final static double[][] MOTOR_SIGNS = {
            //   fl fr bl br
            { 1, 1, 1, 1},    //Forward
            { 1,-1,-1, 1},    //Right
            {-1, 1,-1, 1},    //Counterclockwise
    };
    //final static double driveCalibration = 32;
    final static double driveCalibration = 32*100/91;
    final static double angleCalibration = 1;
    final static double SET_HEADING_THRESHOLD = 5;
    
    private ServoClaw bucket_thing;

    public void calibrate() {
        //if(imu.)
        op.idle();
    }

    /**

    Constructors

     **/

    public JacksonDriveTrain(LinearOpMode op) {
        this.op = op;
        bucket = op.hardwareMap.servo.get("bucket");
        spinner = op.hardwareMap.dcMotor.get("spinner");
        front_left_motor = op.hardwareMap.dcMotor.get("front left drive");
        front_right_motor = op.hardwareMap.dcMotor.get("front right drive");
        rear_left_motor = op.hardwareMap.dcMotor.get("back left drive");
        rear_right_motor = op.hardwareMap.dcMotor.get("back right drive");
        slide = op.hardwareMap.dcMotor.get("slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        //arm = op.hardwareMap.dcMotor.get("arm");
        motorArray[0] = front_left_motor;
        motorArray[1] = front_right_motor;
        motorArray[2] = rear_left_motor;
        motorArray[3] = rear_right_motor;
        for(DcMotor m:motorArray) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        front_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        rear_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        rear_right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        spinner.setDirection(DcMotorSimple.Direction.REVERSE);
        
        stars = op.hardwareMap.dcMotor.get("stars");
        
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = op.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        op.telemetry.addData("Initialized: ", "Interial Movement Unit");
        op.telemetry.update();

        color = op.hardwareMap.colorSensor.get("color");
        color.enableLed(true);
        
        
        //Handle servos
        this.bucket_thing = new ServoClaw(bucket, 0f, 1f);

    }

    /**

    Functions

     **/
    public void starsTest() {
        stars.setPower(1);
        op.sleep(500);
    }
    public void initBucket() {
        bucket.setPosition(0.6);
        op.sleep(100);
        op.telemetry.addData("Servo position: ", bucket.getPosition());
        op.telemetry.update();
    }
    public void dumpBucket(int ts_pos, double angle) {
        bucket.setPosition(0.7);
        op.sleep(100);
        op.telemetry.addData("Servo position: ", bucket.getPosition());
        op.telemetry.update();
        slide.setPower(0.6);
        if(ts_pos == 0) {
            op.sleep(2000);
            slide.setPower(0);
            driveAtHeading(angle, 0, 11, 0.5); 
        }
        // change to 180º
        if(ts_pos == 2) {
            op.sleep(1000);
            slide.setPower(0);
            driveAtHeading(angle, 0, -1.5, 0.5);
        }
        if(ts_pos == 1) {
            op.sleep(1700); 
            slide.setPower(0);
            driveAtHeading(angle, 0, 4.5, 0.5);
        }
        if(ts_pos == 0) {
            bucket.setPosition(0.1);
        }
        else {
            bucket.setPosition(0);
        }
        op.sleep(800);
        op.telemetry.addData("Servo position: ", bucket.getPosition());
        op.telemetry.update();
    }
    public void moveSlide(int direction, int ts_pos) {
        // direction == -1 is down
        // direction == 1 is up
        // ts_pos == 1 is lowest
        // ts_pos == 2 is mid
        // ts_pos == 3 is highest
        double pow =  direction * -0.7;
        slide.setPower(pow);
        if(ts_pos == 1) {
            op.sleep(1070);
        }
        else if(ts_pos == 2) {
            op.sleep(1600);
        }
        else if(ts_pos == 3){
            op.sleep(1800);
        }
        slide.setPower(0);
        op.sleep(100);
    }
    public void longBTilt(int pos) {
        // pos == 0 is tilt bucket to rest
        // pos == 1 is tilt bucket for lowest carousel level
        // pos == 2 is tilt bucket mid carousel level
        // pos == 3 is tilt bucket for highest carousel level
        if(pos == 0) {
            bucket.setPosition(1);
        }
        else if(pos == 1) {
            bucket.setPosition(0);
        }
        else if(pos == 2){
            bucket.setPosition(0.2);
        }
        else if(pos == 3) {
            bucket.setPosition(0.2);
        }
        op.sleep(1000);
    }
    public DcMotor[] getMotors() {
        return motorArray;
    }
    public void spinCarousel(double dir) {
        spinner.setPower(-0.7 * dir * getVoltage() / STANDARD_VOLTAGE);
        op.sleep(3200);
        spinner.setPower(0);
    }
    /**
     *
     * driveInDirection()
     * rotates robot and moves forward continuously
     *
     * @param speed speed (-1, 1) to rotate and go towards
     * @param robotAngle the current calculatedRobotAngle
     * @param rotation rotation in degrees
     */
    public void driveInDirection(double speed, double robotAngle, double rotation) {
        // speed is between 0 and 1
        final double v1 = speed * Math.cos(robotAngle) + rotation;
        final double v2 = speed * Math.sin(robotAngle) - rotation;
        final double v3 = speed * Math.sin(robotAngle) + rotation;
        final double v4 = speed * Math.cos(robotAngle) - rotation;

        front_left_motor.setPower(v1 * speed);
        front_right_motor.setPower(v2 * speed);
        rear_left_motor.setPower(v3 * speed);
        rear_right_motor.setPower(v4 * speed);
    }

    /**
     *
     * setHeading()
     * rotates to heading at speed .7
     *
     * @param heading heading to rotate to
     */
    public void setHeading(double heading) {
        double difHeading = internalHeading - heading;
        driveInDirection(.7, 0, difHeading);
    }

    /**
     * getHeading()
     *
     * @return the current heading of the robot
     */
    public double getHeading() {
        return internalHeading;
    }

    public boolean isTS() {
        op.telemetry.addData("Green: ", color.green());
        op.telemetry.addData("Blue: ", color.blue());
        op.telemetry.addData("Red: ", color.red());
        op.telemetry.addData("isTS: ", color.green() > 0.72*(color.blue() + color.red()));
        op.telemetry.update();
        return color.green() > 0.72*(color.blue() + color.red());
    }
    
    /**
     resetAngle()

     Sets the current angle to zero
     **/
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     resetAngle()

     Returns the current angle of the robot
     **/
    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle -= deltaAngle;

        lastAngles = angles;

        return globalAngle+startAngle;
    }

    /**
     * rotate()
     *
     * @param degrees : rotates by angle IN DEGREES
     */
    public void rotate(int degrees) {
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating clockwise (right) and - when rotating
        // counter-clockwise (left).

        if (degrees < 0) { // turn right.
            leftPower = -0.5;
            rightPower = 0.5;
        } else if (degrees > 0) { // turn left.
            leftPower = 0.5;
            rightPower = -0.5;
        } else
            return;

        // set power to rotate.
        front_left_motor.setPower(leftPower);
        front_right_motor.setPower(rightPower);

        // rotate until turn is completed.
        // turn the motors off.
        front_right_motor.setPower(0);
        front_left_motor.setPower(0);

        // wait for rotation to stop.


        // reset angle tracking on new heading.
        resetAngle();
    }

    /**
     *  setHeading()
     *
     * @param heading Heading in degrees of where to rotate
     * @param power Power between -1 and 1
     * @return TODO
     */
    public boolean setHeading(double heading, double power) {
        op.telemetry.addData("Phase", "setHeading");
        op.telemetry.update();
        boolean turned = false;
        power *= STANDARD_VOLTAGE/14;
        while(!op.isStopRequested()) {
            double error = heading - getAngle();
            //    op.telemetry.addData("Error: ", error);
            op.telemetry.addData("getAngle: ", getAngle());
            op.telemetry.update();
            if(Math.abs(error) < SET_HEADING_THRESHOLD) {
                for(DcMotor m:motorArray)
                    m.setPower(0);
                break;
            }
            for(int i = 0; i < 4; i++)
                motorArray[i].setPower(power*MOTOR_SIGNS[2][i]* -Math.signum(error));
            turned = true;
        }
        op.telemetry.addData("Turned", turned);
        op.telemetry.update();
        if(turned)
            op.sleep(wait);
        return turned;
    }

    /*
    ang =
    prim = distance in cm to drive forward/back
    lat = distance to drive laterally (+ = right and vice versa)
    power = -1 to 1 speed
    */

    /**
     * driveAtHeading()
     *
     * @param ang angle to drive at
     * @param prim distance in cm to drive forward/back
     * @param lat distance to drive laterally (+ = right and vice versa)
     * @param power speed (-1 to 1)
     */
    public void driveAtHeading(double ang, double prim, double lat, double power) {
        double primary = prim * driveCalibration;
        double lateral = lat * driveCalibration;
        double heading = ang * angleCalibration;
        // motorArray[1].setDirection(DcMotorSimple.Direction.REVERSE);
        // motorArray[3].setDirection(DcMotorSimple.Direction.REVERSE);
        setHeading(heading, power);
        //      op.telemetry.addData("Phase", "driveAtHeading");
        //      op.telemetry.addData("Driving: ", primary / 35 + "cm");
        //     op.telemetry.update();
        power *= STANDARD_VOLTAGE/getVoltage();

        double distance = Math.hypot(primary, lateral);
        int[] directions = new int[4];
        double[] targets = new double[4];
        boolean[] keyMotors = new boolean[4];
        for(int i = 0; i < 4; i++) {
            double delta = primary*MOTOR_SIGNS[0][i] + lateral*MOTOR_SIGNS[1][i];
            directions[i] = (int) Math.signum(delta);
            targets[i] = delta + motorArray[i].getCurrentPosition();
            keyMotors[i] = Math.abs(delta) > .5*distance;
        }
        drive:
        while(!op.isStopRequested()) {
            double error = getAngle()-heading;
            for(int i = 0; i < 4; i++){
                motorArray[i].setPower(power* (
                        MOTOR_SIGNS[0][i]*primary/distance +
                                MOTOR_SIGNS[1][i]*lateral/distance +
                                MOTOR_SIGNS[2][i]* -error/DEGREES_AT_FULL_POWER));
                // op.telemetry.addData("motorPower", motorArray[i].getPower());
            }

            for(int i = 0; i < 4; i++){
                op.telemetry.addData("Encoder Status " + i, motorArray[i].getCurrentPosition()*directions[i] );

                if(keyMotors[i] && motorArray[i].getCurrentPosition()*directions[i] > targets[i]*directions[i])
                    break drive;
            }
            //op.telemetry.update();
        }
        for(DcMotor m:motorArray)
            m.setPower(0);
        op.sleep(wait);
        for(int i = 0; i < 4; i++)
            op.telemetry.addData("Encoder Status " + i, (motorArray[i].getCurrentPosition()*directions[i] > targets[i]*directions[i]) ? "ok" : "BAD");
        op.telemetry.update();
    }

    /**
     * getVoltage()
     *
     * @return todo
     */
    double getVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : op.hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
    
    /**
     * setServoPercentClosed
     * 
     * Inputs: percentClosed (float value 0.0f - 1.0f)
     */
    public double setServoPercentClosed(float percentClosed) {
        this.bucket_thing.actuateToPercent(percentClosed);
        return this.bucket_thing.getPercentClosed();
    }
}
