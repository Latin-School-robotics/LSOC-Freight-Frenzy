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


public class JacksonDriveTrain{

    /**

    Variables

     **/

    DcMotor front_left_motor, front_right_motor, rear_left_motor, rear_right_motor;
    DcMotor arm;
    DcMotor spinner;
    // DcMotor rope;
    // Servo grabL, grabR;
    Servo claw;
    double internalHeading;
    DcMotor[] motorArray = new DcMotor[4];
    ColorSensor color;
    VoltageSensor[] voltageSensors;
    //Servo spinner;
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
    final static double angleCalibration = 0.92*90/83;
    final static double SET_HEADING_THRESHOLD = 5;

    public void calibrate() {
        //if(imu.)
        op.idle();
    }

    /**

    Constructors

     **/

    public JacksonDriveTrain(LinearOpMode op) {
        this.op = op;

        // grabL = op.hardwareMap.servo.get("hand servo left");
        // grabR = op.hardwareMap.servo.get("hand servo right");
        // claw = op.hardwareMap.servo.get("claw");
        //spinner = op.hardwareMap.servo.get("spinner");
        front_left_motor = op.hardwareMap.dcMotor.get("front left drive");
        front_right_motor = op.hardwareMap.dcMotor.get("front right drive");
        rear_left_motor = op.hardwareMap.dcMotor.get("back left drive");
        rear_right_motor = op.hardwareMap.dcMotor.get("back right drive");
        arm = op.hardwareMap.dcMotor.get("arm");
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
        
        spinner = op.hardwareMap.dcMotor.get("spinner");
        spinner.setDirection(DcMotorSimple.Direction.REVERSE);
        
        claw = op.hardwareMap.servo.get("claw");
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
    }

    /**

    Functions

     **/

    public DcMotor[] getMotors() {
        return motorArray;
    }
    public void spinCarousel() {
        spinner.setPower(0.7);
        op.sleep(2400);
        spinner.setPower(0);
    }
    public void grab() {
        
    }
    // flip the arm backwards
    public void initArm() {
        // arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // arm.setTargetPosition(800);
        // arm.setPower(0.4);
        // if
        // arm.setTargetPosition(arm.getCurrentPosition() + 800);
        // arm.setPower(0.4);
        arm.setPower(0.4);
        op.sleep(4700);
        arm.setPower(0);
        op.sleep(500);
        arm.setPower(-0.4);
        op.sleep(800);
        arm.setPower(-0.3);
        op.sleep(4500);
        arm.setPower(0);
        op.sleep(500);
    }
    public void setArm(int pos) {
        // highest level = 0
        // mid = 1
        // lowest = 2
        arm.setPower(-0.2);
        op.sleep(440*pos);
        arm.setPower(0);
        op.sleep(1000);
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
        op.telemetry.addData("isTS: ", color.green() > 0.6*(color.blue() + color.red()));
        //op.telemetry.update();
        return color.green() > 0.6*(color.blue() + color.red());
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

        globalAngle += deltaAngle;

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
            double error = getAngle()-heading;
            //    op.telemetry.addData("Error: ", error);
            //    op.telemetry.addData("getAngle: ", getAngle());
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
        //     op.telemetry.addData("Turned", turned);
//        op.telemetry.update();
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
//https://www.quora.com/Why-do-we-use-a-colon-in-Java
// drive is just a label for the loop, because we want to know what loop to break out of
// that's why we do break drive;
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
                //op.telemetry.addData("Encoder Status " + i, motorArray[i].getCurrentPosition()*directions[i] );

                if(keyMotors[i] && motorArray[i].getCurrentPosition()*directions[i] > targets[i]*directions[i])
                    break drive;
            }
            //op.telemetry.update();
        }
        for(DcMotor m:motorArray)
            m.setPower(0);
        op.sleep(wait);
        //    for(int i = 0; i < 4; i++)
        //             op.telemetry.addData("Encoder Status " + i, (motorArray[i].getCurrentPosition()*directions[i] > targets[i]*directions[i]) ? "ok" : "BAD");
        //      op.telemetry.update();
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
}
