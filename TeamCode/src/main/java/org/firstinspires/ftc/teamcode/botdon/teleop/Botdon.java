package org.firstinspires.ftc.teamcode.botdon.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Utility.BasicOpTrain;
import org.firstinspires.ftc.teamcode.Utility.LinearSlide;
import org.firstinspires.ftc.teamcode.Utility.ServoClaw;


@TeleOp(name="TeleOp Drive", group="TeleOp")
public class Botdon extends OpMode {

    //Variables
    BasicOpTrain dt;
    
    DcMotor arm;
    DcMotor spinner;
    ServoClaw clawservo;

    float slowdownModifierp1;
    float slowdownModifierp2;

    float forwardDrive;
    float panDrive;
    float rotation;
    float elevatorSpeed;

    @Override
    public void init() {
        //Telemetry A
        telemetry.addData("Hello! Initializing!", "＼(⌒▽⌒)");
        telemetry.update();

        //Get Motors
        DcMotor front_left_drive = hardwareMap.dcMotor.get("front left drive");
        DcMotor front_right_drive = hardwareMap.dcMotor.get("front right drive");
        DcMotor back_left_drive = hardwareMap.dcMotor.get("back left drive");
        DcMotor back_right_drive = hardwareMap.dcMotor.get("back right drive");
        front_left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        arm = hardwareMap.dcMotor.get("arm");
        spinner = hardwareMap.dcMotor.get("spinner");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo claw_servo = hardwareMap.servo.get("claw");

        //Init Code
        this.dt = new BasicOpTrain(front_left_drive, front_right_drive, back_left_drive, back_right_drive);
        // this.elevator = new LinearSlide(elevator_motor, 0,360);

        //IMPORTANT: Declare claw range constraints below
        this.clawservo = new ServoClaw(claw_servo, 0f, .5f);

        //Telemetry B
        telemetry.addData("Ready for launch!" , "＼(≧▽≦)／");
        telemetry.addData("WARNING!" , "LINEAR SLIDE IS OPERATING IN UNRESTRICTED MODE");
        telemetry.update();
    }

    @Override
    public void loop() {
        //Get Gamepad Vars

        this.slowdownModifierp1 = 1 - (gamepad1.right_trigger * 0.85f);
        this.slowdownModifierp2 = 1 - (gamepad2.left_trigger * 0.85f);

        this.forwardDrive = - gamepad1.right_stick_y * this.slowdownModifierp1;

        this.panDrive = gamepad1.right_stick_x * this.slowdownModifierp1;
        this.rotation = gamepad1.left_stick_x * this.slowdownModifierp1;

        // this.elevatorSpeed = gamepad2.right_stick_y * this.slowdownModifierp2;

        //Manage Driving

        dt.travel(this.forwardDrive, this.panDrive, this.rotation);

        //Manage Linear Slide

        arm.setPower(gamepad2.left_stick_y * this.slowdownModifierp2);
        
        spinner.setPower(gamepad2.right_bumper ? -1 : 0);
        //Manage Claw

        clawservo.actuateToPercent(1 - gamepad2.right_trigger);

        //Telemetry

        telemetry.addData("Driving...", "");
        telemetry.addData("Forward Drive", forwardDrive);
        telemetry.addData("Pan Drive", panDrive);
        telemetry.addData("Rotation", rotation);
        telemetry.update();
    }
}