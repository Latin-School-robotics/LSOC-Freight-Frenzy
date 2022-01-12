package org.firstinspires.ftc.teamcode.dripmobile.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Utility.BasicOpTrain;
import org.firstinspires.ftc.teamcode.Utility.LinearSlide;
import org.firstinspires.ftc.teamcode.Utility.ServoClaw;


@TeleOp(name="drip_or_drown", group="TeleOp")
public class drip_or_drown extends OpMode {

    //Variables
    BasicOpTrain dt;

    DcMotor slide;
    DcMotor spinner;
    DcMotor estar;
    DcMotor flipper;
    ServoClaw bucket;


    float sdm_movp1;
    float sdm_bktp1;
    float sdm_bktp2;
    float sdm_strp2;
    float sdm_lftp2;

    float forwardDrive;
    float panDrive;
    float rotation;

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
        back_left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        slide = hardwareMap.dcMotor.get("slide");
        spinner = hardwareMap.dcMotor.get("spinner");
        estar = hardwareMap.dcMotor.get("stars");
        flipper = hardwareMap.dcMotor.get("flipper");
        flipper.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo bucket_servo = hardwareMap.servo.get("bucket");

        //Init Code
        this.dt = new BasicOpTrain(front_left_drive, front_right_drive, back_left_drive, back_right_drive);
        // this.elevator = new LinearSlide(elevator_motor, 0,360);

        //IMPORTANT: Declare claw range constraints below
        this.bucket = new ServoClaw(bucket_servo, 0f, .75f);
        //flipper.actuateToPercent(0f);

        //Telemetry B
        telemetry.addData("Ready for launch!" , "＼(≧▽≦)／");
        telemetry.addData("WARNING!" , "LINEAR SLIDE IS OPERATING IN UNRESTRICTED MODE");
        telemetry.update();
    }

    @Override
    public void loop() {
        //Get Gamepad Vars

        this.sdm_movp1 = (gamepad1.left_bumper ? 0.5f : 1);
        this.sdm_bktp1 = (gamepad1.right_bumper ? 0.5f : 1);
        this.sdm_lftp2 = (gamepad2.left_bumper ? 0.5f : 1);
        this.sdm_bktp2 = (gamepad2.right_bumper ? 0.5f : 1);

        this.forwardDrive = - gamepad1.left_stick_y * this.sdm_movp1;
        //this.forwardDrive = - gamepad1.right_stick_y * this.sdm_movp1;
        if(gamepad1.dpad_left) {
            this.panDrive = 1f* this.sdm_movp1;
        }
        else if(gamepad1.dpad_right) {
            this.panDrive = -1f* this.sdm_movp1;
        }
        else if(gamepad1.dpad_up) {
            this.panDrive = 1f* this.sdm_movp1;
        }
        else if(gamepad1.dpad_down) {
            this.panDrive = -1f* this.sdm_movp1;
        }
        else {
            this.panDrive = 0f* this.sdm_movp1;
        }

        this.panDrive = -gamepad1.right_stick_x * this.sdm_movp1;
        // this.panDrive = (gamepad1.dpad_left ? 1f : 0) * this.sdm_movp1;
        // this.panDrive = (gamepad1.dpad_right ? 1f : 0) * this.sdm_movp1;
        this.rotation = -gamepad1.left_stick_x * this.sdm_movp1;


        //Manage Driving

        dt.travel(this.forwardDrive, this.panDrive, this.rotation);

        //Manage Linear Slide

        slide.setPower(-gamepad2.left_stick_y * this.sdm_lftp2);

        spinner.setPower((gamepad1.left_bumper ? -1 : 1) * gamepad1.left_trigger * .5f);
        estar.setPower((gamepad1.right_bumper ? -1 : 1) * gamepad1.right_trigger);
        estar.setPower(-gamepad2.right_stick_y * this.sdm_bktp2);
        estar.setPower(gamepad2.right_trigger * (gamepad2.right_bumper ? -1 : 1));

        //Manage Claw

        bucket.actuateToPercent(1 - gamepad2.left_trigger);
        flipper.setPower(-gamepad2.right_stick_y * 0.5f);

        //Telemetry
        telemetry.addData("Driving...", "");
        telemetry.addData("Forward Drive", forwardDrive);
        telemetry.addData("Pan Drive", panDrive);
        telemetry.addData("Rotation", rotation);
        telemetry.update();
    }
}