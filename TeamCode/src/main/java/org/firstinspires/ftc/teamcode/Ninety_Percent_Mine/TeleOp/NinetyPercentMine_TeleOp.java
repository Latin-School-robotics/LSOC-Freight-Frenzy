package org.firstinspires.ftc.teamcode.Ninety_Percent_Mine.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utility.BasicOpTrain;
import org.firstinspires.ftc.teamcode.Utility.LinearSlide;


@TeleOp(name="Ninety TeleOp", group="TeleOp")
public class NinetyPercentMine_TeleOp extends OpMode {

    BasicOpTrain dt;

    float slowDownModP1;
    float slowDownModP2;

    //creating linear slide objects
    LinearSlide armController;
    LinearSlide clawController;
    LinearSlide spinnerController;

    float forwardDrive;
    float panDrive;
    float rotation;

    //init function
    @Override
    public void init() {
        //telemetry a
        telemetry.addData("Hello! Initializing!", "웃");
        telemetry.update();

        //get motors from hardware map
        DcMotor arm = hardwareMap.dcMotor.get("arm");
        DcMotor claw = hardwareMap.dcMotor.get("claw");
        DcMotor spinner = hardwareMap.dcMotor.get("spinner");

        DcMotor front_left_drive = hardwareMap.dcMotor.get("front left drive");
        DcMotor front_right_drive = hardwareMap.dcMotor.get("front right drive");
        DcMotor back_left_drive = hardwareMap.dcMotor.get("back left drive");
        DcMotor back_right_drive = hardwareMap.dcMotor.get("back right drive");

        front_left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left_drive.setDirection(DcMotorSimple.Direction.REVERSE);

        this.dt = new BasicOpTrain(front_left_drive, front_right_drive, back_left_drive, back_right_drive);

        //declare linear slide controllers
        armController = new LinearSlide(arm, 0,360);
        clawController = new LinearSlide(claw, 0,360);
        spinnerController = new LinearSlide(spinner, 0,360);


        //telemetry b
        telemetry.addData("Ready for launch!" , "＼(≧▽≦)／");
        telemetry.addData("WARNING!" , "LINEAR SLIDE IS OPERATING IN UNRESTRICTED MODE");
        telemetry.update();
    }

    //loop function
    @Override
    public void loop() {

        //calculates slowdown modifiers
        this.slowDownModP1 = 1 - 0.85f * gamepad1.right_trigger;
        // float slowDownModP2 = 1 - 0.85f * gamepad1.left_trigger;
        // float slowDownModP1 = 1f;
        this.slowDownModP2 = 1f;

        //calculates arm speed
        float armSpeed = ((gamepad1.dpad_down ? 0.5f : 0) + (gamepad1.dpad_up ? -0.5f : 0)) * this.slowDownModP1;

        //calculates claw speed
        float clawSpeed = ((gamepad1.b ? 0.5f : 0) + (gamepad1.a ? -0.5f : 0)) * this.slowDownModP1;

        //calculates spinner speed
        float spinnerSpeed = (gamepad1.y ? -1f : 0) * this.slowDownModP1;


        //moves arm and claw
        armController.MoveSlideUnrestricted(armSpeed * slowDownModP1);
        clawController.MoveSlideUnrestricted(clawSpeed * slowDownModP1);
        spinnerController.MoveSlideUnrestricted(spinnerSpeed * slowDownModP1);


        //calculates driving variables
        this.forwardDrive = -gamepad1.left_stick_y * slowDownModP1;
        this.panDrive = gamepad1.left_stick_x * slowDownModP1;
        this.rotation = gamepad1.right_stick_x * slowDownModP1;

        //moves wheels
        this.dt.travel(this.forwardDrive, this.panDrive, this.rotation);

        //telemetry
        telemetry.addData("Driving...", "");
        telemetry.addData("Forward Drive", forwardDrive);
        telemetry.addData("Pan Drive", panDrive);
        telemetry.addData("Rotation", rotation);
        telemetry.addData("Arm", armSpeed);
        telemetry.addData("Claw", clawSpeed);
        telemetry.addData("Slowdown Mod", this.slowDownModP1);

        telemetry.update();
    }
}
