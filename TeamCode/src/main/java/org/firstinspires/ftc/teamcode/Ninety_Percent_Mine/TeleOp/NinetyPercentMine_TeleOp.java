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

        //telemetry b
        telemetry.addData("Ready for launch!" , "＼(≧▽≦)／");
        telemetry.addData("WARNING!" , "LINEAR SLIDE IS OPERATING IN UNRESTRICTED MODE");
        telemetry.update();
    }

    //loop function
    @Override
    public void loop() {

        //calculates slowdown modifiers
        float slowDownModP1 = 1 - 0.85f * gamepad2.left_trigger;
        float slowDownModP2 = 1 - 0.85f * gamepad2.right_trigger;

        //calculates arm speed
        float armSpeed = ((gamepad2.dpad_down ? 0.5f : 0) + (gamepad2.dpad_up ? -0.5f : 0)) * this.slowDownModP1;

        //calculates claw speed
        float clawSpeed = ((gamepad2.y ? 0.5f : 0) + (gamepad2.a ? -0.5f : 0)) * this.slowDownModP1;

        //moves arm and claw
        armController.MoveSlideUnrestricted(armSpeed);
        clawController.MoveSlideUnrestricted(clawSpeed);

        //calculates driving variables
        this.forwardDrive = -gamepad2.right_stick_y * this.slowDownModP2;
        this.panDrive = gamepad2.right_stick_x * this.slowDownModP2;
        this.rotation = gamepad2.left_stick_x * this.slowDownModP2;

        //moves wheels
        this.dt.travel(this.forwardDrive, this.panDrive, this.rotation);

        //telemetry
        telemetry.addData("Driving...", "");
        telemetry.addData("Forward Drive", forwardDrive);
        telemetry.addData("Pan Drive", panDrive);
        telemetry.addData("Rotation", rotation);
        telemetry.addData("Arm", armSpeed);
        telemetry.addData("Claw", clawSpeed);

        telemetry.update();
    }
}
