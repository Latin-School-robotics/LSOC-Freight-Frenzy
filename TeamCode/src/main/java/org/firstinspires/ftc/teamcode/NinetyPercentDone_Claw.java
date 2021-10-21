package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Ninety Claw", group="TeleOp")

public class NinetyPercentDone_Claw extends OpMode {

    LinearSlide armController;
    LinearSlide clawController;


    @Override
    public void init() {
        telemetry.addData("Hello! Initializing!", "웃");
        telemetry.update();

        DcMotor arm = hardwareMap.dcMotor.get("arm");
        DcMotor claw = hardwareMap.dcMotor.get("claw");

        armController = new LinearSlide(arm, 0,360);
        clawController = new LinearSlide(claw, 0,360);

        telemetry.addData("Ready for launch!" , "＼(≧▽≦)／");
        telemetry.addData("WARNING!" , "LINEAR SLIDE IS OPERATING IN UNRESTRICTED MODE");
        telemetry.update();
    }

    @Override
    public void loop() {

        armController.MoveSlideUnrestricted(gamepad2.left_stick_y);
        clawController.MoveSlideUnrestricted(gamepad2.left_trigger);


    }
}
