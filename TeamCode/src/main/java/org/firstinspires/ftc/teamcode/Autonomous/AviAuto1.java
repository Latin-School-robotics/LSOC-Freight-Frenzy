package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.JacksonDriveTrain;

@Autonomous(name = "AviAuto1", group = "Autos")
public class AviAuto1 extends LinearOpMode{
    JacksonDriveTrain dt;
    @Override
    public void runOpMode() {
        waitForStart();
        dt = new JacksonDriveTrain(this);
        dt.driveAtHeading(0, 0, 1.5, 0.4);
        // spinning
        dt.spinCarousel();
        // scan barcode
        dt.driveAtHeading(127, 0, 30, 0.7);
        dt.driveAtHeading(127, 22, 0, 0.7);
        // color sensor range of 5cm
        int i = 2;
        while(!dt.isTS() && opModeIsActive()) {
             dt.driveAtHeading(127, 0, 23.2, 0.6);
             i--;
        }
        // getting to shipping hub
        int ts_pos = i;
        for(int j = 0; j < i; j++) {
            dt.driveAtHeading(127, 0, 23.2, 0.6);
        }
        dt.driveAtHeading(127, 0, 32, 0.6);
        dt.driveAtHeading(127, 5, 0, 0.7);
        dt.driveAtHeading(127, -5, 0, 0.7);
        dt.driveAtHeading(37, 50, 0, 1);
        dt.driveAtHeading(37, 0, -20, 0.6);
        dt.driveAtHeading(37, 100, 0, 1);
    }
}
