package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Utility.ServoClaw;
import org.firstinspires.ftc.teamcode.OldCode.JacksonDriveTrain;

@Autonomous(name = "AviBST", group = "Autos")
public class AviBlueTape extends LinearOpMode {
    JacksonDriveTrain dt;
    @Override
    public void runOpMode() {
        dt = new JacksonDriveTrain(this);
        waitForStart();
        // drives to carousel
        dt.driveAtHeading(0, -8, 0, 0.7);
        dt.driveAtHeading(0, 0, 10, 0.7);
        // spins duck off carousel
        dt.spinCarousel(-0.7);
        dt.driveAtHeading(0, -60, 0, 0.7);
        dt.driveAtHeading(0, 0, 25, 0.5);
        
    }
}