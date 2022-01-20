package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Utility.ServoClaw;
import org.firstinspires.ftc.teamcode.OldCode.JacksonDriveTrain;

@Autonomous(name = "AviRST", group = "Autos")
public class AviRedTape extends LinearOpMode {
    JacksonDriveTrain dt;
    @Override
    public void runOpMode() {
        dt = new JacksonDriveTrain(this);
        waitForStart();
        dt.driveAtHeading(0, 0, -8, 0.7);
        dt.driveAtHeading(0, 16, 0, 0.7);
        // spins duck off carousel
        dt.spinCarousel(0.7);
        dt.driveAtHeading(0, 0, -50, 0.5);
        dt.driveAtHeading(0, 10, 0, 0.7);
        dt.driveAtHeading(0, 0, -40, 0.5);
        
    }
}
