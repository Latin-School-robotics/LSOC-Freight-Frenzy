package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Utility.ServoClaw;
import org.firstinspires.ftc.teamcode.OldCode.JacksonDriveTrain;

@Autonomous(name = "AviRSL", group = "Autos")
public class AviRedSlide extends LinearOpMode {
    JacksonDriveTrain dt;
    @Override
    public void runOpMode() {
        dt = new JacksonDriveTrain(this);
        waitForStart();
        // drives to carousel
        dt.driveAtHeading(0, 120, 0, 0.8);
        
    }
}