package org.firstinspires.ftc.teamcode.Ninety_Percent_Mine.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OldCode.JacksonDriveTrain;

@Autonomous(name = "NinetyAuto", group = "Autos")
public class NinetyPercentMine_Autonomous extends LinearOpMode {
    JacksonDriveTrain dt;
    @Override
    public void runOpMode() {
        waitForStart();
        dt = new JacksonDriveTrain(this);
        dt.driveAtHeading(0, 0, 1.5, 0.4);
        dt.driveAtHeading(127, 0, 30, 0.7);
        dt.driveAtHeading(127, 22, 0, 0.7);
        dt.driveAtHeading(127, 0, 32, 0.6);
        dt.driveAtHeading(127, 5, 0, 0.7);
        dt.driveAtHeading(127, -5, 0, 0.7);
        dt.driveAtHeading(37, 50, 0, 1);
        dt.driveAtHeading(37, 0, -20, 0.6);
        dt.driveAtHeading(37, 100, 0, 1);
    }
}
