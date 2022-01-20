package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.OldCode.JacksonDriveTrain;
@Autonomous(name = "AAR_v2", group = "Autos")
public class AAR_v2 extends LinearOpMode{
    JacksonDriveTrain dt;
    @Override
    public void runOpMode() {
        dt = new JacksonDriveTrain(this);
        waitForStart();
        // dt.driveAtHeading(0, 0, -10, 0.7);
        // dt.driveAtHeading(0, -10, 0, 0.7);
        // dt.spinCarousel(-0.8);
        // dt.driveAtHeading(0, 5, 0, 0.7);
        // dt.driveAtHeading(0, 0, -51, 0.7);
        // dt.moveSlide(1, 2);
        // dt.longBTilt(2);
        while(opModeIsActive()) {
            dt.isTS();
        }
    }
    
}
