package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.OldCode.JacksonDriveTrain;
@Autonomous(name = "AviRed", group = "Autos")
public class AviAutoRed extends LinearOpMode{
    JacksonDriveTrain dt;
    @Override
    public void runOpMode() {
        dt = new JacksonDriveTrain(this);
        waitForStart();
        // drives to carousel
        dt.driveAtHeading(0, 0, -8, 0.7);
        dt.driveAtHeading(0, 16, 0, 0.7);
        dt.initBucket();
        // spins duck off carousel
        dt.spinCarousel(0.7);
        // turns and drives to barcode
        dt.driveAtHeading(0, 0, -8, 0.7);
        dt.driveAtHeading(180, 0, 50, 0.6);
        dt.driveAtHeading(180, 21, 0, 0.7);
        // starts scanning barcode
        int i = 2;
        while(!dt.isTS() && opModeIsActive() && i > -1) {
             dt.driveAtHeading(180, 21, 0, 0.7);
             i--;
        }
        if(i < 0) {
            int ts_pos = 0;
            for(int j = 0; j < 0; j++) {
                dt.driveAtHeading(180, 21, 0, 0.7);
            }
            dt.driveAtHeading(180, 45, 0, 0.6);
            dt.driveAtHeading(180, 0, 27, 0.5);
            dt.dumpBucket(ts_pos, 180);
            sleep(300);
            dt.driveAtHeading(180, 0, -70, 1);
            dt.driveAtHeading(180, 180, 0, 1);
        }
        else {
            int ts_pos = i;
            for(int j = 0; j < i; j++) {
                dt.driveAtHeading(180, 21, 0, 0.7);
            }
            dt.driveAtHeading(180, 43, 0, 0.6);
            dt.driveAtHeading(180, 0, 27, 0.5);
            dt.dumpBucket(ts_pos, 180);
            sleep(300);
            dt.driveAtHeading(180, 0, -70, 1);
            dt.driveAtHeading(180, 180, 0, 1);
        }
        }
    
}
