package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.OldCode.JacksonDriveTrain;
@Autonomous(name = "ArmTest", group = "Autos")
public class ArmTest extends LinearOpMode{
    JacksonDriveTrain dt;
    @Override
    public void runOpMode() {
        waitForStart();
        dt = new JacksonDriveTrain(this);
        dt.initArm();
        dt.setArm(0);
    }
    
}
