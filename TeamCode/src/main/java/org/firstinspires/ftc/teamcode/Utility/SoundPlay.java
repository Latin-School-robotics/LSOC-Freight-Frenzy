package org.firstinspires.ftc.teamcode.Utility;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.io.File;
public class SoundPlay {

        // Point to sound files on the phone's drive
        private File filepath;
        private OpMode op;

        public SoundPlay(File filepath, OpMode op) {
            this.filepath = filepath;
            this.op = op;
        }

        public void playsound() {
            if (this.filepath.exists()) {
                op.telemetry.addData("the crap exists,", "pog");
                op.telemetry.update();
                SoundPlayer.getInstance().startPlaying(op.hardwareMap.appContext, filepath);
            } else {
                op.telemetry.addData("Imbecile", "Can't find le file");
                op.telemetry.update();
            }
        }
}
