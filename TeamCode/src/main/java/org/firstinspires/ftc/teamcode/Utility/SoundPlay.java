package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.io.File;

public class SoundPlay {

        // Point to sound files on the phone's drive
        private File file;
        private OpMode op;

        public SoundPlay(String filepath, OpMode op) {
            this.file = new File(filepath);
            this.op = op;
        }

        public void playsound() throws Exception {
            if (this.file.exists()) {
                SoundPlayer.getInstance().startPlaying(op.hardwareMap.appContext, file);
                op.telemetry.addData("Playing", "Audio");
                op.telemetry.update();
            } else {
                throw new Exception("File not Found");
            }
        }
}
