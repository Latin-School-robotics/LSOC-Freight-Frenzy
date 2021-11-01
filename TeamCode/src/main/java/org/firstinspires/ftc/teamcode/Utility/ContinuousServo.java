package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.Servo;

public class ContinuousServo {

    // Private Variables
    private Servo servo;


    //Constructors

    /**
     * ContinuousServo() - Constructor
     *
     * @param servoIn Servo object. Servo that will be spun
     */
    public ContinuousServo(Servo servoIn) {
        this.servo = servoIn;
    }

    //Functions

    /**
     * ContinuousServo.setSpeed()
     *
     * @param speed float: percentage between -1.0f and 1.0f
     * @throws Exception Thrown if speed is not between -1.0f and 1.0f
     */
    public void setSpeed(float speed) throws Exception {
        if (!(speed <=1.0f && speed >=-1.0f)){ throw new Exception("org.firstinspires.ftc.teamcode.Utility.ContinuousServo.setSpeed only accepts a float between 0.0f and 1.0f. Invalid parameter: " + speed); }

        servo.setPosition(speed);
    }

    /**
     * ContinuousServo.getSpeed()
     *
     * @return Returns current speed of serv
     */
    public float getSpeed(){
        return (float) servo.getPosition();
    }
}
