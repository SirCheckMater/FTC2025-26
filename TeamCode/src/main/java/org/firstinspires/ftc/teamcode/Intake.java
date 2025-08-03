package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    enum Slidestate{
        MAX, MIN
    }

    public Slidestate currentState;
    public void setState(Slidestate State){
        currentState.equals(State);
    }
    public void toMAXPos(){
        setState(Slidestate.MAX);
        left.setPosition(1);
        right.setPosition(1);
    }
    public void toMINPos(){
        setState(Slidestate.MIN);
        left.setPosition(0);
        right.setPosition(0);
    }
    HardwareMapping key = new HardwareMapping();
        public Servo right;
        public Servo left;

    public Intake() {
        right = hardwareMap.get(Servo.class, key.intakeServoR);
        left = hardwareMap.get(Servo.class, key.intakeServoL);
    }

    enum ArmState {
        PICKUP, INIT, TRANSFER,
    }


    /**
     * moves the slides to a specific position based on the input
     * @param pos is Centimeters
     * This function is meant for Axon servos
     */
    public void toSpecificPos(double pos){
        int stroke = 36;
        //degrees
        int rangeOfServo = 90;
        int cmPerDegree = 100/36;
        if(pos >= 36){
            left.setPosition(1.0);
            right.setPosition(1.0);
        }
        else if(pos == 0){
            left.setPosition(0);
            right.setPosition(0);
        }
        else{
            left.setPosition((pos*2.77778)/100);
        }
    }


}
