package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    enum state{
        MAX, MIN
    }
    public state currentState;
    public void setState(state State){
        currentState.equals(State);
    }
    HardwareMapping key = new HardwareMapping();
        public Servo right;
        public Servo left;
    public Intake() {
        right = hardwareMap.get(Servo.class, key.intakeServoR);
        left = hardwareMap.get(Servo.class, key.intakeServoL);
    }
    public void toMAXPos(){
        setState(state.MAX);
        left.setPosition(1);
        right.setPosition(1);
    }
    public void toMINPos(){
        setState(state.MIN);
        left.setPosition(0);
        right.setPosition(0);
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
