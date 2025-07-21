package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

enum slideState{
       FLOOR, SKY
    }
    enum armState{

    }
public class Outtake {
    private double integralSum;
    private DcMotor slides;
    private double Kp;
    private double Ki;
    private double Kd;
    private double lastError;
    private ElapsedTime timer = new ElapsedTime();
    public void init(){
        slides = hardwareMap.get(DcMotor.class, "L");
        integralSum = 0;
        Kp = 0;
        Ki = 0;
        Kd = 0;
        lastError = 0;
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     *PID
     * @param reference is targetPosition
     */
    public void update(double reference){
       double error = reference - slides.getCurrentPosition();
       integralSum += error * timer.seconds();
       double derivitave = (error - lastError)/ timer.seconds();
       lastError = error;

       timer.reset();
       double output = (error*Kp)+(derivitave*Kd)+(integralSum*Ki);
       slides.setPower(output);
    }
}
