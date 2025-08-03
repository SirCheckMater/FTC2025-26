package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumDrivetrain {
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    //private Config config;
    private double max;
    private double max_Speed;
    public MecanumDrivetrain(double Max_Speed){
        //config = new Config();
        //back left
        //backLeft = hardwareMap.get(DcMotor.class, config.backLeft);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //front left
        //frontLeft = hardwareMap.get(DcMotor.class, config.frontLeft);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //back right
        //backRight = hardwareMap.get(DcMotor.class, config.backRight);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //front right
        //frontRight = hardwareMap.get(DcMotor.class, config.frontRight);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(Max_Speed > 1.0){
            Max_Speed = 1.0;
        }
        max_Speed = Max_Speed;
    }
    public void drive(double axial, double lateral, double yaw){
        double leftFrontPower  = axial + lateral + yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;
        double rightFrontPower = axial - lateral - yaw;
        //abs value
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        frontLeft.setPower(leftFrontPower* max_Speed);
        backLeft.setPower(leftBackPower*max_Speed);
        backRight.setPower(rightBackPower*max_Speed);
        frontRight.setPower(rightFrontPower*max_Speed);
    }
    public void modifyMaxSpeed(double newMultiplier){
        max_Speed = newMultiplier;

    }
}
