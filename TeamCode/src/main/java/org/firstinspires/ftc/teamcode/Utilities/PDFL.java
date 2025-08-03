package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PDFL {
    private double kP, kD, kF, kL;
    private double deadzone;
    private double homedConstant;
    private boolean homed = false;

    private ElapsedTime timer = new ElapsedTime();
    private RingBuffer<Double> timeBuffer = new RingBuffer<>(3, 0.0);
    private RingBuffer<Double> errorBuffer = new RingBuffer<>(3, 0.0);

    /**
     * Constructor to initialize the PDFL controller with specific constants.
     *
     * @param kP Proportional gain.
     * @param kD Derivative gain.
     * @param kF Feedforward gain.
     * @param kL Lower limit gain.
     */
    public PDFL(double kP, double kD, double kF, double kL){
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
        timer.reset();
    }

    /**
     * Updates the PIDF constants.
     *
     * @param kP Proportional gain.
     * @param kD Derivative gain.
     * @param kF Feedforward gain.
     * @param kL Lower limit gain.
     */
    public void updateConstants(double kP, double kD, double kF, double kL){
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
    }

    /**
     * Sets the deadzone threshold.
     *
     * @param deadzone The deadzone value below which corrections aren't applied.
     */
    public void setDeadzone(double deadzone){
        this.deadzone = deadzone;
    }

    /**
     * Sets the homed state.
     *
     * @param homed True if homed, False otherwise.
     */
    public void setHomed(boolean homed){
        this.homed = homed;
    }

    /**
     * Sets the constant value to return when homed.
     *
     * @param constant The homed constant.
     */
    public void setHomedConstant(double constant){
        homedConstant = constant;
    }

    /**
     * Resets the controller's buffers and timer.
     */
    public void reset(){
        timeBuffer.fill(0.0);
        errorBuffer.fill(0.0);
        timer.reset();
    }

    public void setkP(double value){
        this.kP = value;
    }
    /**
     * Runs the PDFL controller with the given error.
     *
     * @param error The current error.
     * @return The control response.
     */
    public double run(double error){
        if (homed){
            return homedConstant;
        }

        double currentTime = timer.seconds() * 1000; // Convert to milliseconds
        double previous_time = timeBuffer.getValue(currentTime);
        double previous_error = errorBuffer.getValue(error);

        double delta_time = currentTime - previous_time; // in milliseconds
        double delta_error = error - previous_error;

        // If the PDFL hasn't been updated within 200 ms, reset it
        if (delta_time > 200){
            reset();
            return run(error);
        }

        double p = pComponent(error);
        double d = dComponent(delta_error, delta_time);
        double f = fComponent();
        double l = lComponent(error);

        double response = p + d + f + l;

        if (Math.abs(error) < deadzone){
            // Apply only P, D, and F without L component
            response = p + d + f;
        }

        return response;
    }

    private double pComponent(double error){
        return kP * error;
    }

    private double dComponent(double delta_error, double delta_time){
        if (delta_time <= 0) return 0.0; // Prevent division by zero
        double derivative = delta_error / delta_time;
        return derivative * kD;
    }

    private double fComponent(){
        return kF;
    }

    private double lComponent(double error){
        double direction = Math.signum(error);
        return direction * kL;
    }
}
