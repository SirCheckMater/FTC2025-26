package org.firstinspires.ftc.teamcode.Sensors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Utilities.PDFL;
import org.firstinspires.ftc.teamcode.Utilities.Vector2d;


@Config
public class OdoPods {
    // SparkFun OTOS sensor instance
    private GoBildaPinpointDriver pinpoint;

    // PDFL controllers for heading, drive, and strafe
    private PDFL headingController;
    private PDFL driveController;
    private PDFL strafeController;

    // Deadzone threshold
    private double deadzone = .1;
    public static double headingdeadzone = .1;

    // Drivetrain reference for setting motor powers
    private MecanumDrivetrain drivetrain;

    public static double kph = 0;
    public static double kdh = 0;
    public static double kfh = 0;
    public static double klh = 0;

    public static double kpd = 0;
    public static double kdd = 0;
    public static double kfd = 0;
    public static double kld = 0;

    public static double kps = 0;
    public static double kds = 0;
    public static double kfs = 0;
    public static double kls = 0;

    //Contructor
    public OdoPods(HardwareMap hardwareMap, MecanumDrivetrain drivetrain) {

        //Initialize pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo_pods");
        pinpoint.setOffsets(114.25, 31.479);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.recalibrateIMU();

        // Initialize PDFL controllers with initial constants (these should be tuned(used values from summercamp code))
        headingController = new PDFL(kph, kdh, kfh, klh);
        driveController = new PDFL(kpd, kdd, kfd, kld);
        strafeController = new PDFL(kps, kds, kfs, kls);

        this.drivetrain = drivetrain;

        // Set deadzone for controllers
        headingController.setDeadzone(headingdeadzone);
        driveController.setDeadzone(deadzone);
        strafeController.setDeadzone(deadzone);
    }

    public double[] getPosition() {
        Pose2D pos = pinpoint.getPosition();
        return new double[]{pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES)};
    }

    public void setPosition(double x, double y, double heading) {
        Pose2D newPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
        pinpoint.setPosition(newPose);
    }

    public double getX() {
        return pinpoint.getPosition().getX(DistanceUnit.INCH);
    }


    public double getY() {
        return pinpoint.getPosition().getY(DistanceUnit.INCH);
    }

    public double getHeading() {
        return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }
    public double getVelocity(){
        return pinpoint.getHeadingVelocity();
    }

    public void update(){
        pinpoint.update();
    }

    public void recalibrateIMU() {
        pinpoint.recalibrateIMU();
    }

    public void resetPositionAndIMU() {
        pinpoint.resetPosAndIMU();
    }

    /**
     * Runs the PDFL controllers and computes the total correction.
     *
     * @param targetX        The target X position (in inches).
     * @param targetY        The target Y position (in inches).
     * @param targetHeading  The target heading (in degrees).
     * @param currentX       The current X position (in inches).
     * @param currentY       The current Y position (in inches).
     * @param currentHeading The current heading (in degrees).
     * @param speed          The overall speed factor (0 to 1).
     * @return The total correction to apply.
     */
    public double[] computeCorrections(double targetX, double targetY, double targetHeading,
                                       double currentX, double currentY, double currentHeading, double speed) {
        // Calculate position deltas using Vector2d
        Vector2d driveVector = new Vector2d(targetX - currentX, targetY - currentY);
        Vector2d rotatedVector = driveVector.rotateBy(-currentHeading);

        // Apply PDFL corrections
        double driveCorrection = driveController.run(rotatedVector.y) * speed;    // Error along Y-axis
        double strafeCorrection = strafeController.run(rotatedVector.x) * speed;  // Error along X-axis
        double headingCorrection = headingController.run(targetHeading - currentHeading) * speed; // Heading error

        return new double[]{driveCorrection, strafeCorrection, headingCorrection};
    }
    /**
     * Moves the robot to a target (x, y) position and target heading using PDFL corrections.
     *
     * @param targetX        The target X position (in inches).
     * @param targetY        The target Y position (in inches).
     * @param targetHeading  The target heading (in degrees).
     * @param currentX       The current X position (in inches).
     * @param currentY       The current Y position (in inches).
     * @param currentHeading The current heading (in degrees).
     * @param speed          The overall speed factor (0 to 1).
     */
    public void goToPosition(double targetX, double targetY, double targetHeading,
                             double currentX, double currentY, double currentHeading, double speed) {
        double[] corrections = computeCorrections(targetX, targetY, targetHeading,
                currentX, currentY, currentHeading, speed);
        double driveCorrection = corrections[0];
        double strafeCorrection = corrections[1];
        double headingCorrection = corrections[2];

        // Pass corrections to Drivetrain for execution
        drivetrain.drive(driveCorrection, strafeCorrection, headingCorrection);
    }

    /**
     * Holds the robot at a specific (x, y, heading) position using PDFL corrections.
     *
     * @param x       The target X position (in inches).
     * @param y       The target Y position (in in ches).
     * @param h       The target heading (in degrees).
     * @param speed   The overall speed factor (0 to 1).
     * @return True if the robot is within tolerance of the target, False otherwise.
     */
    public boolean holdPosition(double x, double y, double h, double speed) {
        double[] currentPos = getPosition();
        double currentX = currentPos[0];
        double currentY = currentPos[1];
        double currentHeading = currentPos[2];

        // Adjust PDFL control to move towards the target position
        goToPosition(x, y, h, currentX, currentY, currentHeading, speed);

        // Use a tighter tolerance for checking if the target is reached
        double positionTolerance = 2.5;  // Adjust this as needed for your robot's precision
        double headingTolerance = 2.5;    // Tolerance for heading in degrees

        boolean positionReached = Math.abs(currentX - x) < positionTolerance &&
                Math.abs(currentY - y) < positionTolerance &&
                Math.abs(currentHeading - h) < headingTolerance;
        return positionReached;
    }




}