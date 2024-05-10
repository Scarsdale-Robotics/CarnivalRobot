package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

@Config
public class InDepSubsystem extends SubsystemBase {
    private final Motor leftArm;
    private final Motor rightArm;

    private final Servo leftClaw;
    private final Servo rightClaw;
    private final ServoImplEx elbow;
    private final ServoImplEx wrist;

    private boolean isLeftClawOpen;
    private boolean isRightClawOpen;
    private boolean isElbowFlipped;
    private double delta;

    private final Telemetry telemetry;
    private final LinearOpMode opMode;

    public enum Level {
        GROUND(0, 0.58, false),
        BACKBOARD_HIGH(1261,0.55, true),
        BACKBOARD_MID(2150,0.58, true), // tuned values
        BACKBOARD_LOW(2330, 0.27, true);

        public final int target;
        public final double wristTarget;
        public final boolean elbowFlipped;

        Level(int target, double wristTarget, boolean elbowFlipped) {
            this.target = target;
            this.wristTarget = wristTarget;
            this.elbowFlipped = elbowFlipped;
        }

        public Level nextAbove() {
            if (this == GROUND) return BACKBOARD_HIGH;
            if (this == BACKBOARD_HIGH) return BACKBOARD_MID;
            if (this == BACKBOARD_MID) return BACKBOARD_LOW;
            return GROUND;
        }

        public Level nextBelow() {
            if (this == BACKBOARD_LOW) return BACKBOARD_MID;
            if (this == BACKBOARD_MID) return BACKBOARD_HIGH;
            if (this == BACKBOARD_HIGH) return GROUND;
            return BACKBOARD_LOW;
        }
    }
    public enum EndEffector {
        LEFT_CLAW_OPEN(0.8),
        LEFT_CLAW_CLOSED(0.55),
        RIGHT_CLAW_OPEN(0.17),
        RIGHT_CLAW_CLOSED(0.4),
        ELBOW_REST(0.763),
        ELBOW_FLIPPED(0.155);
        public final double servoPosition;

        EndEffector(double servoPosition) {
            this.servoPosition = servoPosition;
        }
    }
    public final int ELBOW_TURN_TICKS = 5038;

    public InDepSubsystem(Motor leftArm, Motor rightArm, ServoImplEx elbow, ServoImplEx wrist, Servo leftClaw, Servo rightClaw, LinearOpMode opMode) {
        this(leftArm, rightArm,elbow,wrist,leftClaw,rightClaw,opMode,null);
    }

    public InDepSubsystem(Motor leftArm, Motor rightArm, ServoImplEx elbow, ServoImplEx wrist, Servo leftClaw, Servo rightClaw, LinearOpMode opMode, Telemetry telemetry) {
        // initialize objects
        this.leftArm = leftArm;
        this.rightArm = rightArm;

        this.rightClaw = rightClaw;
        this.leftClaw = leftClaw;

        this.elbow = elbow;
        this.wrist = wrist;

        this.opMode = opMode;
        this.telemetry = telemetry;
        this.delta = 0;

        close();
    }

    /**
     * Calculates the power coefficient for the arm motors based on the arm position. (https://www.desmos.com/calculator/8qjoopxfda)
     */
    private double calculatePowerCoefficient(double armPos) {
        double relMax = 500;

        if (armPos <= relMax)
            return Math.max(2.0/3.0,
                    1 /
                            (Math.pow(
                                    (armPos-relMax) /
                                            (double)381,2) + 1)
            );
        else
            return Math.max(2.0/3.0,
                    1 /
                            (Math.pow(
                                    (armPos-relMax) /
                                            (double)1525,2) + 1)
            );
    }

    /**
     * Sets the adjusted powers of the arm motors.
     */
    public void rawPower(double power) {
        int armPos = getLeftArmPosition();

        double K_power = calculatePowerCoefficient(armPos-delta);

        // set bounds
        boolean underRange = armPos < delta && power < 0;
        boolean overRange = armPos > 2400 + delta && power > 0;
        if ((underRange || overRange) && !opMode.gamepad1.x) {
            leftArm.motor.setPower(0);
            rightArm.motor.setPower(0);
        } else {
            leftArm.motor.setPower(K_power * power);
            rightArm.motor.setPower(K_power * power);
        }

        setElbowPosition(calculateElbowPosition(armPos-delta));
        setWristPosition(calculateWristPosition(armPos-delta));

        opMode.telemetry.addData("elbow pos: ", elbow.getPosition());
        opMode.telemetry.addData("chicken: ", "nugget");
        opMode.telemetry.addData("elbowPos", elbow.getPosition());
        opMode.telemetry.addData("wristPos", wrist.getPosition());
    }

    /**
     * Sets the wrist servo to the passed position.
     */
    public void setWristPosition(double servoPosition) {
        wrist.setPosition(servoPosition);
    }

    /**
     * Stops the arm motors.
     */
    public void stopMotors() {
        leftArm.stopMotor();
        rightArm.stopMotor();
        leftArm.motor.setPower(0);
        rightArm.motor.setPower(0);
    }

    /**
     * Sets the left claw servo to the passed position.
     */
    public void setLeftClawPosition(double servoPosition) {
        leftClaw.setPosition(servoPosition);
    }

    /**
     * Sets the right claw servo to the passed position.
     */
    public void setRightClawPosition(double servoPosition) {
        rightClaw.setPosition(servoPosition);
    }

    /**
     * Opens both claws.
     */
    public void open() {
        openLeft();
        openRight();
    }

    /**
     * Closes both claws.
     */
    public void close() {
        closeLeft();
        closeRight();
    }

    /**
     * Opens the left claw.
     */
    public void openLeft() {
        leftClaw.setPosition(EndEffector.LEFT_CLAW_OPEN.servoPosition);
        isLeftClawOpen = true;
    }

    /**
     * Opens the right claw.
     */
    public void openRight() {
        rightClaw.setPosition(EndEffector.RIGHT_CLAW_OPEN.servoPosition);
        isRightClawOpen = true;
    }

    /**
     * Closes the left claw.
     */
    public void closeLeft() {
        leftClaw.setPosition(EndEffector.LEFT_CLAW_CLOSED.servoPosition);
        isLeftClawOpen = false;
    }

    /**
     * Closes the right claw.
     */
    public void closeRight() {
        rightClaw.setPosition(EndEffector.RIGHT_CLAW_CLOSED.servoPosition);
        isRightClawOpen = false;
    }

    /**
     * Calculates the elbow's position based on the arm's position. (https://www.desmos.com/calculator/8qjoopxfda)
     */
    private double calculateElbowPosition(double armPos) {
        double lowerBound = Level.BACKBOARD_HIGH.target-50;

        double eRest = EndEffector.ELBOW_REST.servoPosition;
        double eFlipped = EndEffector.ELBOW_FLIPPED.servoPosition;

        double m_elbow = (eFlipped - eRest) / ((double)Level.BACKBOARD_HIGH.target - lowerBound);

        if (armPos <= lowerBound)
            return eRest;
        else if (armPos <= (double)Level.BACKBOARD_HIGH.target)
            return m_elbow*armPos - m_elbow*500 + eRest;
        else
            return eFlipped;
    }

    /**
     * Calculates the wrist's position based on the arm's position
     */
    private double calculateWristPosition(double armPos) {
        double alpha = 40; // angle of wrist to the ground, constant

        // calculate equations for conversions between:
        //  - arm encoder ticks & arm angle in degrees
        //  - wrist angle in degrees & wrist servo position
        double[][] a = new double[][]{ // (ticks_a, theta_a)
                {0, 56},
                {384, 90},
                {2391, 270}
        };
        double[][] w = new double[][]{ // (theta_w, servoPosition_w)
                {116, 0.3},
                {180, 0.48},
                {270, 0.725}
        };
        int n = a.length;
        double[] m_a = new double[n];
        for (int i = 0; i < n; i++) {
            double[] curr = a[i];
            double[] next = a[(i+1) % n];
            m_a[i] = (curr[1] - next[1]) / (curr[0] - next[0]);
        }
        double[] w_a = new double[n];
        for (int i = 0; i < n; i++) {
            double[] curr = w[i];
            double[] next = w[(i+1) % n];
            w_a[i] = (curr[1] - next[1]) / (curr[0] - next[0]);
        }
        double m_a_avg = Arrays.stream(m_a).average().getAsDouble();
        double m_w_avg = Arrays.stream(w_a).average().getAsDouble();

        // calculate arm angle in degrees
        double theta_a = a[0][1] + m_a_avg * (armPos - a[0][0]);

        // calculate wrist angle in degrees
        double theta_w;
        double c = 0.0033; // independent variable coefficient to ground adjustment function
        if (theta_a < 180)
            theta_w = alpha + 270 - theta_a + Math.max(-alpha, -alpha * Math.exp(-armPos * c));
        else
            theta_w = alpha - 90 + theta_a;

        // calculate servo position
        double w_servoPosition = w[0][1] + m_w_avg * (theta_w - w[0][0]);

        return w_servoPosition;

    }

    /**
     * Sets the elbow servo to the passed position.
     */
    public void setElbowPosition(double servoPosition) {
        elbow.setPosition(servoPosition);
    }

    /**
     * Turns the elbow servo to its flip state.
     */
    public void flip() {
        elbow.setPosition(EndEffector.ELBOW_FLIPPED.servoPosition);
        isElbowFlipped = true;
    }

    /**
     * Turns the elbow servo to its non-flipped state.
     */
    public void rest() {
        elbow.setPosition(EndEffector.ELBOW_REST.servoPosition);
        isElbowFlipped = false;
    }

    /**
     * @return the position of the left arm motor, in ticks.
     */
    public int getLeftArmPosition() {
        return leftArm.motor.getCurrentPosition();
    }

    /**
     * @return the power of the left arm motor
     */
    public double getLeftArmVelocity() {
        return leftArm.getCorrectedVelocity();
    }

    /**
     * @return the position of the right arm motor, in ticks.
     */
    public int getRightArmPosition() {
        return rightArm.motor.getCurrentPosition();
    }

    /**
     * @return the power of the right arm motor
     */
    public double getRightArmVelocity() {
        return rightArm.getCorrectedVelocity();
    }

    /**
     * @return the position of the elbow servo
     */
    public double getElbowPosition() {
        return elbow.getPosition();
    }

    /**
     * @return the position of the wrist servo
     */
    public double getWristPosition() {
        return wrist.getPosition();
    }

    /**
     * @return the position of the left claw servo
     */
    public double getLeftClawPosition() {
        return leftClaw.getPosition();
    }

    /**
     * @return the position of the right claw servo
     */
    public double getRightClawPosition() {
        return rightClaw.getPosition();
    }

    /**
     * @return true if the left claw is open, otherwise false if it is closed.
     */
    public boolean getIsLeftClawOpen() {
        return isLeftClawOpen;
    }

    /**
     * @return true if the right claw is open, otherwise false if it is closed.
     */
    public boolean getIsRightClawOpen() {
        return isRightClawOpen;
    }

    /**
     * @return true if the elbow is flipped, otherwise false if it is rested.
     */
    public boolean getIsElbowFlipped() {
        return isElbowFlipped;
    }

    /**
     * Resets the encoder values of both arm motors.
     */
    public void resetArmEncoder() {
        delta = getLeftArmPosition();
    }

    public double getDelta() {
        return delta;
    }
}
