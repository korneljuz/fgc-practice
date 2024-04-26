package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    // reference = setpoint
    // error = difference between reference and
    ElapsedTime timer = new ElapsedTime();
    double kP, kI, kD = 0;
    double encoderPosition = 0;
    double reference, lastReference, error, lastError, integralSum, derivative = 0;
    double armOutput = 0;

    public PIDController(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    public void setGoal(double goal) {
        this.reference = goal;
    }

    public double calculateOutput(double encoderPosition) {
        // obtain the encoder position
        this.encoderPosition = encoderPosition;
        // calculate the error
        error = reference - encoderPosition;

        // rate of change of the error
        derivative = (error - lastError) / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        // reset integral sum upon setpoint/reference changes
        if (reference != lastReference) {
            integralSum = 0;
        }

        armOutput = (kP * error) + (kI * integralSum) + (kD * derivative);

        lastError = error;
        lastReference = reference;
        // reset the timer for next time
        timer.reset();

        return armOutput;
    }

}