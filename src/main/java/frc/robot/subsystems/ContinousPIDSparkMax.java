package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpiutil.math.MathUtil;

public class ContinousPIDSparkMax {
    /**
     * Takes the desired principle angle (within -PI to +PI) and the current angle not contraint to a range
     * and calculates which direction around the circle is faster to get to desired angle.
     * 
     * @param desiredPrincipleAngle An angle within -PI to +PI like the output from kinematics
     * @param currentAngle The current angle measured by the encoder.
     * @return The setpoint which is closer to the desiredAngle.
     */
    public static Rotation2d calculate(Rotation2d desiredAngle, Rotation2d currentAngle) {
        // Ensure the desired angle is between -pi to pi
        desiredAngle = new Rotation2d(MathUtil.angleModulus(desiredAngle.getRadians()));

        // Create a new currentAngle that is within -pi to pi
        Rotation2d currentModulatedAngle = new Rotation2d(MathUtil.angleModulus(currentAngle.getRadians()));

        double shortestError = MathUtil.inputModulus(desiredAngle.getRadians() - currentModulatedAngle.getRadians(), -Math.PI, Math.PI);
        
        Rotation2d setpoint = new Rotation2d(currentAngle.getRadians() + shortestError);

        return setpoint;
    }
}
