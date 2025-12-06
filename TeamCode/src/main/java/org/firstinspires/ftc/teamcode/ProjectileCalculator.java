package org.firstinspires.ftc.teamcode;

public class ProjectileCalculator {

    // Acceleration due to gravity (m/s^2)
    private static final double GRAVITY = 9.81;

    /**
     * Calculates the necessary launch angle (in radians) to reach a specific horizontal distance and vertical height.
     *
     * @param initialVelocity The initial speed of the projectile (m/s).
     * @param distanceX The horizontal distance to the target (m).
     * @param distanceY The vertical height to the target (m), relative to launch height.
     * @return The launch angle in radians, or NaN if the target is unreachable.
     */
    public static double calculateLaunchAngle(double initialVelocity, double distanceX, double distanceY) {
        double vSquared = initialVelocity * initialVelocity;
        double vQuad = Math.pow(initialVelocity, 4);
        double gravitySquared = GRAVITY * GRAVITY;
        double xSquared = distanceX * distanceX;

        // The term inside the square root (part of the quadratic formula solution)
        double operandB = GRAVITY * (gravitySquared * xSquared + 2 * distanceY * vSquared);

        // Check if the target is unreachable (discriminant is negative)
        if (operandB > vQuad) {
            return Double.NaN; // Cannot reach target with given speed
        }

        double root = Math.sqrt(vQuad - operandB);
        double angle1 = Math.atan2(vSquared + root, GRAVITY * distanceX);
        double angle2 = Math.atan2(vSquared - root, GRAVITY * distanceX);

        // Return the smaller (lower) angle
        return Math.min(angle1, angle2);
    }

    // Helper method to convert radians to degrees for easier understanding
    public static double radiansToDegrees(double angleRadians) {
        return Math.toDegrees(angleRadians);
    }

    public static void main(String[] args) {
        double velocity = 30.0; // m/s
        double targetDistanceX = 50.0; // m
        double targetDistanceY = 5.0; // m (e.g., shooting onto a small platform 5m high)

        double angleRad = calculateLaunchAngle(velocity, targetDistanceX, targetDistanceY);

        if (!Double.isNaN(angleRad)) {
            System.out.println("Required launch angle (radians): " + angleRad);
            System.out.println("Required launch angle (degrees): " + radiansToDegrees(angleRad));
        } else {
            System.out.println("Target is unreachable with the given velocity and parameters.");
        }

        // Example for flat ground (distanceY = 0)
        double angleFlatRad = calculateLaunchAngle(velocity, 80.0, 0.0);
        if (!Double.isNaN(angleFlatRad)) {
            System.out.println("\nAngle for flat ground (degrees): " + radiansToDegrees(angleFlatRad));
        }
    }
}
