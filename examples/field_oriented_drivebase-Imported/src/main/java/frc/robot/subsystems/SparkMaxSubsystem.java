package frc.robot.subsystems;

import java.util.logging.Logger;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTableInstance;

public class SparkMaxSubsystem extends SubsystemBase {

    // Create 4 motors (using PWM channels; change channels as needed)
    private final PWMSparkMax motor1 = new PWMSparkMax(0);
    private final PWMSparkMax motor2 = new PWMSparkMax(1);
    private final PWMSparkMax motor3 = new PWMSparkMax(2);
    private final PWMSparkMax motor4 = new PWMSparkMax(3);

    // Controller (use port 0 — change if needed). Fully-qualified name to avoid adding imports.
    private final edu.wpi.first.wpilibj.XboxController controller = new edu.wpi.first.wpilibj.XboxController(0);

    // Speeds (will be driven from the controller)
    private double speed1 = 0.5;
    private double speed2 = 0.5;
    private double speed3 = 0.5;
    private double speed4 = 0.5;

    public SparkMaxSubsystem() {
        // Optionally publish initial speeds to SmartDashboard (not required for controller control)
        SmartDashboard.putNumber("Motor1 Speed", speed1);
        SmartDashboard.putNumber("Motor2 Speed", speed2);
        SmartDashboard.putNumber("Motor3 Speed", speed3);
        SmartDashboard.putNumber("Motor4 Speed", speed4);
    }

    @Override
    public void periodic() {
        // Read controller sticks and drive motors directly from the controller.
        // L/R sticks control motor1 and motor2; triggers control motor3 and motor4.
        double leftY = -controller.getLeftY();
        double rightY = -controller.getRightY();
        double triggerLeft = controller.getLeftTriggerAxis();
        double triggerRight = controller.getRightTriggerAxis();

        speed1 = leftY;
        speed2 = rightY;
        speed3 = triggerLeft;
        speed4 = triggerRight;

        // Apply speeds to motors so outputs reflect controller input
        motor1.set(speed1);
        motor2.set(speed2);
        motor3.set(speed3);
        motor4.set(speed4);

        // Publish outputs to SmartDashboard so the simulator GUI shows values
        SmartDashboard.putNumber("Motor1 Output", motor1.get());
        SmartDashboard.putNumber("Motor2 Output", motor2.get());
        SmartDashboard.putNumber("Motor3 Output", motor3.get());
        SmartDashboard.putNumber("Motor4 Output", motor4.get());

        // PWMSparkMax does not provide an encoder API; publish NaN as a placeholder for velocity
        SmartDashboard.putNumber("Motor1 Velocity", Double.NaN);
        SmartDashboard.putNumber("Motor2 Velocity", Double.NaN);
        SmartDashboard.putNumber("Motor3 Velocity", Double.NaN);
        SmartDashboard.putNumber("Motor4 Velocity", Double.NaN);

        // Also publish to NetworkTables (keeps previous behavior)
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        nt.getDoubleTopic("Motor1/Output").publish().set(motor1.get());
        nt.getDoubleTopic("Motor1/Velocity").publish().set(Double.NaN);
        nt.getDoubleTopic("Motor2/Output").publish().set(motor2.get());
        nt.getDoubleTopic("Motor2/Velocity").publish().set(Double.NaN);
        nt.getDoubleTopic("Motor3/Output").publish().set(motor3.get());
        nt.getDoubleTopic("Motor3/Velocity").publish().set(Double.NaN);
        nt.getDoubleTopic("Motor4/Output").publish().set(motor4.get());
        nt.getDoubleTopic("Motor4/Velocity").publish().set(Double.NaN);

        // Print a concise line to the console so you can see activity in the simulator log
        System.out.println(String.format(
            "Motor outputs: [%.2f, %.2f, %.2f, %.2f]",
            motor1.get(), motor2.get(), motor3.get(), motor4.get()));
    }

    // Individual motor controls
    public void runMotor1() { motor1.set(speed1); }
    public void runMotor2() { motor2.set(speed2); }
    public void runMotor3() { motor3.set(speed3); }
    public void runMotor4() { motor4.set(speed4); }

    public void stopAll() {
        motor1.stopMotor();
        motor2.stopMotor();
        motor3.stopMotor();
        motor4.stopMotor();
    }
}