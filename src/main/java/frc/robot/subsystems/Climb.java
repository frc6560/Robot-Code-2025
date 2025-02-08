package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import static frc.robot.utility.NetworkTable.NtValueDisplay.ntDispTab;
import frc.robot.ManualControls;

public class Climb extends SubsystemBase {
    
    private TalonFX leader;
    private TalonFX follower;
    private final ManualControls controls; // Reference to controller input

    private static final int LEADER_ID = 20;
    private static final int FOLLOWER_ID = 21;
    private static final double MAX_CLIMB_SPEED = 0.7; // Scale joystick input

    public Climb(ManualControls controls) { 
        this.leader = new TalonFX(LEADER_ID, "Canivore");
        this.follower = new TalonFX(FOLLOWER_ID, "Canivore");
        this.controls = controls; // Store the passed controls reference

        // Set follower to follow leader
        this.follower.setControl(new Follower(LEADER_ID, true)); 

        ntDispTab("Climb")
        .add("Climb Duty Cycle", this::getDutyCycle);
    }

    public void setClimbSpeed() {
        double speed = -controls.getClimbSpeed() * MAX_CLIMB_SPEED; // Inverted joystick Y-axis
        leader.set(speed);
    }

    public void stop() {
        leader.set(0);
    }

    public double getDutyCycle() {
        return leader.get();
    }

    // Ensure this runs continuously
    @Override
    public void periodic() {
        setClimbSpeed();
    }
}
