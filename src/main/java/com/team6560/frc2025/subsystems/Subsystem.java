package com.team6560.frc2025.subsystems;
// import anything you might need here.

import com.ctre.phoenix6.hardware.TalonFX;
// just as an example, I imported ctre's TalonFX class. 
// perhaps you'll need a different one. 
// typically speaking, you can find documentation on your motor with a simple google search. 
// searching up the motor name and then "frc" should be good enough here!
// sometimes, an encoder is built into the motor. be careful.


import com.ctre.phoenix6.hardware.CANcoder;
// this is just an example; again, you might need a different encoder class.
// typically you can find documentation on your encoder with a simple google search.

/** A prototypical generic subsystem. */
public class Subsystem {
    public final TalonFX m_subsystemMotor;
    public final CANcoder m_subsystemEncoder;

    /**
     * Constructs a new Subsystem.
     */
    public Subsystem() {
        // Initialize the motor and encoder here.
        m_subsystemMotor = new TalonFX(1); // Replace 1 with your motor's CAN ID. Bug CAD/Build to do this successfully. If they resist, tell me.
        m_subsystemEncoder = new CANcoder(2); // Replace 2 with your encoder's ID. See above. Why is this a CANCoder? because it's a type of absolute encoder.
    }

    /** This is how you set a motor. */
    public void setMotor(double speed) {
        // Set the motor speed here.
        // This is in RPM of the motor. If you want something like elevator velocity, you'll have to convert. Be aware.
        // There's other ways we can do this, as you will see!
        m_subsystemMotor.set(speed);
    }
}
