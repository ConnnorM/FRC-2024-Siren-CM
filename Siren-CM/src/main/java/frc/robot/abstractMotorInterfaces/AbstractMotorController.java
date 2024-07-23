package frc.robot.abstractMotorInterfaces;

public abstract class AbstractMotorController {

    public double sensorToRealDistanceFactor = 1D;

    public abstract void setVelocity(double Velocity);

    public abstract AbstractMotorController setInvert(boolean invert);

    public abstract void setVelocity(double Velocity, double Acceleration, boolean FOC, double feed, int PIDSLOT, boolean brake, boolean forwardLimit, boolean backwardLimit);

    public abstract void set(double Percent);

    public abstract void setPosition(double Position);

    public abstract void setPosition(double Position, boolean FOC, double feed, int PidSlot, boolean brake, boolean forwardlimit, boolean backwardlimit, double Velocity);

    public abstract AbstractMotorController setBrake(boolean brake);

    public abstract double getVelocity();
    
    public abstract double getAngularVelocity();

    public abstract double getRotations();

    public abstract double getVoltage();

    public abstract void setVoltage(double Voltage);

    public abstract double getCurrent();

    public abstract int getID();

    public abstract void setCurrentLimit(int limit);

    /**
     * @param Ramp This sets the rate at which you want the motor to reach its speed.
     */
    public abstract void setOpenLoopRampRate(double Ramp);


    /**
     * @param minRange The lower end of the range you don't want the motor to go into.
     * @param maxRange The higher end of the range you want want the motor to go into.
     * @return Sets the range you do not want the motor to go into.
     */
    public abstract void setRange(double minRange, double maxRange);

    public AbstractMotorController follow(AbstractMotorController leader) {
        return follow(leader, false);
    }

    public abstract AbstractMotorController follow(AbstractMotorController leader, boolean invert);


}
