package frc.robot.abstractMotorInterfaces;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

public class VortexMotorController extends AbstractMotorController {
    public CANSparkBase vortex;
    public RelativeEncoder encoder;
    public SparkPIDController sparkPIDController;

    public VortexMotorController(int ID){
        super();
        vortex = new CANSparkFlex(ID, CANSparkLowLevel.MotorType.kBrushless);
        encoder = vortex.getEncoder();
    }

    /**
     * Used when using Spark PID controller
     * @param ID
     * @param kP
     */
    public VortexMotorController(int ID, double kP, double kI, double IZone, double FF){
        super();
        vortex = new CANSparkFlex(ID, CANSparkLowLevel.MotorType.kBrushless);
        sparkPIDController = vortex.getPIDController();
        sparkPIDController.setP(kP);
        sparkPIDController.setI(kI);
        sparkPIDController.setD(0);
        sparkPIDController.setIZone(IZone);
        sparkPIDController.setFF(FF);
        sparkPIDController.setOutputRange(-1,1);
        vortex.burnFlash();
        encoder = vortex.getEncoder();
        
    }

    @Override
    public AbstractMotorController setInvert(boolean invert) {
        vortex.setInverted(invert);
        return this;
    }

    public RelativeEncoder getEncoder() {
        return vortex.getEncoder();
    }

    public SparkAbsoluteEncoder getAbsoluteEncoder(SparkAbsoluteEncoder.Type encoderType) {
        SparkAbsoluteEncoder.Type newType;
        newType = SparkAbsoluteEncoder.Type.kDutyCycle;
        return vortex.getAbsoluteEncoder(newType);
    }

    @Override
    public void setVelocity(double Velocity, double Acceleration, boolean FOC, double feed, int PIDSLOT, boolean brake, boolean forwardLimit, boolean backwardLimit) {}

    @Override
    public void set(double Percent) {
        vortex.set(Percent);
    }

    @Override
    public void setPosition(double Position) {
        vortex.getEncoder().setPosition(Position);
    }

    public SparkPIDController getPIDController(){
        return vortex.getPIDController();
    }

    @Override
    public void setPosition(double Position, boolean FOC, double feed, int PidSlot, boolean brake, boolean forwardlimit, boolean backwardlimit, double Velocity) {}
    
    public void setReferencePercent(double percent){
        sparkPIDController.setReference(percent, CANSparkBase.ControlType.kDutyCycle);
    }


    @Override
    public AbstractMotorController setBrake(boolean brake) {
        if (brake) {
            vortex.setIdleMode(IdleMode.kBrake);
        } else {
            vortex.setIdleMode(IdleMode.kCoast);
        }
        return this;
    }
    public double getSpeed(){
        return vortex.getAppliedOutput();
    }

    /**
     * @return RPM of Vortex
     */
    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void setVelocity(double Velocity) {
        sparkPIDController.setReference(Velocity, CANSparkBase.ControlType.kVelocity);
    }

    public void setReference(double setpoint){
        sparkPIDController.setReference(setpoint, CANSparkBase.ControlType.kPosition);
    }
  
    @Override
    public double getAngularVelocity() {
        return 0;
    }

    @Override
    public double getRotations() {
        return encoder.getPosition() * sensorToRealDistanceFactor;
    }

    @Override
    public double getVoltage() {
        return vortex.getBusVoltage();
    }

    @Override
    public void setVoltage(double Voltage) {
        vortex.setVoltage(Voltage);
    }

    @Override
    public double getCurrent() {
        return vortex.getOutputCurrent();
    }

    @Override
    public int getID() {
        return vortex.getDeviceId();
    }

    @Override
    public void setCurrentLimit(int limit) {
        vortex.setSmartCurrentLimit(limit);
    }

    @Override
    public void setOpenLoopRampRate(double Ramp) {
        vortex.setOpenLoopRampRate(Ramp);
    }

    @Override
    public void setRange(double minRange, double maxRange) {}

    public AbstractMotorController follow(AbstractMotorController leader,boolean invert){
        return this;
    }
}