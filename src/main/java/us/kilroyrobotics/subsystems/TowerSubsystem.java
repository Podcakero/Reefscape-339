// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import us.kilroyrobotics.Constants;
import us.kilroyrobotics.util.TowerEvent;
import us.kilroyrobotics.util.TowerState;

public class TowerSubsystem extends SubsystemBase {

    private TowerState currentState = TowerState.INIT;
    private TowerEvent pendingEvent = TowerEvent.NONE;

    private Timer stateTimer = new Timer();

    private Elevator elevator;
    private Wrist wrist;
    private CoralIntakeMotor coralIntakeMotor;

    private double safetyFactor = 1;
    private int currentLevel = 0;

    /** Creates a new Tower. */
    public TowerSubsystem(Elevator elevator, Wrist wrist, CoralIntakeMotor coralIntakeMotor) {
        this.elevator = elevator;
        this.wrist = wrist;
        this.coralIntakeMotor = coralIntakeMotor;

        stateTimer.start();
    }

    public void initialize() {
        pendingEvent = TowerEvent.NONE;
        setState(TowerState.INIT);

        wrist.stop();
        elevator.stop();
    }

    public void home() {
        coralIntakeMotor.setSpeed(0);

        elevator.resetClosedLoopControl();
        wrist.resetClosedLoopControl();

        pendingEvent = TowerEvent.NONE;
        setState(TowerState.INIT);
    }

    public void runStateMachine() {
        switch (currentState) {
            case INIT:
                elevator.setPosition(Constants.ElevatorConstants.kCoralStationHeight);
                wrist.setAngle(Constants.CoralMechanismConstants.kIntakingAngle);
                coralIntakeMotor.setSpeed(Constants.CoralMechanismConstants.kWheelSpeedIntaking);

                setState(TowerState.HOMING);
                break;
            case HOMING:
                if (wrist.inPosition() && elevator.inPosition()) setState(TowerState.HOME);
                break;
            case HOME:
                if (isTriggered(TowerEvent.INTAKE_CORAL)) setState(TowerState.TILTING_TO_INTAKE);
                else if (coralIntakeMotor.isCoralDetected()) setState(TowerState.GOT_CORAL);
                else currentLevel = 0;
                break;
            case RAISING_TO_INTAKE:
                if (elevator.inPosition()) {
                    wrist.setAngle(Constants.CoralMechanismConstants.kIntakingAngle);

                    setState(TowerState.INTAKING);
                }
            case TILTING_TO_INTAKE:
                if (wrist.inPosition()) {
                    coralIntakeMotor.setSpeed(
                            Constants.CoralMechanismConstants.kWheelSpeedIntaking);

                    setState(TowerState.RAISING_TO_INTAKE);
                }
            case INTAKING:
                if (coralIntakeMotor.isCoralDetected()) {
                    coralIntakeMotor.setSpeed(Constants.CoralMechanismConstants.kWheelSpeedHolding);

                    setState(TowerState.GOT_CORAL);
                }
                break;
            case GOT_CORAL:
                if (isTriggered(TowerEvent.GOTO_L1)) {
                    elevator.setPosition(Constants.ElevatorConstants.kL1Height);
                    coralIntakeMotor.setSpeed(0);

                    currentLevel = 1;
                    setState(TowerState.RAISING_TO_L1);
                } else if (isTriggered(TowerEvent.GOTO_L2)) {
                    elevator.setPosition(Constants.ElevatorConstants.kL2Height);
                    coralIntakeMotor.setSpeed(0);

                    currentLevel = 2;
                    setState(TowerState.RAISING_TO_L2);
                } else if (isTriggered(TowerEvent.GOTO_L3)) {
                    elevator.setPosition(Constants.ElevatorConstants.kL3Height);
                    coralIntakeMotor.setSpeed(0);

                    currentLevel = 3;
                    setState(TowerState.RAISING_TO_L3);
                } else if (isTriggered(TowerEvent.GOTO_L4)) {
                    elevator.setPosition(Constants.ElevatorConstants.kL4Height);
                    coralIntakeMotor.setSpeed(0);

                    currentLevel = 4;
                    setState(TowerState.RAISING_TO_L4);
                } else if (!coralIntakeMotor.isCoralDetected()) coralIntakeMotor.setSpeed(0);
                break;
            case RAISING_TO_L1:
                if (elevator.inPosition()) {
                    wrist.setAngle(Constants.CoralMechanismConstants.kScoringL1);

                    setState(TowerState.TILTING_TO_SCORE);
                }
                break;
            case RAISING_TO_L2:
                if (elevator.inPosition()) {
                    wrist.setAngle(Constants.CoralMechanismConstants.kScoringL2);

                    setState(TowerState.TILTING_TO_SCORE);
                }
                break;
            case RAISING_TO_L3:
                if (elevator.inPosition()) {
                    wrist.setAngle(Constants.CoralMechanismConstants.kScoringL3);

                    setState(TowerState.TILTING_TO_SCORE);
                }
                break;
            case RAISING_TO_L4:
                if (elevator.inPosition()) {
                    wrist.setAngle(Constants.CoralMechanismConstants.kScoringL4);

                    setState(TowerState.TILTING_TO_SCORE);
                }
                break;
            case TILTING_TO_SCORE:
                if (wrist.inPosition()) setState(TowerState.READY_TO_SCORE_CORAL);
                break;
            case READY_TO_SCORE_CORAL:
                if (isTriggered(TowerEvent.SCORE)) {
                    coralIntakeMotor.setSpeed(
                            Constants.CoralMechanismConstants.kWheelSpeedOuttaking);

                    setState(TowerState.SCORING_CORAL);
                } else if ((isTriggered(TowerEvent.GOTO_L4)) && (currentLevel != 4)) {
                    elevator.setPosition(Constants.ElevatorConstants.kL4Height);
                    wrist.setAngle(Constants.CoralMechanismConstants.kScoringL4);

                    currentLevel = 4;
                    setState(TowerState.RAISING_TO_L4);
                } else if ((isTriggered(TowerEvent.GOTO_L3)) && (currentLevel != 3)) {
                    elevator.setPosition(Constants.ElevatorConstants.kL3Height);
                    wrist.setAngle(Constants.CoralMechanismConstants.kScoringL3);

                    currentLevel = 3;
                    setState(TowerState.RAISING_TO_L3);
                } else if ((isTriggered(TowerEvent.GOTO_L2)) && (currentLevel != 2)) {
                    elevator.setPosition(Constants.ElevatorConstants.kL2Height);
                    wrist.setAngle(Constants.CoralMechanismConstants.kScoringL2);

                    currentLevel = 2;
                    setState(TowerState.RAISING_TO_L2);
                } else if ((isTriggered(TowerEvent.GOTO_L1)) && (currentLevel != 1)) {
                    elevator.setPosition(Constants.ElevatorConstants.kL1Height);
                    wrist.setAngle(Constants.CoralMechanismConstants.kScoringL1);

                    currentLevel = 1;
                    setState(TowerState.RAISING_TO_L1);
                }
                break;
            case SCORING_CORAL:
                if (!coralIntakeMotor.isCoralDetected() || stateTimer.hasElapsed(0.3)) {
                    wrist.setAngle(Constants.CoralMechanismConstants.kIntakingAngle);

                    setState(TowerState.PAUSING_AFTER_SCORING_CORAL);
                }
                break;
            case PAUSING_AFTER_SCORING_CORAL:
                if (wrist.inPosition() && stateTimer.hasElapsed(0.2)) {
                    elevator.setPosition(Constants.ElevatorConstants.kCoralStationHeight);

                    setState(TowerState.INIT);
                }
                break;
        }
    }

    public void triggerEvent(TowerEvent event) {
        pendingEvent = event;
    }

    public TowerEvent getPendingEvent() {
        return pendingEvent;
    }

    public TowerState getState() {
        return currentState;
    }

    private Boolean isTriggered(TowerEvent event) {
        if (pendingEvent == event) {
            pendingEvent = TowerEvent.NONE;
            return true;
        } else {
            return false;
        }
    }

    private void setState(TowerState newState) {
        currentState = newState;
        stateTimer.reset();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        runStateMachine();
    }
}
