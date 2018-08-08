#include "pinout.h"
#include "robotGeometry.h"
#include "interpolation.h"
#include "fanControl.h"
#include "RampsStepper.h"
#include "queue.h"
#include "command.h"

RampsStepper stepperRotate(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN);
RampsStepper stepperLower(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);
RampsStepper stepperHigher(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN);
RampsStepper stepperExtruder(E_STEP_PIN, E_DIR_PIN, E_ENABLE_PIN);
RobotGeometry geometry;
Interpolation interpolator;
Queue<Cmd> queue(15);
Command command;


void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);     


  //reduction of steppers..
  stepperHigher.setReductionRatio(32.0 / 10.0, 200 * 16);  //big gear: 32, small gear: 9, steps per rev: 200, microsteps: 16
  stepperLower.setReductionRatio(32.0 / 10.0, 200 * 16);
  stepperRotate.setReductionRatio(32.0 / 10.0, 200 * 16);

  //start positions..
  stepperHigher.setPositionRad(PI / 2.0);  //90°
  stepperLower.setPositionRad(0);          // 0°
  stepperRotate.setPositionRad(0);         // 0°

  //enable and init..
  setStepperEnable(false);
  interpolator.setInterpolation(0, 240, 240, 0, 0, 240, 240, 0);
  //interpolator.setInterpolation(0, 0, 0, 0, 0, 0, 0, 0);

  Serial.println("start");
}

void setStepperEnable(bool enable) {
  stepperRotate.enable(enable);
  stepperLower.enable(enable);
  stepperHigher.enable(enable);

}

void loop () {
  //update and Calculate all Positions, Geometry and Drive all Motors...
  interpolator.updateActualPosition();
  geometry.set(interpolator.getXPosmm(), interpolator.getYPosmm(), interpolator.getZPosmm());
  stepperRotate.stepToPositionRad(geometry.getRotRad());
  stepperLower.stepToPositionRad (geometry.getLowRad());
  stepperHigher.stepToPositionRad(geometry.getHighRad());
  stepperRotate.update();
  stepperLower.update();
  stepperHigher.update();

  if (!queue.isFull()) {
    if (command.handleGcode()) {
      Serial.print("HANDLE CMD: ");
      Serial.println(command.getCmd().id);
      queue.push(command.getCmd());
      printOk();
    }
  }
  if ((!queue.isEmpty()) && interpolator.isFinished()) {
    executeCommand(queue.pop());
  }

  if (millis() % 500 < 250) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void cmdMove(Cmd (&cmd)) {
  Serial.print(cmd.valueX);
  Serial.print(", ");
  Serial.print(cmd.valueY);
  Serial.print(", ");
  Serial.print(cmd.valueZ);
  Serial.print(", ");
  Serial.print(cmd.valueE);
  Serial.print(", ");
  Serial.println(cmd.valueF);

  interpolator.setInterpolation(cmd.valueX, cmd.valueY, cmd.valueZ, 0, cmd.valueF);
}
void cmdDwell(Cmd (&cmd)) {
  delay(int(cmd.valueT * 1000));
}
void cmdGripperOn(Cmd (&cmd)) {
}
void cmdGripperOff(Cmd (&cmd)) {

}
void cmdStepperOn() {
  setStepperEnable(true);
}
void cmdStepperOff() {
  setStepperEnable(false);
}
void cmdFanOn() {
}
void cmdFanOff() {
}

void handleAsErr(Cmd (&cmd)) {
  printComment("Unknown Cmd " + String(cmd.id) + String(cmd.num) + " (queued)");
  printFault();
}

void executeCommand(Cmd cmd) {
  if (cmd.id == -1) {
    String msg = "parsing Error";
    printComment(msg);
    handleAsErr(cmd);
    return;
  }

  if (cmd.valueX == NAN) {
    cmd.valueX = interpolator.getXPosmm();
  }
  if (cmd.valueY == NAN) {
    cmd.valueY = interpolator.getYPosmm();
  }
  if (cmd.valueZ == NAN) {
    cmd.valueZ = interpolator.getZPosmm();
  }
  if (cmd.valueE == NAN) {
    cmd.valueE = interpolator.getEPosmm();
  }

  Serial.print("EXEC CMD: ");
  Serial.println(cmd.id);

  //decide what to do
  if (cmd.id == 'G') {
    switch (cmd.num) {
      case 0: cmdMove(cmd); break;
      case 1: cmdMove(cmd); break;
      case 4: cmdDwell(cmd); break;
      //case 21: break; //set to mm
      //case 90: cmdToAbsolute(); break;
      //case 91: cmdToRelative(); break;
      //case 92: cmdSetPosition(cmd); break;
      default: handleAsErr(cmd);
    }
  } else if (cmd.id == 'M') {
    switch (cmd.num) {
      //case 0: cmdEmergencyStop(); break;
      case 3: cmdGripperOn(cmd); break;
      case 5: cmdGripperOff(cmd); break;
      case 17: cmdStepperOn(); break;
      case 18: cmdStepperOff(); break;
      case 106: cmdFanOn(); break;
      case 107: cmdFanOff(); break;
      default: handleAsErr(cmd);
    }
  } else {
    handleAsErr(cmd);
  }
}


