/*
 =====================================================================
   PCB ROBOT  -  v10 FINAL
   Board : NodeMCU 1.0 (ESP-12E Module)
   Folder: pcb_robot / pcb_robot.ino  (ONLY this file)
 =====================================================================
   WORKFLOW PER COMPONENT:
     1. Home position (0,0)
     2. Y moves to FEEDER_Y  (fixed per component type)
        R=50mm  C=100mm  L=120mm   X stays at 0
     3. PICK component from feeder
     4. Y moves to PLACE_Y   (from cy_px detected by Python)
     5. X moves to PLACE_X   (from cx_px detected by Python)
     6. PLACE component on board
     7. Return home (0,0)

   COORDINATE MAPPING  (both axes same formula):
     pixel 0   ->  1 mm  (1mm safety offset from home)
     pixel 640 -> 201 mm
     formula: mm = 1.0 + (px / 640.0) * 200.0

   WORKSPACE  : 200 x 200 mm     Steps/mm = 400
   X_MAX      : 80400 steps      Y_MAX = 80400 steps

   FEEDER Y (pick position, X=0):
     R (Resistor)  ->  50 mm
     C (Capacitor) -> 100 mm
     L (LED)       -> 120 mm

   PIN ASSIGNMENTS:
     X STEP  D1 GPIO5    X DIR   D2 GPIO4
     Y STEP  D7 GPIO13   Y DIR   D0 GPIO16
     Z SERVO D5 GPIO14
     ENABLE  D6 GPIO12   LOW=enabled
     X LIMIT D3 GPIO0    INPUT_PULLUP (switch->GND)
     Y LIMIT D4 GPIO2    INPUT_PULLUP (switch->GND)

   IF MOTOR BEEPS/JAMS DURING HOMING:
     Flip X_HOME_DIR or Y_HOME_DIR from LOW to HIGH below.
 =====================================================================
*/

#include <ArduinoJson.h>
#include <Servo.h>

// ── PINS ─────────────────────────────────────────────────────
const int PIN_X_STEP  = 5;
const int PIN_X_DIR   = 4;
const int PIN_Y_STEP  = 13;
const int PIN_Y_DIR   = 16;
const int PIN_SERVO_Z = 12;   // D5
const int PIN_SERVO_G = 14;   // D6 (Grip)
const int PIN_X_LIMIT = 0;   // D3
const int PIN_Y_LIMIT = 2;

// ── HOMING DIRECTIONS  (flip if motor moves wrong way) ───────
const int X_HOME_DIR    = LOW;
const int X_BACKOFF_DIR = HIGH;
const int Y_HOME_DIR    = LOW;
const int Y_BACKOFF_DIR = HIGH;

// ── WORKSPACE ────────────────────────────────────────────────
const float SPM      = 200.0f;
const float WS_MM    = 200.0f;    // workspace = 200 mm both axes
const int   IMG_PX   = 640;       // YOLO image size
const float OFFSET   = 1.0f;      // 1mm safety from home

// Derived limits in steps
const long AXIS_MAX  = (long)(SPM * (WS_MM + OFFSET));  // 80400
const long AXIS_MIN  = (long)(SPM * OFFSET);            // 400  (1mm)

// ── FEEDER Y POSITIONS (mm) — pick point, X=0 ────────────────
const float FEEDER_R = 50.0f;
const float FEEDER_C = 100.0f;
const float FEEDER_L = 150.0f;

// ── SPEED ────────────────────────────────────────────────────
const int STEP_US  = 280;    // normal moves  (lower = faster)
const int HOME_US  = 300;    // homing speed
const int WDT_N    = 200;    // watchdog feed every N steps

// ── SERVO ────────────────────────────────────────────────────
// Z movement
const int Z_UP   = 20;
const int Z_DOWN = 90;
const int T_MOVE = 400;
// Gripper
const int G_OPEN  = 20;   // unlock
const int G_CLOSE = 120;   // lock
const int T_GRIP    = 350;   // ms grip hold
const int T_RELEASE = 300;   // ms release hold

// ── STATE ────────────────────────────────────────────────────
Servo srvZ;   // Up/Down
Servo srvG;   // Grip

String buf = "";
bool   busy = false;
long   posX = 0;
long   posY = 0;

// ─────────────────────────────────────────────────────────────
//  COORDINATE CONVERSION
//  Same formula for both X and Y axes.
//  pixel 0   -> AXIS_MIN (1mm)
//  pixel 640 -> AXIS_MAX (201mm)
// ─────────────────────────────────────────────────────────────
long pixToSteps(int px) {
  float mm = OFFSET + ((float)px / (float)IMG_PX) * WS_MM;
  return constrain((long)(mm * SPM), AXIS_MIN, AXIS_MAX);
}

// ─────────────────────────────────────────────────────────────
//  FEEDER Y LOOKUP  (fixed pick position per component)
// ─────────────────────────────────────────────────────────────
long feederYsteps(const char *cls) {
  if (strcmp(cls, "R") == 0) return (long)(FEEDER_R * SPM);
  if (strcmp(cls, "C") == 0) return (long)(FEEDER_C * SPM);
  if (strcmp(cls, "L") == 0) return (long)(FEEDER_L * SPM);
  return (long)(FEEDER_R * SPM);
}

// ─────────────────────────────────────────────────────────────
//  SAFE STEP  — returns actual steps completed (signed)
//  Zeros axisPos on unexpected limit hit to prevent runaway.
// ─────────────────────────────────────────────────────────────
long stepSafe(int stepPin, int dirPin, int limitPin,
              long &axisPos, long steps) {
  if (steps == 0) return 0;

  bool goNeg = (steps < 0);
  digitalWrite(dirPin, goNeg ? LOW : HIGH);

  // Block move if already at limit
  if (goNeg && digitalRead(limitPin) == LOW) {
    Serial.println(F("LIMIT-BLOCK: pos zeroed"));
    axisPos = 0;
    return 0;
  }

  long total = abs(steps);
  long done  = 0;

  for (long i = 0; i < total; i++) {
    if (goNeg && digitalRead(limitPin) == LOW) {
      Serial.println(F("LIMIT-HIT: pos zeroed"));
      axisPos = 0;
      return 0;   // return 0 so caller adds nothing to axisPos
    }
    digitalWrite(stepPin, HIGH); delayMicroseconds(STEP_US);
    digitalWrite(stepPin, LOW);  delayMicroseconds(STEP_US);
    done++;
    if (done % WDT_N == 0) ESP.wdtFeed();
  }

  return goNeg ? -done : done;
}

// ─────────────────────────────────────────────────────────────
//  GOTO  —  absolute move, both axes
// ─────────────────────────────────────────────────────────────
void goTo(long tx, long ty) {
  tx = constrain(tx, 0L, AXIS_MAX);
  ty = constrain(ty, 0L, AXIS_MAX);

  long dx = tx - posX;
  long dy = ty - posY;

  long stepsX = abs(dx);
  long stepsY = abs(dy);

  bool dirX = (dx >= 0);
  bool dirY = (dy >= 0);

  digitalWrite(PIN_X_DIR, dirX ? HIGH : LOW);
  digitalWrite(PIN_Y_DIR, dirY ? HIGH : LOW);

  long maxSteps = max(stepsX, stepsY);
  long accX = 0;
  long accY = 0;

  for (long i = 0; i < maxSteps; i++) {

    accX += stepsX;
    accY += stepsY;

    // X step
    if (accX >= maxSteps) {
      if (!( !dirX && digitalRead(PIN_X_LIMIT) == LOW )) {
        digitalWrite(PIN_X_STEP, HIGH);
        digitalWrite(PIN_X_STEP, LOW);
        posX += (dirX ? 1 : -1);
      } else {
        posX = 0;
      }
      accX -= maxSteps;
    }

    // Y step
    if (accY >= maxSteps) {
      if (!( !dirY && digitalRead(PIN_Y_LIMIT) == LOW )) {
        digitalWrite(PIN_Y_STEP, HIGH);
        digitalWrite(PIN_Y_STEP, LOW);
        posY += (dirY ? 1 : -1);
      } else {
        posY = 0;
      }
      accY -= maxSteps;
    }

    delayMicroseconds(STEP_US);

    if (i % WDT_N == 0) {
      ESP.wdtFeed();
      yield();
    }
  }

  Serial.printf("POS X=%.2fmm Y=%.2fmm\n", posX/SPM, posY/SPM);
}
// ─────────────────────────────────────────────────────────────
//  HOMING
// ─────────────────────────────────────────────────────────────
void homeOne(int stepPin, int dirPin, int limitPin,
             int homeDir, int backDir,
             long &pos, const char *name) {

  Serial.printf("Homing %s...\n", name);

  // Back off first if already at switch
  if (digitalRead(limitPin) == LOW) {
    Serial.printf("  %s: at limit, backing off first\n", name);
    digitalWrite(dirPin, backDir);
    for (long i = 0; i < (long)(8.0f * SPM); i++) {
      digitalWrite(stepPin, HIGH); delayMicroseconds(HOME_US);
      digitalWrite(stepPin, LOW);  delayMicroseconds(HOME_US);
      if (i % WDT_N == 0) ESP.wdtFeed();
    }
  }

  // Move toward switch
  digitalWrite(dirPin, homeDir);
  long count = 0;
  const long maxSteps = (long)(300.0f * SPM);

  while (digitalRead(limitPin) == HIGH && count < maxSteps) {
    digitalWrite(stepPin, HIGH); delayMicroseconds(HOME_US);
    digitalWrite(stepPin, LOW);  delayMicroseconds(HOME_US);
    count++;
    if (count % WDT_N == 0) { ESP.wdtFeed(); yield(); }
  }

  if (count >= maxSteps)
    Serial.printf("  %s: SWITCH NOT FOUND - check wiring!\n", name);
  else
    Serial.printf("  %s: found (%ld steps)\n", name, count);

  // Back off 3 mm
  digitalWrite(dirPin, backDir);
  for (long i = 0; i < (long)(3.0f * SPM); i++) {
    digitalWrite(stepPin, HIGH); delayMicroseconds(HOME_US);
    digitalWrite(stepPin, LOW);  delayMicroseconds(HOME_US);
    if (i % WDT_N == 0) ESP.wdtFeed();
  }

  pos = 0;
  Serial.printf("  %s homed. Origin = 0\n", name);
}

void homeAxes() {
  homeOne(PIN_X_STEP, PIN_X_DIR, PIN_X_LIMIT,
          X_HOME_DIR, X_BACKOFF_DIR, posX, "X");
  delay(300);
  homeOne(PIN_Y_STEP, PIN_Y_DIR, PIN_Y_LIMIT,
          Y_HOME_DIR, Y_BACKOFF_DIR, posY, "Y");
  delay(300);
  Serial.println(F("HOMED"));
}

// ─────────────────────────────────────────────────────────────
//  GRIPPER
// ─────────────────────────────────────────────────────────────
void doPick() {
  // 1. Go down
  srvZ.write(Z_DOWN);
  delay(T_MOVE);

  // 2. Close gripper (LOCK)
  srvG.write(G_CLOSE);
  delay(T_GRIP);

  // 3. Go up
  srvZ.write(Z_UP);
  delay(T_MOVE);

  Serial.println(F("PICK-OK"));
}
void doPlace(const char *cls) {
  // 1. Go down
  srvZ.write(Z_DOWN);
  delay(T_MOVE);

  // 2. Open gripper (UNLOCK)
  srvG.write(G_OPEN);
  delay(T_RELEASE);

  // 3. Go up
  srvZ.write(Z_UP);
  delay(T_MOVE);

  Serial.printf("PLACE-OK %s\n", cls);
}

// ─────────────────────────────────────────────────────────────
//  JSON HANDLER
//  Reads: component, cx_px (place X), cy_px (place Y)
//  Feeder Y is looked up by component type (fixed).
// ─────────────────────────────────────────────────────────────
void handleJSON(String &raw) {
  raw.trim();
  if (raw.length() == 0) return;

  StaticJsonDocument<2048> doc;
  if (deserializeJson(doc, raw)) {
    Serial.println(F("JSON-ERR"));
    return;
  }

  int cnt = doc["count"] | 0;
  if (cnt == 0) { Serial.println(F("NO-COMP")); return; }

  Serial.printf("JOB: %d component(s)\n", cnt);
  busy = true;

  int idx = 0;
  for (JsonObject comp : doc["components"].as<JsonArray>()) {
    idx++;
    const char *cls  = comp["component"] | "?";
    int          cxP = comp["cx_px"]     | 0;
    int          cyP = comp["cy_px"]     | 0;

    // Feeder pick position: X=0, Y=fixed per type
    long pickY  = feederYsteps(cls);

    // Board place position: both from image detection
    long placeX = pixToSteps(cxP);
    long placeY = pixToSteps(cyP);

    Serial.printf("\n[%d/%d] %s  cx_px=%d cy_px=%d\n",
                  idx, cnt, cls, cxP, cyP);
    Serial.printf("  Feeder  X=0mm      Y=%.1fmm\n", pickY  / SPM);
    Serial.printf("  Place   X=%.2fmm  Y=%.2fmm\n",
                  placeX / SPM, placeY / SPM);

    // ── STEP 1: Move to feeder  (X=0, feeder Y) ─────────────
    Serial.println(F("  -> Moving to feeder..."));
    goTo(0L, pickY);

    // ── STEP 2: PICK ─────────────────────────────────────────
    doPick();

// STEP 3: Move to place (DIAGONAL movement happens here)
    goTo(placeX, placeY);
    // ── STEP 4: PLACE ─────────────────────────────────────────
    doPlace(cls);

    ESP.wdtFeed();
    yield();
  }

  // ── Return home ──────────────────────────────────────────
    Serial.println(F("\nAll components placed. Moving to safe position..."));

// Small safe lift before homing (avoid hitting parts)
    goTo(posX, posY - (long)(10 * SPM));   // move Y up 10mm

    Serial.println(F("Starting real homing..."));

// REAL HOMING (uses limit switches)
    homeAxes();

    busy = false;
    Serial.println(F("ACK-DONE"));
}

// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_X_STEP, OUTPUT); pinMode(PIN_X_DIR, OUTPUT);
  pinMode(PIN_Y_STEP, OUTPUT); pinMode(PIN_Y_DIR, OUTPUT);

  pinMode(PIN_X_LIMIT, INPUT_PULLUP);
  pinMode(PIN_Y_LIMIT, INPUT_PULLUP);

  srvZ.attach(PIN_SERVO_Z);
  srvG.attach(PIN_SERVO_G);

  srvZ.write(Z_UP);     // arm up
  srvG.write(G_OPEN);   // gripper open

  delay(500);

  Serial.println(F("BOOTING v10..."));
  Serial.printf("Workspace : %.0fx%.0f mm  SPM=%.0f\n",
                WS_MM, WS_MM, SPM);
  Serial.printf("Feeders   : R=%.0f  C=%.0f  L=%.0f mm\n",
                FEEDER_R, FEEDER_C, FEEDER_L);
  Serial.printf("Coord map : px0->%.1fmm  px640->%.1fmm\n",
                OFFSET, OFFSET + WS_MM);

  homeAxes();

  // Python waits for this string before sending JSON
  Serial.println(F("PCB-ROBOT-READY"));
}

// ─────────────────────────────────────────────────────────────
//  LOOP  —  one char per tick, no blocking while
// ─────────────────────────────────────────────────────────────
void loop() {
  ESP.wdtFeed();
  yield();

  if (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n') {
      buf.trim();
      if (buf.length() > 0) {
        if (buf == "HOME") {
          homeAxes();
          Serial.println(F("PCB-ROBOT-READY"));
        } else if (!busy) {
          handleJSON(buf);
        } else {
          Serial.println(F("BUSY"));
        }
      }
      buf = "";
    } else if (c != '\r') {
      buf += c;
      if (buf.length() > 2048) {
        buf = "";
        Serial.println(F("BUF-OVERFLOW"));
      }
    }
  }
}
