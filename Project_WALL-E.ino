  #include <WiFi.h>
  #include <WebServer.h>
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  #include <math.h>
  #include <stdlib.h>

  // --- MOTOR DRIVER PINS (matched to our wiring) ---
  #define ENA 25  // Left motor speed (PWM)
  #define IN1 27  // Left motor dir 1 (positive speed = forward)
  #define IN2 26  // Left motor dir 2
  #define IN3 14  // Right motor dir 1
  #define IN4 13  // Right motor dir 2
  #define ENB 33  // Right motor speed (PWM), original wiring

  // --- SENSOR / SERVO PINS ---
  #define LEFT_ENCODER_PIN 35      // left wheel encoder; GPIO35 is input-only (external signal conditioning required)
  #define RIGHT_ENCODER_PIN 34     // right wheel encoder; GPIO34 is input-only (external signal conditioning required)
  // GPIO25 is already assigned to ENA (left motor PWM), so it is not an encoder input.
  #define OLED_SDA_PIN 22
  #define OLED_SCL_PIN 23
  #define TRIG_PIN 18
  #define ECHO_PIN 19
  #define SERVO_PIN 21

  // --- DUAL-ENCODER SPEED / OLED ---
  #define OLED_WIDTH 128
  #define OLED_HEIGHT 64
  #define OLED_RESET -1
  const uint8_t OLED_I2C_ADDRESS = 0x3C; // common SSD1306 address; use 0x3D if your module is strapped that way
  const float WHEEL_CIRCUMFERENCE_M = 0.21991f; // pi * 0.07 m for a 7 cm wheel diameter
  const uint16_t SPEED_ENCODER_PULSES_PER_REV = 20; // encoder edges per wheel revolution
  const float SPEED_MAX_KMH = 10.0f;           // hard display/measurement ceiling
  const uint32_t SPEED_UPDATE_MS = 250;
  const uint32_t SPEED_ZERO_TIMEOUT_MS = 1200;
  Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
  // Shared counters let speed display and odometry use both wheel sensors.
  volatile uint32_t leftEncoderTicks = 0;
  volatile uint32_t rightEncoderTicks = 0;
  volatile uint32_t leftEncoderLastUs = 0;
  volatile uint32_t rightEncoderLastUs = 0;
  portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
  uint32_t speedLeftPreviousTicks = 0;
  uint32_t speedRightPreviousTicks = 0;
  uint32_t lastWheelMotionMs = 0;
  float measuredSpeedKmh = 0.0f;
  float targetXcm = 0.0f;
  float targetYcm = 0.0f;
  bool targetSet = false;
  uint32_t lastSpeedUpdateMs = 0;
  uint32_t lastDisplayUpdateMs = 0;
  bool oledReady = false;
  extern int dispLeft;  // definitions remain with the original car state below
  extern int dispRight;

  // Average both wheel encoder distances over a fixed time window for km/h display.
  void updateSpeedMeasurement() {
    const uint32_t nowMs = millis();
    const uint32_t elapsedMs = nowMs - lastSpeedUpdateMs;
    if (elapsedMs < SPEED_UPDATE_MS) return;
    lastSpeedUpdateMs = nowMs;

    uint32_t leftNow, rightNow;
    portENTER_CRITICAL(&encoderMux);
    leftNow = leftEncoderTicks;
    rightNow = rightEncoderTicks;
    portEXIT_CRITICAL(&encoderMux);

    const uint32_t leftEdges = leftNow - speedLeftPreviousTicks;
    const uint32_t rightEdges = rightNow - speedRightPreviousTicks;
    speedLeftPreviousTicks = leftNow;
    speedRightPreviousTicks = rightNow;

    if (leftEdges || rightEdges) {
      const float distancePerEdgeM = WHEEL_CIRCUMFERENCE_M / (float)SPEED_ENCODER_PULSES_PER_REV;
      const float averageDistanceM = 0.5f * (leftEdges + rightEdges) * distancePerEdgeM;
      const float rawSpeedKmh = constrain(averageDistanceM * 3.6f / (elapsedMs / 1000.0f), 0.0f, SPEED_MAX_KMH);
      measuredSpeedKmh = constrain(measuredSpeedKmh * 0.65f + rawSpeedKmh * 0.35f, 0.0f, SPEED_MAX_KMH);
      lastWheelMotionMs = nowMs;
    } else if (nowMs - lastWheelMotionMs >= SPEED_ZERO_TIMEOUT_MS) {
      measuredSpeedKmh = 0.0f;
    }
  }

  String modeName();

  String displayModeName();

  void updateSpeedDisplay() {
    uint32_t now = millis();
    if (!oledReady || now - lastDisplayUpdateMs < 500) return;
    lastDisplayUpdateMs = now;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    String mode = displayModeName();
    mode.toUpperCase();
    display.setCursor(0, 0);
    display.print("MODE: "); display.print(mode);
    display.setTextSize(2);
    display.setCursor(0, 15);
    display.print(constrain(measuredSpeedKmh, 0.0f, SPEED_MAX_KMH), 1);
    display.setTextSize(1);
    display.print(" km/h");
    display.setCursor(0, 40);
    if ((mode == "AUTO" || mode == "DODGE") && targetSet) {
      display.print("DST X:"); display.print(targetXcm, 0); display.print(" cm");
      display.setCursor(0, 52);
      display.print("DST Y:"); display.print(targetYcm, 0); display.print(" cm");
    } else if (mode == "AUTO") {
      display.print("DST: waiting for target");
    } else if (mode == "DODGE") {
      display.print("Avoiding obstacles");
    } else {
      display.print("ESP32 differential drive");
    }
    display.display();
  }

  // --- MOTOR PWM (ESP32 core v3.x) ---
  const int PWM_FREQ = 5000;
  const int PWM_RES = 8; // 0 - 255

  // --- SERVO PWM (driven directly, no ESP32Servo library) ---
  const int SERVO_PWM_FREQ = 50;
  const int SERVO_PWM_RES = 16;
  const int SERVO_MIN_US = 500;   // pulse for 0 deg   (try 600 if the servo buzzes at 0)
  const int SERVO_MAX_US = 2400;  // pulse for 180 deg (try 2300 if it buzzes at 180)

  // Web server on port 80
  WebServer server(80);

  // Wi-Fi credentials (written as arrays on purpose)
  const char ssid[] = "ESP32_Car_Control";
  const char password[] = "password123";

  // --- MODES ---
  #define MODE_MANUAL 0
  #define MODE_AUTO 1      // obstacle-avoiding mode
  #define MODE_FOLLOW 2
  #define MODE_DODGE 3
  int carMode = MODE_MANUAL;

  // --- CAR STATE ---
  bool emergencyStop = false;
  bool isAccelerating = false;
  bool isBraking = false;
  bool steerLeftBtn = false;
  bool steerRightBtn = false;
  float tiltAngle = 0.0;
  float buttonSteer = 0.0;        // smoothed button steering, -1..+1

  float currentSpeed = 0.0;       // -255 to +255
  const float MAX_SPEED = 255.0;
  const float ACCEL_RATE = 16.0; // faster manual response; reaches motor-effective PWM much sooner
  const float COAST_RATE = 2.0;
  const float BRAKE_RATE = 10.0;

  int dispLeft = 0;               // wheel speeds shown on the phone (-255..255)
  int dispRight = 0;

  // Gradual steering with LEFT / RIGHT buttons
  const float BUTTON_STEER_MAX = 0.6;  // fraction of full steering (0.6 = 60% of max tilt)
  const float STEER_RAMP = 0.02;       // build-up per 20 ms tick while held
  const float STEER_RETURN = 0.05;     // return-to-straight speed after release
  const float BUTTON_STEER_SIGN = -1.0; // -1.0 flips LEFT/RIGHT buttons; set to 1.0 to undo

  // --- PHONE TILT -> STEERING CALIBRATION (the 3 values below can be tuned live from the page) ---
  const float TILT_SIGN = 1.0;         // set to -1.0 to flip steering permanently
  const float TILT_MAX_DEG = 40.0;     // tilt beyond this counts as 40
  float tiltDeadzoneDeg = 4.0;         // ignore small wobble around centre (degrees)
  float innerSlowPerDeg = 0.020;       // inner wheels lose 2.0% speed per degree of tilt
  float outerFastPerDeg = 0.008;       // outer wheels gain 0.8% speed per degree of tilt

  unsigned long lastPhysicsUpdate = 0;
  unsigned long lastCommandTime = 0;
  const unsigned long COMMAND_TIMEOUT_MS = 700; // release controls if phone goes silent

  // =====================================================================
  //  AUTONOMOUS MODES: SETTINGS
  // =====================================================================

  // Where things physically are on THIS build
  const bool CHANNELS_SWAPPED = true;     // true = the "left" motor channel (ENA/IN1/IN2) is on the car's RIGHT side
  const bool SERVO_HIGH_IS_LEFT = true;   // true = servo angles above 90 look to the car's LEFT

  // Car size: 17 cm wide x 28 cm long
  const float CAR_HALF_WIDTH_CM = 8.5;
  const float CORRIDOR_MARGIN_CM = 4.5;
  const float HALF_CORRIDOR_CM = CAR_HALF_WIDTH_CM + CORRIDOR_MARGIN_CM;  // 13 cm: closer to the centre line than this = in the way

  // --- AUTO navigation / dodge configuration (all position units are centimetres) ---
  const float WHEEL_TRACK_CM = 14.0f;       // CALIBRATE: centre-to-centre distance between drive wheels
  const float TARGET_TOLERANCE_CM = 8.0f;
  const float SAFE_OBSTACLE_CM = 25.0f;
  const float MIN_ESCAPE_CLEARANCE_CM = 30.0f;
  const float DODGE_WAYPOINT_CM = 40.0f;
  const float DODGE_ANGLE_DEG = 45.0f;
  const uint32_t DODGE_WAYPOINT_TIMEOUT_MS = 8000;
  const uint32_t DODGE_TURN_TIMEOUT_MS = 5000;
  const uint8_t MAX_DODGE_ATTEMPTS = 3;
  const bool PREFER_LEFT_ON_TIE = true;   // user-adjustable side choice when scans are equally clear
  const float DODGE_CLEARANCE_TIE_CM = 2.0f;
  const int NAV_MIN_EFFECTIVE_PWM = 160; // raise small commands above typical motor stall duty
  const int NAV_CRUISE_PWM = 180;
  const int NAV_MAX_PWM = 220;
  const int NAV_TURN_PWM = 175;
  const int NAV_PWM_RAMP_STEP = 15;           // applied every 20 ms for gentle starts/stops
  const float NAV_HEADING_KP = 80.0f;        // PWM per radian when following a target vector
  const float NAV_HEADING_DEADBAND_DEG = 5.0f;
  const unsigned long NAV_UPDATE_MS = 80;
  const uint32_t AUTO_NO_ENCODER_PROGRESS_MS = 2500;
  const float AUTO_ENCODER_PROGRESS_CM = 0.75f;
  const float AUTO_ENCODER_PROGRESS_DEG = 2.0f;
  const unsigned long SERVO_SETTLE_MS = 280;
  const unsigned long ULTRASONIC_PERIOD_MS = 70;
  const unsigned long ULTRASONIC_TIMEOUT_US = 30000;
  const int SERVO_CENTER = 90;
  const int SERVO_LOOK_LEFT = 135;
  const int SERVO_LOOK_RIGHT = 45;

  // Encoder geometry: 7 cm diameter wheel and 20 edges/revolution, confirmed by user.
  const float WHEEL_CIRCUMFERENCE_CM = 21.991f;
  const uint16_t ENCODER_PULSES_PER_REV = 20;
  const uint32_t ENCODER_MIN_EDGE_US = 1500;

  // --- FOLLOW mode (object follower) ---
  const long FOLLOW_MIN_CM = 10;          // below this distance, back away
  const long FOLLOW_HOLD_MAX_CM = 15;     // inclusive hold band: 10 through 15 cm
  const long FOLLOW_MAX_CM = 30;          // source object-following logic tracks targets within 30 cm
  const int FOLLOW_MIN_SPEED = 140;
  const int FOLLOW_MAX_SPEED = 200;
  const int FOLLOW_TURN_SPEED = 190;      // pivot speed when the object is far to one side
  const float FOLLOW_DEADBAND_DEG = 8.0;  // object this close to straight ahead = drive straight
  const float FOLLOW_PIVOT_DEG = 35.0;    // object farther off than this = turn in place first
  const unsigned long FOLLOW_TICK_MS = 70;
  const unsigned long FOLLOW_STALE_MS = 800;   // ignore echoes older than this

  const int FOLLOW_ANGLES[] = {45, 60, 75, 90, 105, 120, 135};
  const int FOLLOW_COUNT = 7;

  // =====================================================================
  //  ODOMETRY, ASYNCHRONOUS RANGE SENSOR, AND NAVIGATION STATE
  // =====================================================================
  enum AutoState : uint8_t { STATE_IDLE, STATE_NAVIGATING, STATE_AVOIDING };
  enum ContinuousDodgePhase : uint8_t {
    DODGE_CRUISE, DODGE_SCAN_LEFT_SETTLE, DODGE_SCAN_LEFT_WAIT,
    DODGE_SCAN_RIGHT_SETTLE, DODGE_SCAN_RIGHT_WAIT,
    DODGE_TO_WAYPOINT, DODGE_RESTORE_HEADING, DODGE_CENTER_SETTLE,
    DODGE_VERIFY_FORWARD, DODGE_BLOCKED,
    DODGE_BACKUP, DODGE_TURN_RIGHT, DODGE_WAIT_RANGE,
    DODGE_OBSTACLE_STOP, DODGE_STOP_AFTER_BACKUP, DODGE_STOP_AFTER_TURN
  };
  enum DodgePhase : uint8_t {
    DODGE_LOOK_LEFT_SETTLE, DODGE_LOOK_LEFT_WAIT,
    DODGE_LOOK_RIGHT_SETTLE, DODGE_LOOK_RIGHT_WAIT,
    DODGE_EXECUTE_WAYPOINT, DODGE_ALIGN_TARGET, DODGE_VERIFY_PATH
  };
  enum UltrasonicPhase : uint8_t { ULTRA_IDLE, ULTRA_TRIGGER_LOW, ULTRA_TRIGGER_HIGH, ULTRA_WAIT_ECHO };

  struct Pose2D { float xCm; float yCm; float thetaRad; }; // theta=0 faces +Y; positive is clockwise/right
  Pose2D robotPose = {0.0f, 0.0f, 0.0f};
  AutoState autoState = STATE_IDLE;
  DodgePhase dodgePhase = DODGE_LOOK_LEFT_SETTLE;
  UltrasonicPhase ultrasonicPhase = ULTRA_IDLE;
  uint32_t autoStateSinceMs = 0;
  uint32_t dodgePhaseSinceMs = 0;
  uint32_t lastNavUpdateMs = 0;
  uint32_t lastAutoUltrasonicSeq = 0;
  uint32_t lastFollowUltrasonicSeq = 0;
  uint32_t lastUltrasonicStartMs = 0;
  uint32_t triggerHighSinceUs = 0;
  uint32_t echoWaitSinceUs = 0;
  uint32_t ultrasonicSequence = 0;
  uint32_t ultrasonicSampleMs = 0;
  long ultrasonicDistanceCm = 400;
  bool ultrasonicSampleValid = false;
  bool ultrasonicPingBusy = false;
  volatile uint32_t echoRiseUs = 0;
  volatile uint32_t echoWidthUs = 0;
  volatile bool echoCaptureReady = false;
  portMUX_TYPE echoMux = portMUX_INITIALIZER_UNLOCKED;
  uint32_t previousLeftTicks = 0;
  uint32_t previousRightTicks = 0;
  int navTargetLeftPwm = 0, navTargetRightPwm = 0;
  int navAppliedLeftPwm = 0, navAppliedRightPwm = 0;
  float dodgeLeftClearanceCm = 0.0f, dodgeRightClearanceCm = 0.0f;
  float waypointXcm = 0.0f, waypointYcm = 0.0f;
  bool waypointActive = false;
  // AUTO follows two axis-aligned legs: first Y (forward/back), then X (left/right).
  uint8_t navigationLeg = 0;             // 0 = Y leg, 1 = X leg
  float navigationStartXcm = 0.0f;
  bool autoProgressWatchActive = false;
  uint32_t autoProgressSinceMs = 0;
  float autoProgressXcm = 0.0f;
  float autoProgressYcm = 0.0f;
  float autoProgressHeadingRad = 0.0f;
  uint8_t dodgeAttempts = 0;
  ContinuousDodgePhase continuousDodgePhase = DODGE_CRUISE;
  uint32_t lastDodgeUpdateMs = 0;
  uint32_t lastDodgeUltrasonicSeq = 0;
  uint32_t continuousDodgePhaseSinceMs = 0;
  uint32_t dodgeRetryAfterMs = 0;
  bool dodgeTimedMotionStarted = false;
  bool dodgeTurnLeft = true;
  float dodgeModeLeftClearanceCm = 0.0f;
  float dodgeModeRightClearanceCm = 0.0f;
  float dodgeModeWaypointXcm = 0.0f;
  float dodgeModeWaypointYcm = 0.0f;
  float dodgeModeReturnHeadingRad = 0.0f;
  long fDist[FOLLOW_COUNT];
  unsigned long fTime[FOLLOW_COUNT];
  int fIdx = 3;
  int fDir = 1;
  unsigned long fLastTick = 0;

  char statusBuf[80] = "Manual mode";  // message sent to the phone

  // Servo calibration (0 deg, then centre)
  volatile bool calibRequested = false;
  int calibStage = 0;                  // 0 = idle, 1 = going to 0, 2 = going to centre
  unsigned long calibStart = 0;

  // Embedded HTML Dashboard
  const char HTML_PAGE[] PROGMEM = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
    <title>ESP32 Car Control</title>
    <style>
      * { -webkit-tap-highlight-color: transparent; }
      body { font-family: Arial; text-align: center; background: #1a1a1a; color: white; margin:0; padding:10px; user-select: none; -webkit-user-select: none; -webkit-touch-callout: none; }
      h2 { margin: 10px 0; }
      .badge { display: inline-block; padding: 8px 16px; border-radius: 20px; font-weight: bold; margin-bottom: 10px; }
      .manual { background: #2196F3; }
      .auto { background: #4CAF50; }
      .follow { background: #8e24aa; }
      .dodge { background: #ef6c00; }
      .btn { display: inline-block; padding: 20px 10px; font-size: 18px; border: none; border-radius: 12px; margin: 8px; color: white; font-weight: bold; touch-action: none; }
      .wide { width: 80%; max-width: 300px; }
      .half { width: 36%; max-width: 140px; }
      .btn-mode { background: #ff9800; width: 21%; max-width: 90px; padding: 16px 2px; font-size: 14px; margin: 4px 3px; touch-action: manipulation; }
      .btn-gas { background: #4CAF50; }
      .btn-gas:active { background: #2E7D32; }
      .btn-brake { background: #f44336; }
      .btn-brake:active { background: #c62828; }
      .btn-steer { background: #607D8B; }
      .btn-steer:active { background: #37474F; }
      .btn-estop { background: #d50000; font-size: 22px; padding: 22px 10px; border: 4px solid #fff; }
      .btn-estop.resume { background: #00897b; }
      .telemetry { background: #2d2d2d; padding: 15px; border-radius: 10px; margin: 15px auto; width: 80%; max-width: 300px; }
      #status { color: #ff5252; min-height: 20px; font-size: 14px; }
      .prompt { display: none; margin: 8px auto; padding: 12px; width: 85%; max-width: 320px; border-radius: 10px; font-weight: bold; }
      .prompt.info { background: #2d2d2d; }
      .prompt.ask { background: #f9a825; color: #000; }
      .prompt.stuck { background: #d50000; }
      .help { display: none; margin: 8px auto; padding: 10px; width: 85%; max-width: 320px; border-radius: 10px; background: #5d4037; font-size: 13px; text-align: left; }
      .row { display: flex; justify-content: space-between; align-items: center; margin: 8px 0 2px; font-size: 14px; }
      input[type=range] { width: 100%; }
      .mini { background: #455a64; border: none; color: white; border-radius: 8px; padding: 10px 12px; font-weight: bold; margin: 6px 4px 0; touch-action: manipulation; }
      .bar { height: 12px; background: #444; border-radius: 6px; overflow: hidden; margin: 4px 0 8px; }
      .bar > div { height: 100%; width: 0; transition: width 0.1s; }
    </style>
  </head>
  <body>
    <h2>ESP32 Smart Car</h2>
    <div id="modeBadge" class="badge manual">MANUAL MODE</div>
    <br>
    <button id="estopBtn" class="btn btn-estop wide">EMERGENCY STOP</button>
    <div id="status"></div>
    <div id="prompt" class="prompt info"></div>
    <div>
      <button id="m_manual" class="btn btn-mode" onclick="setMode('manual')">MANUAL</button>
      <button id="m_auto" class="btn btn-mode" onclick="setMode('auto')">AUTO</button>
      <button id="m_follow" class="btn btn-mode" onclick="setMode('follow')">FOLLOW</button>
      <button id="m_dodge" class="btn btn-mode" onclick="setMode('dodge')">DODGE</button>
    </div>

    <div id="targetPanel" class="telemetry" style="display:none">
      <b>AUTO DESTINATION (centimetres)</b>
      <div style="display:flex;gap:8px;margin-top:10px">
        <label style="flex:1">X <input id="targetX" type="number" step="1" value="0" style="width:90%"></label>
        <label style="flex:1">Y <input id="targetY" type="number" step="1" value="100" style="width:90%"></label>
      </div>
      <button class="mini" onclick="sendTarget()">NAVIGATE TO X,Y</button>
      <div id="targetResult" style="margin-top:8px;font-size:13px"></div>
    </div>

    <div class="telemetry manualOnly">
      <div>Phone tilt: <b id="tiltVal">0</b>&deg; <span id="tiltSrc" style="font-size:12px;color:#aaa"></span></div>
      <div style="margin-top:8px">Left wheels: <b id="lw">0</b></div>
      <div class="bar"><div id="lbar"></div></div>
      <div>Right wheels: <b id="rw">0</b></div>
      <div class="bar"><div id="rbar"></div></div>
      <button class="mini" id="centerBtn">SET CENTER</button>
      <button class="mini" id="invertBtn">REVERSE TILT: OFF</button>
    </div>
    <div id="tiltHelp" class="help"></div>

    <div id="steerRow">
      <button id="left" class="btn btn-steer half">&#9664; LEFT</button>
      <button id="right" class="btn btn-steer half">RIGHT &#9654;</button>
    </div>

    <div class="manualOnly">
      <button id="gas" class="btn btn-gas wide">ACCELERATE</button>
      <br>
      <button id="brake" class="btn btn-brake wide">BRAKE / REVERSE</button>
    </div>

    <div class="telemetry manualOnly">
      <b>Tilt calibration</b>
      <div class="row"><span>Inner wheel slow-down</span><span><b id="iVal">2.0</b> %/deg</span></div>
      <input type="range" id="iSl" min="0" max="5" step="0.1" value="2.0">
      <div class="row"><span>Outer wheel speed-up</span><span><b id="oVal">0.8</b> %/deg</span></div>
      <input type="range" id="oSl" min="0" max="3" step="0.1" value="0.8">
      <div class="row"><span>Dead zone</span><span><b id="dVal">4</b>&deg;</span></div>
      <input type="range" id="dSl" min="0" max="15" step="1" value="4">
    </div>

    <script>
      let currentMode = "manual";
      let estopOn = false;
      let asking = false;
      let st = {g:0, b:0, l:0, r:0};
      let tilt = 0, lastSentTilt = 0;
      let inFlight = false, dirty = false, lastPush = 0;
      let pollBusy = false;

      // ---------- tilt sensing ----------
      let tiltInvert = false, tiltOffset = 0, tiltSmooth = 0, tiltEvents = 0, tiltSource = '';
      try { tiltInvert = localStorage.getItem('tiltInvert') === '1'; } catch (e) {}

      function updateInvertLabel() {
        document.getElementById('invertBtn').innerText = 'REVERSE TILT: ' + (tiltInvert ? 'ON' : 'OFF');
      }
      updateInvertLabel();

      function screenAngle() {
        if (screen.orientation && typeof screen.orientation.angle === 'number') return screen.orientation.angle;
        if (typeof window.orientation === 'number') return window.orientation;
        return 0;
      }

      function applyTilt(rawDeg) {
        tiltSmooth = tiltSmooth * 0.6 + rawDeg * 0.4;
        let v = Math.round((tiltSmooth - tiltOffset) * (tiltInvert ? -1 : 1));
        tilt = Math.max(-90, Math.min(90, v));
        document.getElementById('tiltVal').innerText = tilt;
        document.getElementById('tiltSrc').innerText = '(' + tiltSource + ')';
        if (currentMode === 'manual' && tilt !== lastSentTilt && Date.now() - lastPush > 80) push();
      }

      // Main method: gravity vector -> left/right tilt of the screen (works held flat or upright)
      function handleMotion(e) {
        const a = e.accelerationIncludingGravity;
        if (!a || a.x === null || a.y === null || a.z === null) return;
        const mag = Math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        if (mag < 1) return;
        tiltEvents++; tiltSource = 'motion';
        const ang = ((screenAngle() % 360) + 360) % 360;
        let sx; // component along the screen's right edge
        if (ang === 0) sx = a.x;
        else if (ang === 90) sx = -a.y;
        else if (ang === 180) sx = -a.x;
        else sx = a.y;
        const roll = -Math.asin(Math.max(-1, Math.min(1, sx / mag))) * 180 / Math.PI;
        applyTilt(roll);
      }

      // Backup method: gyro orientation angles
      function handleOrientation(e) {
        if (tiltSource === 'motion') return;
        if (e.gamma === null && e.beta === null) return;
        tiltEvents++; tiltSource = 'orientation';
        const ang = ((screenAngle() % 360) + 360) % 360;
        let r;
        if (ang === 0) r = e.gamma;
        else if (ang === 180) r = -e.gamma;
        else if (ang === 90) r = e.beta;
        else r = -e.beta;
        applyTilt(r || 0);
      }

      window.addEventListener('devicemotion', handleMotion);
      window.addEventListener('deviceorientation', handleOrientation);

      setTimeout(() => {
        const help = document.getElementById('tiltHelp');
        if (tiltEvents > 0) { help.style.display = 'none'; return; }
        help.style.display = 'block';
        help.innerText = window.isSecureContext
          ? 'Tilt sensor gave no data. In Chrome: Settings > Site settings > Motion sensors = Allowed, then reload.'
          : 'Chrome blocks tilt on http pages. One-time fix: open chrome://flags, search "Insecure origins treated as secure", set Enabled, add http://192.168.4.1, tap Relaunch, then reconnect to the car Wi-Fi.';
      }, 2500);

      document.getElementById('centerBtn').addEventListener('click', () => { tiltOffset = tiltSmooth; });
      document.getElementById('invertBtn').addEventListener('click', () => {
        tiltInvert = !tiltInvert;
        try { localStorage.setItem('tiltInvert', tiltInvert ? '1' : '0'); } catch (e) {}
        updateInvertLabel();
      });

      // ---------- wheel speed display ----------
      function showWheels(l, r) {
        document.getElementById('lw').innerText = l;
        document.getElementById('rw').innerText = r;
        const lb = document.getElementById('lbar');
        const rb = document.getElementById('rbar');
        lb.style.width = (Math.min(255, Math.abs(l)) / 255 * 100) + '%';
        rb.style.width = (Math.min(255, Math.abs(r)) / 255 * 100) + '%';
        lb.style.background = l >= 0 ? '#4CAF50' : '#f44336';
        rb.style.background = r >= 0 ? '#4CAF50' : '#f44336';
      }

      // ---------- controls ----------
      function anyHeld() { return st.g || st.b || st.l || st.r; }

      // One small request at a time, always carrying the latest state
      function push() {
        if (estopOn) return;
        if (inFlight) { dirty = true; return; }
        inFlight = true; dirty = false;
        lastPush = Date.now(); lastSentTilt = tilt;
        fetch(`/ctl?g=${st.g}&b=${st.b}&l=${st.l}&r=${st.r}&t=${tilt}`, {cache: 'no-store'})
          .then(r => r.text())
          .then(t => {
            const p = t.split(',');
            if (p.length === 2) showWheels(Number(p[0]), Number(p[1]));
          })
          .catch(() => {})
          .finally(() => { inFlight = false; if (dirty) push(); });
      }

      function updateSteerRow() {
        document.getElementById('steerRow').style.display = (currentMode === 'manual' || asking) ? 'block' : 'none';
      }

      function showPrompt(code, text) {
        const p = document.getElementById('prompt');
        p.innerText = text;
        p.className = code === 'ASK' ? 'prompt ask' : (code === 'STUCK' ? 'prompt stuck' : 'prompt info');
        p.style.display = text ? 'block' : 'none';
      }

      // In auto mode, LEFT / RIGHT answer the car's question
      function choose(dir) {
        asking = false;
        updateSteerRow();
        fetch('/choose?dir=' + dir, {cache: 'no-store'}).catch(() => {});
      }

      function bindHold(id, key) {
        const el = document.getElementById(id);
        const press = e => {
          e.preventDefault();
          if (currentMode !== 'manual') {
            if (asking && (key === 'l' || key === 'r')) choose(key === 'l' ? 'L' : 'R');
            return;
          }
          st[key] = 1; push();
        };
        const release = e => {
          e.preventDefault();
          if (currentMode !== 'manual') return;
          st[key] = 0; push();
        };
        el.addEventListener('pointerdown', press);
        el.addEventListener('pointerup', release);
        el.addEventListener('pointercancel', release);
        el.addEventListener('pointerleave', release);
        el.addEventListener('contextmenu', e => e.preventDefault());
      }
      bindHold('gas', 'g');
      bindHold('brake', 'b');
      bindHold('left', 'l');
      bindHold('right', 'r');

      // Heartbeat while any control is held (car stops if this goes silent)
      setInterval(() => { if (anyHeld()) push(); }, 100);

      // Idle refresh: keeps tilt steering and the wheel bars live while coasting
      setInterval(() => { if (!anyHeld() && currentMode === 'manual') push(); }, 400);

      // Poll autonomous modes for live state, range, and any intervention prompt.
      setInterval(() => {
        if (currentMode === 'manual' || pollBusy) return;
        pollBusy = true;
        fetch('/status', {cache: 'no-store'})
          .then(r => r.text())
          .then(t => {
            const i = t.indexOf('|');
            const code = t.substring(0, i);
            const text = t.substring(i + 1);
            // Reflect the autonomous sub-mode in the dashboard badge as well as on OLED.
            const badge = document.getElementById('modeBadge');
            if (code === 'DODGE_MODE') {
              badge.innerText = 'DODGE MODE';
              badge.className = 'badge dodge';
            } else if (currentMode === 'auto') {
              badge.innerText = 'AUTO MODE';
              badge.className = 'badge auto';
            } else if (currentMode === 'dodge') {
              badge.innerText = 'DODGE MODE';
              badge.className = 'badge dodge';
            }
            const nowAsking = (code === 'ASK');
            if (nowAsking && !asking && navigator.vibrate) navigator.vibrate([200, 100, 200]);
            asking = nowAsking;
            showPrompt(code, text);
            updateSteerRow();
          })
          .catch(() => {})
          .finally(() => { pollBusy = false; });
      }, 500);

      function setEstopUI(on) {
        estopOn = on;
        const b = document.getElementById('estopBtn');
        b.innerText = on ? 'RESUME' : 'EMERGENCY STOP';
        b.className = on ? 'btn btn-estop resume wide' : 'btn btn-estop wide';
        document.getElementById('status').innerText = on ? 'CAR STOPPED' : '';
      }

      function toggleEstop() {
        const want = estopOn ? 0 : 1;
        st = {g:0, b:0, l:0, r:0};
        fetch('/estop?state=' + want, {cache: 'no-store'})
          .then(r => r.text())
          .then(t => setEstopUI(t === '1'))
          .catch(() => { document.getElementById('status').innerText = 'No connection to car! Use the power switch.'; });
      }
      document.getElementById('estopBtn').addEventListener('pointerdown', e => { e.preventDefault(); toggleEstop(); });

      // ---------- modes ----------
      function setModeUI(mode) {
        currentMode = mode;
        asking = false;
        const badge = document.getElementById('modeBadge');
        document.querySelectorAll('.manualOnly').forEach(el => {
          el.style.display = (mode === 'manual') ? 'block' : 'none';
        });
        if (mode === 'auto') {
          badge.innerText = 'AUTO MODE';
          badge.className = 'badge auto';
        } else if (mode === 'follow') {
          badge.innerText = 'FOLLOW MODE';
          badge.className = 'badge follow';
        } else if (mode === 'dodge') {
          badge.innerText = 'DODGE MODE';
          badge.className = 'badge dodge';
        } else {
          badge.innerText = 'MANUAL MODE';
          badge.className = 'badge manual';
          showPrompt('OK', '');
        }
        ['manual', 'auto', 'follow', 'dodge'].forEach(m => {
          document.getElementById('m_' + m).style.outline = (m === mode) ? '3px solid #fff' : 'none';
        });
        document.getElementById('targetPanel').style.display = (mode === 'auto') ? 'block' : 'none';
        updateSteerRow();
      }

      function sendTarget() {
        const x = document.getElementById('targetX').value;
        const y = document.getElementById('targetY').value;
        fetch(`/target?x=${encodeURIComponent(x)}&y=${encodeURIComponent(y)}`, {cache: 'no-store'})
          .then(async r => { const t = await r.text(); if (!r.ok) throw new Error(t); return t; })
          .then(t => { document.getElementById('targetResult').innerText = t; })
          .catch(e => { document.getElementById('targetResult').innerText = e.message || 'Could not send destination'; });
      }

      function setMode(m) {
        st = {g:0, b:0, l:0, r:0};
        fetch('/mode?m=' + m, {cache: 'no-store'})
          .then(res => res.text())
          .then(mode => setModeUI(mode))
          .catch(() => {});
      }

      // Sync with the car when the page loads (e.g. after a refresh)
      fetch('/mode', {cache: 'no-store'})
        .then(r => r.text())
        .then(m => setModeUI(m))
        .catch(() => {});

      // ---------- live calibration sliders ----------
      let calTimer = null;
      function sendCal() {
        const i = document.getElementById('iSl').value;
        const o = document.getElementById('oSl').value;
        const d = document.getElementById('dSl').value;
        document.getElementById('iVal').innerText = Number(i).toFixed(1);
        document.getElementById('oVal').innerText = Number(o).toFixed(1);
        document.getElementById('dVal').innerText = d;
        clearTimeout(calTimer);
        calTimer = setTimeout(() => {
          fetch(`/cal?i=${i}&o=${o}&d=${d}`, {cache: 'no-store'}).catch(() => {});
        }, 150);
      }
      ['iSl', 'oSl', 'dSl'].forEach(id => document.getElementById(id).addEventListener('input', sendCal));

      // Load the car's current calibration values into the sliders
      fetch('/cal', {cache: 'no-store'})
        .then(r => r.text())
        .then(t => {
          const p = t.split(',');
          if (p.length === 3) {
            document.getElementById('iSl').value = p[0];
            document.getElementById('oSl').value = p[1];
            document.getElementById('dSl').value = p[2];
            document.getElementById('iVal').innerText = p[0];
            document.getElementById('oVal').innerText = p[1];
            document.getElementById('dVal').innerText = p[2];
          }
        })
        .catch(() => {});
    </script>
  </body>
  </html>
  )rawliteral";

  // --- MOTOR CONTROL ---

  void applyMotorSpeeds(int leftSpeed, int rightSpeed) {
    dispLeft = constrain(leftSpeed, -255, 255);
    dispRight = constrain(rightSpeed, -255, 255);

    // Left motor direction
    if (leftSpeed > 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    } else if (leftSpeed < 0) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      leftSpeed = -leftSpeed;
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }

    // Right motor direction
    if (rightSpeed > 0) {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    } else if (rightSpeed < 0) {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      rightSpeed = -rightSpeed;
    } else {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }

    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    ledcWrite(ENA, leftSpeed);
    ledcWrite(ENB, rightSpeed);
  }

  void updatePhysicsAndMotors() {
    // 1. Base speed (acceleration and braking)
    if (isAccelerating) {
      if (currentSpeed < 0) currentSpeed += BRAKE_RATE;
      else currentSpeed += ACCEL_RATE;
    } else if (isBraking) {
      currentSpeed -= BRAKE_RATE;
    } else {
      if (currentSpeed > 0) {
        currentSpeed -= COAST_RATE;
        if (currentSpeed < 0) currentSpeed = 0;
      } else if (currentSpeed < 0) {
        currentSpeed += COAST_RATE;
        if (currentSpeed > 0) currentSpeed = 0;
      }
    }

    currentSpeed = constrain(currentSpeed, -MAX_SPEED, MAX_SPEED);

    // 2. Gradual button steering (builds up while held, eases back when released)
    float steerTarget = 0.0;
    if (steerRightBtn && !steerLeftBtn) steerTarget = BUTTON_STEER_MAX;
    else if (steerLeftBtn && !steerRightBtn) steerTarget = -BUTTON_STEER_MAX;

    float rate = (steerTarget == 0.0) ? STEER_RETURN : STEER_RAMP;
    if (buttonSteer < steerTarget) {
      buttonSteer += rate;
      if (buttonSteer > steerTarget) buttonSteer = steerTarget;
    } else if (buttonSteer > steerTarget) {
      buttonSteer -= rate;
      if (buttonSteer < steerTarget) buttonSteer = steerTarget;
    }

    // 3. Steering angle from phone tilt (+ buttons), calibrated per degree
    float maxEff = TILT_MAX_DEG - tiltDeadzoneDeg;
    float t = tiltAngle * TILT_SIGN;
    float tiltEff = 0.0;
    if (t > tiltDeadzoneDeg) tiltEff = t - tiltDeadzoneDeg;
    else if (t < -tiltDeadzoneDeg) tiltEff = t + tiltDeadzoneDeg;

    float steerDeg = constrain(tiltEff + BUTTON_STEER_SIGN * buttonSteer * maxEff, -maxEff, maxEff);
    float absDeg = fabs(steerDeg);

    float slow = innerSlowPerDeg * absDeg;
    float innerFactor = 1.0 - slow;
    if (innerFactor < 0.0) innerFactor = 0.0;
    float outerFactor = 1.0 + outerFastPerDeg * absDeg;

    float leftSpeedF = currentSpeed;
    float rightSpeedF = currentSpeed;

    if (steerDeg > 0) {         // steering right: right side is the inner side
      rightSpeedF = currentSpeed * innerFactor;
      leftSpeedF = currentSpeed * outerFactor;
    } else if (steerDeg < 0) {  // steering left: left side is the inner side
      leftSpeedF = currentSpeed * innerFactor;
      rightSpeedF = currentSpeed * outerFactor;
    }

    applyMotorSpeeds((int)leftSpeedF, (int)rightSpeedF);
  }

  // --- SERVO ---

  void servoWrite(int angle) {
    angle = constrain(angle, 0, 180);
    int pulseUs = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
    uint32_t duty = (uint32_t)pulseUs * 65535UL / 20000UL;
    ledcWrite(SERVO_PIN, duty);
  }

  void startServoCalibration() {
    calibStage = 1;
    calibStart = millis();
    servoWrite(0);              // home to 0 degrees first
  }

  void serviceCalibration() {
    if (calibRequested) {
      calibRequested = false;
      startServoCalibration();
    }
    if (calibStage == 1 && millis() - calibStart >= 700) {
      servoWrite(SERVO_CENTER); // then to centre (90)
      calibStage = 2;
      calibStart = millis();
    } else if (calibStage == 2 && millis() - calibStart >= 500) {
      calibStage = 0;           // done
    }
  }

  // --- ASYNCHRONOUS ULTRASONIC SENSOR ---
  // The 10 us trigger and echo pulse are serviced without pulseIn() or delay().
  void IRAM_ATTR onEchoChange() {
    const uint32_t nowUs = micros();
    const bool high = digitalRead(ECHO_PIN) == HIGH;
    portENTER_CRITICAL_ISR(&echoMux);
    if (high) {
      echoRiseUs = nowUs;
    } else if (echoRiseUs != 0) {
      echoWidthUs = nowUs - echoRiseUs;
      echoRiseUs = 0;
      echoCaptureReady = true;
    }
    portEXIT_CRITICAL_ISR(&echoMux);
  }

  void publishUltrasonicSample(uint32_t widthUs, bool valid) {
    ultrasonicSampleValid = valid && widthUs >= 100 && widthUs <= ULTRASONIC_TIMEOUT_US;
    ultrasonicDistanceCm = ultrasonicSampleValid ? (long)(widthUs * 0.0343f / 2.0f) : 400;
    ultrasonicSampleMs = millis();
    ultrasonicSequence++;
  }

  void serviceUltrasonic() {
    const uint32_t nowUs = micros();
    const uint32_t nowMs = millis();
    if (ultrasonicPhase == ULTRA_TRIGGER_LOW && (uint32_t)(nowUs - triggerHighSinceUs) >= 2) {
      digitalWrite(TRIG_PIN, HIGH);
      triggerHighSinceUs = nowUs;
      ultrasonicPhase = ULTRA_TRIGGER_HIGH;
    } else if (ultrasonicPhase == ULTRA_TRIGGER_HIGH && (uint32_t)(nowUs - triggerHighSinceUs) >= 10) {
      digitalWrite(TRIG_PIN, LOW);
      ultrasonicPhase = ULTRA_WAIT_ECHO;
      echoWaitSinceUs = nowUs;
      lastUltrasonicStartMs = nowMs;
    }

    bool ready;
    uint32_t widthUs = 0;
    portENTER_CRITICAL(&echoMux);
    ready = echoCaptureReady;
    if (ready) {
      widthUs = echoWidthUs;
      echoCaptureReady = false;
    }
    portEXIT_CRITICAL(&echoMux);

    if (ultrasonicPhase == ULTRA_WAIT_ECHO && ready) {
      ultrasonicPhase = ULTRA_IDLE;
      ultrasonicPingBusy = false;
      publishUltrasonicSample(widthUs, true);
    } else if (ultrasonicPhase == ULTRA_WAIT_ECHO &&
              (uint32_t)(nowUs - echoWaitSinceUs) >= ULTRASONIC_TIMEOUT_US) {
      ultrasonicPhase = ULTRA_IDLE;
      ultrasonicPingBusy = false;
      portENTER_CRITICAL(&echoMux);
      echoRiseUs = 0;
      echoCaptureReady = false;
      portEXIT_CRITICAL(&echoMux);
      publishUltrasonicSample(0, false); // no echo means no measured object within range
    }

    if (ultrasonicPhase == ULTRA_IDLE && !ultrasonicPingBusy &&
        (uint32_t)(nowMs - lastUltrasonicStartMs) >= ULTRASONIC_PERIOD_MS) {
      digitalWrite(TRIG_PIN, LOW);
      triggerHighSinceUs = nowUs;
      ultrasonicPhase = ULTRA_TRIGGER_LOW;
      ultrasonicPingBusy = true;
    }
  }

  // --- DUAL-ENCODER ODOMETRY ---
  void IRAM_ATTR onLeftEncoderPulse() {
    const uint32_t nowUs = micros();
    portENTER_CRITICAL_ISR(&encoderMux);
    if ((uint32_t)(nowUs - leftEncoderLastUs) >= ENCODER_MIN_EDGE_US) {
      leftEncoderLastUs = nowUs;
      leftEncoderTicks++;
    }
    portEXIT_CRITICAL_ISR(&encoderMux);
  }

  void IRAM_ATTR onRightEncoderPulse() {
    const uint32_t nowUs = micros();
    portENTER_CRITICAL_ISR(&encoderMux);
    if ((uint32_t)(nowUs - rightEncoderLastUs) >= ENCODER_MIN_EDGE_US) {
      rightEncoderLastUs = nowUs;
      rightEncoderTicks++;
    }
    portEXIT_CRITICAL_ISR(&encoderMux);
  }

  float normalizeAngleRad(float angle) {
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
  }

  void resetOdometry() {
    portENTER_CRITICAL(&encoderMux);
    previousLeftTicks = leftEncoderTicks;
    previousRightTicks = rightEncoderTicks;
    portEXIT_CRITICAL(&encoderMux);
    robotPose = {0.0f, 0.0f, 0.0f}; // boot origin, facing positive Y
  }

  void updateOdometry() {
    uint32_t leftNow, rightNow;
    portENTER_CRITICAL(&encoderMux);
    leftNow = leftEncoderTicks;
    rightNow = rightEncoderTicks;
    portEXIT_CRITICAL(&encoderMux);

    // Unsigned subtraction is wrap-safe for the monotonically increasing edge counters.
    const int32_t leftEdges = (int32_t)(leftNow - previousLeftTicks);
    const int32_t rightEdges = (int32_t)(rightNow - previousRightTicks);
    previousLeftTicks = leftNow;
    previousRightTicks = rightNow;

    // Single-channel encoders do not report direction. Infer each sign from the current
    // physical motor command, mapping the manual channel values through the wiring swap.
    int physicalLeftCommand = navAppliedLeftPwm;
    int physicalRightCommand = navAppliedRightPwm;
    if (carMode == MODE_MANUAL) {
      if (CHANNELS_SWAPPED) { physicalLeftCommand = dispRight; physicalRightCommand = dispLeft; }
      else { physicalLeftCommand = dispLeft; physicalRightCommand = dispRight; }
    }
    const float leftSign = (physicalLeftCommand > 0) ? 1.0f : ((physicalLeftCommand < 0) ? -1.0f : 0.0f);
    const float rightSign = (physicalRightCommand > 0) ? 1.0f : ((physicalRightCommand < 0) ? -1.0f : 0.0f);
    const float cmPerEdge = WHEEL_CIRCUMFERENCE_CM / (float)ENCODER_PULSES_PER_REV;
    const float dLeft = (float)leftEdges * cmPerEdge * leftSign;
    const float dRight = (float)rightEdges * cmPerEdge * rightSign;
    const float dCenter = 0.5f * (dLeft + dRight);
    const float dTheta = (dLeft - dRight) / WHEEL_TRACK_CM; // clockwise-positive heading from +Y
    const float midTheta = robotPose.thetaRad + 0.5f * dTheta;

    // Exact differential-drive arc integration using the midpoint heading.
    robotPose.xCm += dCenter * sinf(midTheta);
    robotPose.yCm += dCenter * cosf(midTheta);
    robotPose.thetaRad = normalizeAngleRad(robotPose.thetaRad + dTheta);
  }

  // Auto motor targets are ramped in physical left/right coordinates, then mapped to channels.
  void setNavWheelTargets(int leftPwm, int rightPwm) {
    navTargetLeftPwm = constrain(leftPwm, -NAV_MAX_PWM, NAV_MAX_PWM);
    navTargetRightPwm = constrain(rightPwm, -NAV_MAX_PWM, NAV_MAX_PWM);
  }

  int rampToward(int current, int target) {
    if (current < target) return min(current + NAV_PWM_RAMP_STEP, target);
    if (current > target) return max(current - NAV_PWM_RAMP_STEP, target);
    return current;
  }

  void serviceNavMotorRamp() {
    navAppliedLeftPwm = rampToward(navAppliedLeftPwm, navTargetLeftPwm);
    navAppliedRightPwm = rampToward(navAppliedRightPwm, navTargetRightPwm);
    if (CHANNELS_SWAPPED) applyMotorSpeeds(navAppliedRightPwm, navAppliedLeftPwm);
    else applyMotorSpeeds(navAppliedLeftPwm, navAppliedRightPwm);
  }

  void stopNavImmediately() {
    navTargetLeftPwm = navTargetRightPwm = 0;
    navAppliedLeftPwm = navAppliedRightPwm = 0;
    applyMotorSpeeds(0, 0);
  }

  void setStatus(const char text[]) {
    strncpy(statusBuf, text, sizeof(statusBuf) - 1);
    statusBuf[sizeof(statusBuf) - 1] = '\0';
  }

  void setAutoState(uint8_t state, const char text[]) {
    autoState = (AutoState)state;
    autoStateSinceMs = millis();
    autoProgressWatchActive = false; // restart the movement watchdog across navigation/avoid transitions
    if (text && text[0]) setStatus(text);
  }

  void failAutoStopped(const char text[]);

  bool autoEncoderProgressStalled(uint32_t now) {
    const bool commanded = navTargetLeftPwm != 0 || navTargetRightPwm != 0 ||
                           navAppliedLeftPwm != 0 || navAppliedRightPwm != 0;
    if (!commanded) {
      autoProgressWatchActive = false;
      return false;
    }

    if (!autoProgressWatchActive) {
      autoProgressWatchActive = true;
      autoProgressSinceMs = now;
      autoProgressXcm = robotPose.xCm;
      autoProgressYcm = robotPose.yCm;
      autoProgressHeadingRad = robotPose.thetaRad;
      return false;
    }

    const float movedCm = hypotf(robotPose.xCm - autoProgressXcm, robotPose.yCm - autoProgressYcm);
    const float turnedDeg = fabsf(normalizeAngleRad(robotPose.thetaRad - autoProgressHeadingRad)) * 180.0f / PI;
    if (movedCm >= AUTO_ENCODER_PROGRESS_CM || turnedDeg >= AUTO_ENCODER_PROGRESS_DEG) {
      autoProgressSinceMs = now;
      autoProgressXcm = robotPose.xCm;
      autoProgressYcm = robotPose.yCm;
      autoProgressHeadingRad = robotPose.thetaRad;
    } else if (now - autoProgressSinceMs >= AUTO_NO_ENCODER_PROGRESS_MS) {
      failAutoStopped("AUTO stopped: no encoder movement; check GPIO 34/35, motor output, and wheel calibration");
      return true;
    }
    return false;
  }

  float navigationTargetX() { return navigationLeg == 0 ? navigationStartXcm : targetXcm; }
  float navigationTargetY() { return targetYcm; }

  float targetDistanceTo(float xCm, float yCm) {
    return hypotf(xCm - robotPose.xCm, yCm - robotPose.yCm);
  }

  float targetHeadingError(float xCm, float yCm) {
    const float dx = xCm - robotPose.xCm;
    const float dy = yCm - robotPose.yCm;
    const float desiredHeading = atan2f(dx, dy); // heading is measured from positive Y
    return normalizeAngleRad(desiredHeading - robotPose.thetaRad);
  }

  void driveTowardPoint(float xCm, float yCm, int maxPwm, bool allowPivot) {
    const float error = targetHeadingError(xCm, yCm);
    const float absError = fabsf(error);
    const float deadband = NAV_HEADING_DEADBAND_DEG * PI / 180.0f;
    if (allowPivot && absError > 0.65f) {
      const int turn = (error > 0.0f) ? NAV_TURN_PWM : -NAV_TURN_PWM;
      setNavWheelTargets(turn, -turn);
      return;
    }

    const int base = min(NAV_CRUISE_PWM, maxPwm);
    const int correction = (int)(NAV_HEADING_KP * error);
    int left = base + correction;
    int right = base - correction;
    if (absError < deadband) left = right = base;
    left = constrain(left, 0, maxPwm);
    right = constrain(right, 0, maxPwm);
    if (left > 0 && left < NAV_MIN_EFFECTIVE_PWM) left = NAV_MIN_EFFECTIVE_PWM;
    if (right > 0 && right < NAV_MIN_EFFECTIVE_PWM) right = NAV_MIN_EFFECTIVE_PWM;
    setNavWheelTargets(left, right);
  }

  // AUTO traverses signed world-axis legs instead of driving a diagonal vector:
  // Y is driven forward/reverse while facing +Y, then X is driven after a 90-degree pivot.
  void driveAutoAxisLeg(float desiredHeadingRad, bool reverse, float remainingCm) {
    const float error = normalizeAngleRad(desiredHeadingRad - robotPose.thetaRad);
    const float deadband = NAV_HEADING_DEADBAND_DEG * PI / 180.0f;
    if (fabsf(error) > deadband) {
      const int turn = (error > 0.0f) ? NAV_TURN_PWM : -NAV_TURN_PWM;
      setNavWheelTargets(turn, -turn);
      return;
    }

    int base = NAV_CRUISE_PWM;
    if (remainingCm < 35.0f) {
      const long boundedRemaining = constrain((long)remainingCm, (long)TARGET_TOLERANCE_CM, 35L);
      base = (int)map(boundedRemaining, (long)TARGET_TOLERANCE_CM, 35L,
                      NAV_MIN_EFFECTIVE_PWM, NAV_CRUISE_PWM);
    }
    const int correction = (int)(NAV_HEADING_KP * error);
    const int direction = reverse ? -1 : 1;
    int left = direction * base + correction;
    int right = direction * base - correction;
    left = constrain(left, -NAV_MAX_PWM, NAV_MAX_PWM);
    right = constrain(right, -NAV_MAX_PWM, NAV_MAX_PWM);
    if (left > 0 && left < NAV_MIN_EFFECTIVE_PWM) left = NAV_MIN_EFFECTIVE_PWM;
    if (left < 0 && left > -NAV_MIN_EFFECTIVE_PWM) left = -NAV_MIN_EFFECTIVE_PWM;
    if (right > 0 && right < NAV_MIN_EFFECTIVE_PWM) right = NAV_MIN_EFFECTIVE_PWM;
    if (right < 0 && right > -NAV_MIN_EFFECTIVE_PWM) right = -NAV_MIN_EFFECTIVE_PWM;
    setNavWheelTargets(left, right);
  }

  void failAutoStopped(const char text[]) {
    stopNavImmediately();
    waypointActive = false;
    setAutoState(STATE_IDLE, text);
  }

  void beginDodge() {
    if (dodgeAttempts >= MAX_DODGE_ATTEMPTS) {
      failAutoStopped("Dodge limit reached; stopped for safety");
      return;
    }
    dodgeAttempts++;
    stopNavImmediately(); // Step 1: halt before moving the sensor or selecting an escape
    dodgeLeftClearanceCm = dodgeRightClearanceCm = 0.0f;
    servoWrite(SERVO_LOOK_LEFT);
    setAutoState(STATE_AVOIDING, "Obstacle close: stopping and scanning sides");
    dodgePhase = DODGE_LOOK_LEFT_SETTLE;
    dodgePhaseSinceMs = millis();
  }

  void beginNavigation(float xCm, float yCm) {
    targetXcm = xCm;
    targetYcm = yCm;
    targetSet = true;
    waypointActive = false;
    navigationStartXcm = robotPose.xCm;
    navigationLeg = (fabsf(targetYcm - robotPose.yCm) <= TARGET_TOLERANCE_CM) ? 1 : 0;
    dodgeAttempts = 0;
    lastAutoUltrasonicSeq = ultrasonicSequence;
    stopNavImmediately();
    setAutoState(STATE_NAVIGATING, "Navigating to destination");
  }

  void resetAuto() {
    stopNavImmediately();
    targetSet = false;
    waypointActive = false;
    dodgeAttempts = 0;
    servoWrite(SERVO_CENTER);
    setAutoState(STATE_IDLE, "AUTO: enter destination X,Y in centimetres");
  }

  void runAutoMode() {
    if (calibStage != 0 || emergencyStop) {
      stopNavImmediately();
      return;
    }
    const uint32_t now = millis();
    if (now - lastNavUpdateMs < NAV_UPDATE_MS) return;
    lastNavUpdateMs = now;

    if (autoState == STATE_IDLE) {
      stopNavImmediately();
      return;
    }

    // A timed-out sensor is a fail-stop. An individual no-echo is treated as clear range.
    if (now - ultrasonicSampleMs > 500) {
      failAutoStopped("Range sensor timeout; stopped");
      return;
    }

    if (autoState == STATE_NAVIGATING) {
      if (ultrasonicSequence != lastAutoUltrasonicSeq) {
        lastAutoUltrasonicSeq = ultrasonicSequence;
        if (ultrasonicSampleValid && ultrasonicDistanceCm < SAFE_OBSTACLE_CM) {
          beginDodge();
          return;
        }
      }
      if (autoEncoderProgressStalled(now)) return;

      const float destX = waypointActive ? waypointXcm : navigationTargetX();
      const float destY = waypointActive ? waypointYcm : navigationTargetY();
      if (targetDistanceTo(destX, destY) <= TARGET_TOLERANCE_CM) {
        stopNavImmediately();
        if (waypointActive) {
          waypointActive = false;
          servoWrite(SERVO_CENTER);
          dodgePhase = DODGE_ALIGN_TARGET;
          dodgePhaseSinceMs = now;
          setAutoState(STATE_AVOIDING, "Bypass waypoint reached; realigning to destination");
        } else if (navigationLeg == 0) {
          // The Y leg is complete. Pivot to the signed X direction and finish the route.
          navigationLeg = 1;
          setStatus("Y leg complete; turning for X leg");
        } else {
          setAutoState(STATE_IDLE, "Destination reached");
          targetSet = false;
        }
        return;
      }
      if (waypointActive) {
        // Obstacle bypass waypoints may be diagonal; use the general vector controller here.
        driveTowardPoint(destX, destY, NAV_MAX_PWM, true);
      } else if (navigationLeg == 0) {
        // Negative Y is reverse travel; do not turn the car around to move backward.
        driveAutoAxisLeg(0.0f, targetYcm < robotPose.yCm, fabsf(targetYcm - robotPose.yCm));
      } else {
        // Positive X is a clockwise/right quarter turn; negative X is counterclockwise/left.
        const float xHeading = (targetXcm >= robotPose.xCm) ? (PI * 0.5f) : (-PI * 0.5f);
        driveAutoAxisLeg(xHeading, false, fabsf(targetXcm - robotPose.xCm));
      }
      return;
    }

    if (autoState != STATE_AVOIDING) return;

    switch (dodgePhase) {
      case DODGE_LOOK_LEFT_SETTLE:
        if (now - dodgePhaseSinceMs >= SERVO_SETTLE_MS) {
          lastAutoUltrasonicSeq = ultrasonicSequence;
          dodgePhase = DODGE_LOOK_LEFT_WAIT;
          dodgePhaseSinceMs = now;
        }
        break;

      case DODGE_LOOK_LEFT_WAIT:
        if (ultrasonicSequence != lastAutoUltrasonicSeq) {
          lastAutoUltrasonicSeq = ultrasonicSequence;
          dodgeLeftClearanceCm = ultrasonicSampleValid ? (float)ultrasonicDistanceCm : 400.0f;
          servoWrite(SERVO_LOOK_RIGHT);
          dodgePhase = DODGE_LOOK_RIGHT_SETTLE;
          dodgePhaseSinceMs = now;
        }
        break;

      case DODGE_LOOK_RIGHT_SETTLE:
        if (now - dodgePhaseSinceMs >= SERVO_SETTLE_MS) {
          lastAutoUltrasonicSeq = ultrasonicSequence;
          dodgePhase = DODGE_LOOK_RIGHT_WAIT;
          dodgePhaseSinceMs = now;
        }
        break;

      case DODGE_LOOK_RIGHT_WAIT:
        if (ultrasonicSequence != lastAutoUltrasonicSeq) {
          lastAutoUltrasonicSeq = ultrasonicSequence;
          dodgeRightClearanceCm = ultrasonicSampleValid ? (float)ultrasonicDistanceCm : 400.0f;
          const float bestClearance = max(dodgeLeftClearanceCm, dodgeRightClearanceCm);
          if (bestClearance < MIN_ESCAPE_CLEARANCE_CM) {
            failAutoStopped("No safe escape path measured; stopped");
            break;
          }
          // Heading is clockwise-positive: left is theta-45 deg, right is theta+45 deg.
          const float clearanceDifference = dodgeLeftClearanceCm - dodgeRightClearanceCm;
          const bool tied = fabsf(clearanceDifference) <= DODGE_CLEARANCE_TIE_CM;
          const bool escapeLeft = tied ? PREFER_LEFT_ON_TIE : (clearanceDifference > 0.0f);
          const float sign = escapeLeft ? -1.0f : 1.0f;
          const float escapeHeading = robotPose.thetaRad + sign * DODGE_ANGLE_DEG * PI / 180.0f;
          waypointXcm = robotPose.xCm + DODGE_WAYPOINT_CM * sinf(escapeHeading);
          waypointYcm = robotPose.yCm + DODGE_WAYPOINT_CM * cosf(escapeHeading);
          waypointActive = true;
          servoWrite(SERVO_CENTER);
          dodgePhase = DODGE_EXECUTE_WAYPOINT;
          dodgePhaseSinceMs = now;
          setStatus(escapeLeft ? "Dodge: taking left clearance waypoint" : "Dodge: taking right clearance waypoint");
        }
        break;

      case DODGE_EXECUTE_WAYPOINT:
        // Allow the servo to settle at centre before resuming. If an obstacle is now
        // critically close, stop and ask for intervention rather than stacking dodges.
        if (now - dodgePhaseSinceMs < SERVO_SETTLE_MS) {
          stopNavImmediately();
        } else if (ultrasonicSampleValid && ultrasonicDistanceCm < 12) {
          failAutoStopped("Obstacle remains critically close; stopped");
        } else if (targetDistanceTo(waypointXcm, waypointYcm) <= TARGET_TOLERANCE_CM) {
          stopNavImmediately();
          waypointActive = false;
          dodgePhase = DODGE_ALIGN_TARGET;
          dodgePhaseSinceMs = now;
          setStatus("Bypass waypoint reached; realigning to destination");
        } else {
          driveTowardPoint(waypointXcm, waypointYcm, NAV_CRUISE_PWM, true);
        }
        break;

      case DODGE_ALIGN_TARGET: {
        if (ultrasonicSampleValid && ultrasonicDistanceCm < 12) {
          failAutoStopped("Obstacle remains critically close; stopped");
          break;
        }
        const float resumeHeading = (navigationLeg == 0) ? 0.0f :
            ((targetXcm >= robotPose.xCm) ? (PI * 0.5f) : (-PI * 0.5f));
        const float error = normalizeAngleRad(resumeHeading - robotPose.thetaRad);
        if (fabsf(error) <= 10.0f * PI / 180.0f) {
          stopNavImmediately();
          servoWrite(SERVO_CENTER);
          lastAutoUltrasonicSeq = ultrasonicSequence;
          dodgePhase = DODGE_VERIFY_PATH;
          dodgePhaseSinceMs = now;
        } else {
          const int turn = (error > 0.0f) ? 80 : -80;
          setNavWheelTargets(turn, -turn);
        }
        break;
      }

      case DODGE_VERIFY_PATH:
        if (now - dodgePhaseSinceMs >= SERVO_SETTLE_MS && ultrasonicSequence != lastAutoUltrasonicSeq) {
          lastAutoUltrasonicSeq = ultrasonicSequence;
          if (ultrasonicSampleValid && ultrasonicDistanceCm < SAFE_OBSTACLE_CM) {
            if (dodgeAttempts < MAX_DODGE_ATTEMPTS) beginDodge();
            else failAutoStopped("Target path still blocked; stopped for safety");
          } else {
            dodgeAttempts = 0;
            setAutoState(STATE_NAVIGATING, "Path clear after bypass; resuming destination vector");
          }
        }
        break;
    }
  }

  void beginContinuousDodgeScan() {
    stopNavImmediately();
    servoWrite(SERVO_LOOK_LEFT);
    continuousDodgePhase = DODGE_SCAN_LEFT_SETTLE;
    continuousDodgePhaseSinceMs = millis();
    lastDodgeUltrasonicSeq = ultrasonicSequence;
    setStatus("Dodge: stopped; scanning left and right");
  }

  void resetDodgeMode() {
    stopNavImmediately();
    continuousDodgePhase = DODGE_CRUISE;
    continuousDodgePhaseSinceMs = millis();
    lastDodgeUpdateMs = 0;
    lastDodgeUltrasonicSeq = ultrasonicSequence;
    dodgeRetryAfterMs = 0;
    servoWrite(SERVO_CENTER);
    setStatus("Dodge mode: scanning ahead and avoiding obstacles");
  }

  void runDodgeMode() {
    if (calibStage != 0 || emergencyStop) {
      stopNavImmediately();
      return;
    }

    const uint32_t now = millis();
    if (now - lastDodgeUpdateMs < NAV_UPDATE_MS) return;
    lastDodgeUpdateMs = now;

    // Stop if the range sensor stops publishing. A fresh no-echo is treated as clear.
    if (now - ultrasonicSampleMs > 500) {
      stopNavImmediately();
      continuousDodgePhase = DODGE_WAIT_RANGE;
      lastDodgeUltrasonicSeq = ultrasonicSequence;
      servoWrite(SERVO_CENTER);
      setStatus("Dodge stopped: waiting for ultrasonic reading");
      return;
    }

    switch (continuousDodgePhase) {
      case DODGE_CRUISE:
        servoWrite(SERVO_CENTER);
        if (ultrasonicSequence != lastDodgeUltrasonicSeq) {
          lastDodgeUltrasonicSeq = ultrasonicSequence;
          // Match the supplied obstacle logic: react to a measured object under 30 cm.
          if (ultrasonicSampleValid && ultrasonicDistanceCm >= 15 && ultrasonicDistanceCm <= 25) {
            stopNavImmediately();
            continuousDodgePhase = DODGE_OBSTACLE_STOP;
            continuousDodgePhaseSinceMs = now;
            setStatus("Dodge: obstacle detected; stopping for 500 ms");
            break;
          }
        }
        setNavWheelTargets(150, 150);
        break;

      case DODGE_OBSTACLE_STOP:
        stopNavImmediately();
        if (now - continuousDodgePhaseSinceMs >= 500) {
          continuousDodgePhase = DODGE_BACKUP;
          dodgeTimedMotionStarted = false;
          setStatus("Dodge: reversing for 200 ms");
        }
        break;

      case DODGE_BACKUP:
        setNavWheelTargets(-200, -200);
        // Start timing after the existing PWM ramp reaches the requested reverse speed.
        if (!dodgeTimedMotionStarted && navAppliedLeftPwm <= -200 && navAppliedRightPwm <= -200) {
          dodgeTimedMotionStarted = true;
          continuousDodgePhaseSinceMs = now;
        } else if (dodgeTimedMotionStarted && now - continuousDodgePhaseSinceMs >= 200) {
          stopNavImmediately();
          continuousDodgePhase = DODGE_STOP_AFTER_BACKUP;
          continuousDodgePhaseSinceMs = now;
          setStatus("Dodge: reverse complete; pausing before side scan");
        }
        break;

      case DODGE_STOP_AFTER_BACKUP:
        stopNavImmediately();
        if (now - continuousDodgePhaseSinceMs >= 500) {
          servoWrite(SERVO_LOOK_LEFT);
          continuousDodgePhase = DODGE_SCAN_LEFT_SETTLE;
          continuousDodgePhaseSinceMs = now;
          setStatus("Dodge: checking left clearance");
        }
        break;

      case DODGE_SCAN_LEFT_SETTLE:
        stopNavImmediately();
        if (now - continuousDodgePhaseSinceMs >= SERVO_SETTLE_MS) {
          lastDodgeUltrasonicSeq = ultrasonicSequence;
          continuousDodgePhase = DODGE_SCAN_LEFT_WAIT;
        }
        break;

      case DODGE_SCAN_LEFT_WAIT:
        stopNavImmediately();
        if (ultrasonicSequence != lastDodgeUltrasonicSeq) {
          lastDodgeUltrasonicSeq = ultrasonicSequence;
          // A no-echo is represented as 0 here, matching the source's clear-path test.
          dodgeModeLeftClearanceCm = ultrasonicSampleValid ? (float)ultrasonicDistanceCm : 0.0f;
          servoWrite(SERVO_LOOK_RIGHT);
          continuousDodgePhase = DODGE_SCAN_RIGHT_SETTLE;
          continuousDodgePhaseSinceMs = now;
          setStatus("Dodge: checking right clearance");
        }
        break;

      case DODGE_SCAN_RIGHT_SETTLE:
        stopNavImmediately();
        if (now - continuousDodgePhaseSinceMs >= SERVO_SETTLE_MS) {
          lastDodgeUltrasonicSeq = ultrasonicSequence;
          continuousDodgePhase = DODGE_SCAN_RIGHT_WAIT;
        }
        break;

      case DODGE_SCAN_RIGHT_WAIT:
        stopNavImmediately();
        if (ultrasonicSequence != lastDodgeUltrasonicSeq) {
          lastDodgeUltrasonicSeq = ultrasonicSequence;
          dodgeModeRightClearanceCm = ultrasonicSampleValid ? (float)ultrasonicDistanceCm : 0.0f;
          // Prefer a side with no echo (clear beyond range); otherwise choose the larger reading.
          if (dodgeModeLeftClearanceCm == 0.0f) dodgeTurnLeft = true;
          else if (dodgeModeRightClearanceCm == 0.0f) dodgeTurnLeft = false;
          else dodgeTurnLeft = dodgeModeLeftClearanceCm >= dodgeModeRightClearanceCm;

          servoWrite(SERVO_CENTER);
          continuousDodgePhase = DODGE_TURN_RIGHT;
          dodgeTimedMotionStarted = false;
          setStatus(dodgeTurnLeft ? "Dodge: turning toward clearer left side" : "Dodge: turning toward clearer right side");
        }
        break;

      case DODGE_TURN_RIGHT:
        // Differential-drive pivot; negative-left/positive-right is a left turn.
        setNavWheelTargets(dodgeTurnLeft ? -200 : 200, dodgeTurnLeft ? 200 : -200);
        if (!dodgeTimedMotionStarted &&
            (dodgeTurnLeft ? (navAppliedLeftPwm <= -200 && navAppliedRightPwm >= 200)
                           : (navAppliedLeftPwm >= 200 && navAppliedRightPwm <= -200))) {
          dodgeTimedMotionStarted = true;
          continuousDodgePhaseSinceMs = now;
        } else if (dodgeTimedMotionStarted && now - continuousDodgePhaseSinceMs >= 500) {
          stopNavImmediately();
          continuousDodgePhase = DODGE_STOP_AFTER_TURN;
          continuousDodgePhaseSinceMs = now;
          setStatus("Dodge: turn complete; pausing before forward travel");
        }
        break;

      case DODGE_STOP_AFTER_TURN:
        stopNavImmediately();
        if (now - continuousDodgePhaseSinceMs >= 200) {
          servoWrite(SERVO_CENTER);
          lastDodgeUltrasonicSeq = ultrasonicSequence;
          continuousDodgePhase = DODGE_CRUISE;
          setStatus("Dodge: maneuver complete; continuing forward");
        }
        break;

      case DODGE_WAIT_RANGE:
        stopNavImmediately();
        if (ultrasonicSequence != lastDodgeUltrasonicSeq) {
          lastDodgeUltrasonicSeq = ultrasonicSequence;
          continuousDodgePhase = DODGE_CRUISE;
          setStatus("Dodge: fresh range reading; resuming");
        }
        break;

      // Not used by standalone Dodge; kept as safe, stopped fallbacks.
      case DODGE_TO_WAYPOINT:
      case DODGE_RESTORE_HEADING:
      case DODGE_CENTER_SETTLE:
      case DODGE_VERIFY_FORWARD:
      case DODGE_BLOCKED:
        stopNavImmediately();
        continuousDodgePhase = DODGE_CRUISE;
        break;
    }
  }

  void driveSides(int physicalLeft, int physicalRight) {
    setNavWheelTargets(physicalLeft, physicalRight);
  }

  void pivotToward(int side, int speed) {
    driveSides(side * speed, -side * speed);
  }

  // =====================================================================
  //  FOLLOW MODE
  // =====================================================================

  void resetFollow() {
    stopNavImmediately();
    for (int i = 0; i < FOLLOW_COUNT; i++) {
      fDist[i] = 400;
      fTime[i] = 0;
    }
    fIdx = 3;
    fDir = 1;
    fLastTick = 0;
    lastFollowUltrasonicSeq = ultrasonicSequence;
    servoWrite(FOLLOW_ANGLES[fIdx]);
    setStatus("Follow mode: calibrating servo");
    startServoCalibration();
  }

  void runFollowMode() {
    if (calibStage != 0) return;
    unsigned long now = millis();
    if (now - fLastTick < FOLLOW_TICK_MS) return;
    fLastTick = now;

    // Consume only a fresh asynchronous sample, then advance the servo scan.
    if (ultrasonicSequence != lastFollowUltrasonicSeq) {
      lastFollowUltrasonicSeq = ultrasonicSequence;
      fDist[fIdx] = ultrasonicSampleValid ? ultrasonicDistanceCm : 400;
      fTime[fIdx] = ultrasonicSampleMs;
      fIdx += fDir;
      if (fIdx >= FOLLOW_COUNT - 1) { fIdx = FOLLOW_COUNT - 1; fDir = -1; }
      else if (fIdx <= 0) { fIdx = 0; fDir = 1; }
      servoWrite(FOLLOW_ANGLES[fIdx]);
    }

    // 2) Find the nearest recent echo
    long dmin = 1000;
    int imin = -1;
    for (int i = 0; i < FOLLOW_COUNT; i++) {
      if (fTime[i] == 0 || now - fTime[i] > FOLLOW_STALE_MS) continue;
      if (fDist[i] <= FOLLOW_MAX_CM && fDist[i] < dmin) {
        dmin = fDist[i];
        imin = i;
      }
    }

    if (imin < 0) {
      // No recent usable echo is a stop condition. Never continue the last drive command on stale data.
      stopNavImmediately();
      setStatus("No recent target - stopped; place target in sensor view");
      return;
    }

    // 3) Direction of the object: weighted average of the angles that see it
    float wsum = 0.0;
    float asum = 0.0;
    for (int i = 0; i < FOLLOW_COUNT; i++) {
      if (fTime[i] == 0 || now - fTime[i] > FOLLOW_STALE_MS) continue;
      if (fDist[i] > dmin + 10) continue;
      float w = 1.0 / (1.0 + (fDist[i] - dmin));
      wsum += w;
      asum += w * FOLLOW_ANGLES[i];
    }
    float angle = asum / wsum;
    float offset = angle - SERVO_CENTER;
    float absOff = fabs(offset);
    bool highSide = offset > 0;
    int objSide = (highSide == SERVO_HIGH_IS_LEFT) ? -1 : 1;   // -1 = object on the left, +1 = on the right

    // Exact prescribed distance bands: >15 cm approach, 10-15 cm hold, <10 cm reverse.
    if (dmin < FOLLOW_MIN_CM) {
      // Back straight away until a fresh scan reports at least 10 cm.
      driveSides(-FOLLOW_MIN_SPEED, -FOLLOW_MIN_SPEED);
      setStatus("Target under 10 cm - backing off");
      return;
    }
    if (dmin <= FOLLOW_HOLD_MAX_CM) {
      stopNavImmediately();
      setStatus("Target in 10-15 cm hold band");
      return;
    }

    char msg[60];
    snprintf(msg, sizeof(msg), "Following: object %ld cm, %s", dmin,
            (absOff < FOLLOW_DEADBAND_DEG) ? "straight ahead" : (objSide < 0 ? "on the left" : "on the right"));
    setStatus(msg);

    // Above 15 cm, steer toward the tracked bearing and approach the target.
    long dClamped = constrain(dmin, FOLLOW_HOLD_MAX_CM + 1, FOLLOW_MAX_CM);
    int fwd = map(dClamped, FOLLOW_HOLD_MAX_CM + 1, FOLLOW_MAX_CM, FOLLOW_MIN_SPEED, FOLLOW_MAX_SPEED);

    if (absOff > FOLLOW_PIVOT_DEG) {
      pivotToward(objSide, FOLLOW_TURN_SPEED);          // object far to one side: turn in place first
    } else if (absOff > FOLLOW_DEADBAND_DEG) {
      float k = (absOff - FOLLOW_DEADBAND_DEG) / (FOLLOW_PIVOT_DEG - FOLLOW_DEADBAND_DEG);
      float cut = 0.85 * k;
      float keep = 1.0 - cut;
      int inner = (int)(fwd * keep);
      if (objSide < 0) driveSides(inner, fwd);          // object on the left: left wheels slower
      else driveSides(fwd, inner);
    } else {
      driveSides(fwd, fwd);
    }
  }

  // --- EMERGENCY STOP ---

  void setEmergencyStop(bool on) {
    emergencyStop = on;
    isAccelerating = false;
    isBraking = false;
    steerLeftBtn = false;
    steerRightBtn = false;
    buttonSteer = 0.0;
    currentSpeed = 0.0;
    stopNavImmediately();
    if (!on) {
      if (carMode == MODE_AUTO) resetAuto();
      else if (carMode == MODE_FOLLOW) resetFollow();
      else if (carMode == MODE_DODGE) resetDodgeMode();
    }
  }

  // --- MODE SWITCHING ---

  String modeName() {
    if (carMode == MODE_AUTO) return String("auto");
    if (carMode == MODE_FOLLOW) return String("follow");
    if (carMode == MODE_DODGE) return String("dodge");
    return String("manual");
  }

  // The OLED shows DODGE while AUTO is in its obstacle-bypass state.
  String displayModeName() {
    if (carMode == MODE_DODGE || (carMode == MODE_AUTO && autoState == STATE_AVOIDING)) return String("dodge");
    return modeName();
  }

  void setCarMode(int m) {
    carMode = m;

    stopNavImmediately();
    currentSpeed = 0.0;
    isAccelerating = false;
    isBraking = false;
    steerLeftBtn = false;
    steerRightBtn = false;
    buttonSteer = 0.0;

    if (m == MODE_AUTO) {
      resetAuto();
    } else if (m == MODE_FOLLOW) {
      resetFollow();
    } else if (m == MODE_DODGE) {
      calibStage = 0;
      resetDodgeMode();
    } else {
      calibStage = 0;
      servoWrite(SERVO_CENTER);
      setStatus("Manual mode");
    }
  }

  // --- WEB ROUTE HANDLERS ---

  void handleRoot() { server.send(200, "text/html", HTML_PAGE); }

  // /mode?m=manual|auto|follow|dodge (no argument reports the current mode)
  void handleMode() {
    if (server.hasArg("m")) {
      String m = server.arg("m");
      if (m == "manual") setCarMode(MODE_MANUAL);
      else if (m == "auto") setCarMode(MODE_AUTO);
      else if (m == "follow") setCarMode(MODE_FOLLOW);
      else if (m == "dodge") setCarMode(MODE_DODGE);
    }
    server.send(200, "text/plain", modeName());
  }

  // Dashboard coordinates are in centimetres from the boot origin; reject malformed/ridiculous inputs.
  void handleTarget() {
    if (carMode != MODE_AUTO || emergencyStop || !server.hasArg("x") || !server.hasArg("y")) {
      server.send(409, "text/plain", "Switch to AUTO and release emergency stop first");
      return;
    }
    String xText = server.arg("x");
    String yText = server.arg("y");
    char *xEnd = nullptr;
    char *yEnd = nullptr;
    const float x = strtof(xText.c_str(), &xEnd);
    const float y = strtof(yText.c_str(), &yEnd);
    if (xEnd == xText.c_str() || yEnd == yText.c_str() || *xEnd != '\0' || *yEnd != '\0' ||
        !isfinite(x) || !isfinite(y) || fabsf(x) > 5000.0f || fabsf(y) > 5000.0f) {
      server.send(400, "text/plain", "Coordinates must be finite centimetre values within +/-5000");
      return;
    }
    beginNavigation(x, y);
    server.send(200, "text/plain", "Destination accepted in cm");
  }

  // Phone -> car: held buttons + tilt.  Car -> phone: current wheel speeds "left,right"
  void handleControl() {
    lastCommandTime = millis();
    if (!emergencyStop) {
      isAccelerating = server.arg("g") == "1";
      isBraking = server.arg("b") == "1";
      steerLeftBtn = server.arg("l") == "1";
      steerRightBtn = server.arg("r") == "1";
      tiltAngle = server.arg("t").toFloat();
    }
    server.send(200, "text/plain", String(dispLeft) + "," + String(dispRight));
  }

  void handleEstop() {
    setEmergencyStop(server.arg("state") == "1");
    server.send(200, "text/plain", emergencyStop ? "1" : "0");
  }

  // Live tilt calibration from the page sliders. With no arguments it just reports the values.
  void handleCal() {
    if (server.hasArg("i") || server.hasArg("o") || server.hasArg("d")) {
      if (server.hasArg("i")) innerSlowPerDeg = constrain(server.arg("i").toFloat(), 0.0, 5.0) / 100.0;
      if (server.hasArg("o")) outerFastPerDeg = constrain(server.arg("o").toFloat(), 0.0, 3.0) / 100.0;
      if (server.hasArg("d")) tiltDeadzoneDeg = constrain(server.arg("d").toFloat(), 0.0, 15.0);

      Serial.printf("Tilt calibration -> inner slow %.1f %%/deg, outer fast %.1f %%/deg, deadzone %.0f deg\n",
                    innerSlowPerDeg * 100.0, outerFastPerDeg * 100.0, tiltDeadzoneDeg);
    }
    String s = String(innerSlowPerDeg * 100.0, 1) + "," + String(outerFastPerDeg * 100.0, 1) + "," + String(tiltDeadzoneDeg, 0);
    server.send(200, "text/plain", s);
  }

  // Car -> phone message: "CODE|text"  (CODE = OK, ASK or STUCK)
  void handleStatus() {
    String state = "IDLE";
    if (carMode == MODE_AUTO) {
      if (autoState == STATE_NAVIGATING) state = "NAVIGATING";
      else if (autoState == STATE_AVOIDING) state = "DODGE_MODE";
    } else if (carMode == MODE_DODGE) {
      state = "DODGE_MODE";
    }
    String text = (carMode == MODE_MANUAL) ? String("Manual mode") : String(statusBuf);
    if (carMode == MODE_AUTO || carMode == MODE_DODGE) {
      text += " | PWM L/R:" + String(navAppliedLeftPwm) + "/" + String(navAppliedRightPwm) +
              " | Range:" + (ultrasonicSampleValid ? String(ultrasonicDistanceCm) + " cm" : String("no echo"));
    }
    if (carMode == MODE_AUTO) {
      text += " | Pose X:" + String(robotPose.xCm, 1) + " Y:" + String(robotPose.yCm, 1) +
              " cm H:" + String(robotPose.thetaRad * 180.0f / PI, 1) + " deg";
    }
    server.send(200, "text/plain", state + "|" + text);
  }

  void handleChoose() {
    server.send(409, "text/plain", "Auto mode chooses its own dodge path");
  }

  // Phone joined the car's Wi-Fi: re-home the servo (0 deg, then centre)
  void onStationConnected(arduino_event_id_t event) {
    calibRequested = true;
  }

  void setup() {
    Serial.begin(115200);

    // Motor pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // PWM channels (motors first, then servo)
    ledcAttach(ENA, PWM_FREQ, PWM_RES);
    ledcAttach(ENB, PWM_FREQ, PWM_RES);
    ledcAttach(SERVO_PIN, SERVO_PWM_FREQ, SERVO_PWM_RES);

    applyMotorSpeeds(0, 0);

    // Ultrasonic
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), onEchoChange, CHANGE);
    pinMode(LEFT_ENCODER_PIN, INPUT);  // GPIO35 has no internal pull-up; supply clean 0-3.3 V logic
    pinMode(RIGHT_ENCODER_PIN, INPUT); // GPIO34 has no internal pull-up; supply clean 0-3.3 V logic
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), onLeftEncoderPulse, FALLING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), onRightEncoderPulse, FALLING);
    resetOdometry();
    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    Wire.setClock(400000); // faster OLED transfers reduce pauses in motor-control servicing
    oledReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS);
    if (oledReady) {
      display.clearDisplay();
      display.setTextColor(SSD1306_WHITE);
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("ESP32 car booting");
      display.display();
    } else {
      Serial.println("OLED init failed: check SSD1306 address/wiring; drive functions remain available");
    }
    lastSpeedUpdateMs = millis();

    // Wi-Fi access point + web server
    WiFi.onEvent(onStationConnected, ARDUINO_EVENT_WIFI_AP_STACONNECTED);
    WiFi.softAP(ssid, password);
    WiFi.setSleep(false);   // no Wi-Fi power saving = lower latency

    server.on("/", handleRoot);
    server.on("/mode", handleMode);
    server.on("/target", handleTarget);
    server.on("/ctl", handleControl);
    server.on("/estop", handleEstop);
    server.on("/cal", handleCal);
    server.on("/status", handleStatus);
    server.on("/choose", handleChoose);
    server.begin();

    startServoCalibration();

    Serial.println("ESP32 car ready. Join Wi-Fi ESP32_Car_Control, then open http://192.168.4.1");
  }

  void loop() {
    server.handleClient();
    serviceCalibration();
    serviceUltrasonic();
    updateSpeedMeasurement();
    updateSpeedDisplay();

    const unsigned long now = millis();
    if (now - lastPhysicsUpdate >= 20) {
      lastPhysicsUpdate = now;

      if (emergencyStop) {
        stopNavImmediately();
        applyMotorSpeeds(0, 0);
      } else if (carMode == MODE_MANUAL) {
        if ((isAccelerating || isBraking || steerLeftBtn || steerRightBtn) &&
            now - lastCommandTime > COMMAND_TIMEOUT_MS) {
          isAccelerating = false;
          isBraking = false;
          steerLeftBtn = false;
          steerRightBtn = false;
        }
        updatePhysicsAndMotors();
      } else if (carMode == MODE_AUTO) {
        runAutoMode();
        serviceNavMotorRamp();
      } else if (carMode == MODE_FOLLOW) {
        runFollowMode();
        serviceNavMotorRamp();
      } else {
        runDodgeMode();
        serviceNavMotorRamp();
      }
      updateOdometry();
    }
  }
