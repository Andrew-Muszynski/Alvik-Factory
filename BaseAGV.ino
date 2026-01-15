/*
  BaseAGV.ino - Arduino Alvik Factory AGV 

  UPDATED:
  - BLUE sticker is the START/ORIGIN node (A), x,y = 0,0
  - RED stickers are route markers (count 1 => D, count 2 => E)
  - Robust waypoint detection with HSV confidence gating
  - Startup lockout until BLUE confirmation
*/

#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <Arduino_Alvik.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

// =================== Type Definitions ===============
typedef enum {
  STATE_NOT_READY,
  STATE_IDLE,
  STATE_WAITING_LOAD,
  STATE_MOVING,
  STATE_ARRIVED,
  STATE_RETURNING,
  STATE_ADVANCING      
} RobotState;

typedef enum {
  POS_UNKNOWN,
  POS_A,
  POS_B,
  POS_C,
  POS_D,
  POS_E
} Position;

typedef enum {
  WP_NONE = 0,
  WP_BLUE = 1,
  WP_RED  = 2
} WaypointColor;

// =================== WiFi & Agent ===================
char WIFI_SSID[]     = "ISECapstone";                   // Replace with your WiFi username
char WIFI_PASSWORD[] = "j0shf1sh";                      // Replace with your WiFi password
char AGENT_IP[]      = "192.168.0.0";                 // Replace with your the IP address of the ROS2 Desktop
uint32_t AGENT_PORT  = 8888;                            // Change if your port 8888 was busy and the agent cannot host with it

// =================== Robot Configuration ============
char ROBOT_NAME[16] = "Alvik1";
char T_STATUS[32], T_CMD[32];

// =================== micro-ROS ======================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

rcl_publisher_t pub_status;
rcl_subscription_t sub_cmd;

std_msgs__msg__String msg_status;
char cmd_buf[128];
std_msgs__msg__String msg_cmd_in;

// =================== Alvik ==========================
Arduino_Alvik alvik;

// =================== Line Following Params ==========
const int   TAPE_MIN      = 100;
const float BASE_SPEED    = 35.0f;
const float TURN_SPEED    = 60.0f;
const float REVERSE_TURN  = -12.0f;
const float KP            = 0.03f;
const float KD            = 0.18f;

int L, C, R;
float last_error = 0;
unsigned long lost_start_ms = 0;

// =================== Safety =========================
const float SAFE_STOP_CM = 7.0f;
bool safety_hold = false;

// =================== Robot State ====================
RobotState robot_state = STATE_NOT_READY;
Position current_position = POS_UNKNOWN;
Position target_destination = POS_D;
Position target_queue_position = POS_UNKNOWN;  // NEW: For advancing/returning

// =================== Waypoint / Color Detection =====
const int COLOR_STABLE_COUNT = 5;
const unsigned long COLOR_COOLDOWN_MS = 1000;
int color_hit_count = 0;
unsigned long last_color_detection_time = 0;

bool clearing_sticker = false;
unsigned long clear_start_time = 0;
const unsigned long CLEAR_TIME_MS = 800;

int red_count = 0;
int blue_count = 0;   // NEW: Count blue stickers during return/advance

// Startup confirmation on BLUE start node
bool ready_confirmed = false;
unsigned long blue_confirm_start_ms = 0;
const unsigned long START_CONFIRM_MS = 2000;

// =================== Pose / Orientation =============
float start_yaw_deg = 0.0f;

// =================== Robust Waypoint Thresholds =====
const int   LINE_TAPE_THRESH = 250;

const float S_MIN_STICKER = 0.60f;
const float V_MIN_STICKER = 0.08f;
const float V_MAX_STICKER = 0.30f;

const float START_S_MIN = 0.45f;
const float START_V_MIN = 0.05f;
const float START_V_MAX = 0.35f;

// =================== Route + Forced Corner Turn =====
enum TurnDir : int8_t { TURN_LEFT = -1, TURN_RIGHT = +1 };

const TurnDir ROUTE_TURNS[] = { TURN_LEFT, TURN_RIGHT, TURN_RIGHT, TURN_RIGHT, TURN_RIGHT, TURN_RIGHT };
const int ROUTE_TURN_COUNT = sizeof(ROUTE_TURNS) / sizeof(ROUTE_TURNS[0]);

volatile int route_turn_idx = 0;

bool forced_turn_active = false;
unsigned long forced_turn_start_ms = 0;
int forced_turn_dir = +1;
unsigned long lost_all_start_ms = 0;

const int OFF_TAPE_THRESH = 150;
const unsigned long CORNER_LOST_MS = 120;
const unsigned long MAX_TURN_MS = 1800;

// =================== Robot ID =======================
int getAlvikID() {
  String mac = WiFi.macAddress();
  mac.toUpperCase();
  if (mac == "74:4D:BD:9F:C6:E8") return 1;   // Alvik1 (new)
  if (mac == "3C:84:27:C2:BC:40") return 2;   // Alvik2
  return 3;
}

// =================== Utility ========================
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void setWheels(float l, float r) {
  const float MAX_CMD = 60.0f;
  const float MIN_CMD = 8.0f;
  if (fabsf(l) > 1e-3 && fabsf(l) < MIN_CMD) l = (l > 0) ? MIN_CMD : -MIN_CMD;
  if (fabsf(r) > 1e-3 && fabsf(r) < MIN_CMD) r = (r > 0) ? MIN_CMD : -MIN_CMD;
  alvik.set_wheels_speed(clampf(l, -MAX_CMD, MAX_CMD), clampf(r, -MAX_CMD, MAX_CMD));
}

void stopRobot() {
  alvik.brake();
}

float normDeg(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// =================== LED Indicators =================
void setLEDState(RobotState state) {
  switch(state) {
    case STATE_NOT_READY:
      alvik.left_led.set_color(1, 0, 1);   // Magenta
      alvik.right_led.set_color(1, 0, 1);
      break;
    case STATE_IDLE:
      alvik.left_led.set_color(0, 0, 1);   // Blue
      alvik.right_led.set_color(0, 0, 1);
      break;
    case STATE_WAITING_LOAD:
      alvik.left_led.set_color(1, 1, 0);   // Yellow
      alvik.right_led.set_color(1, 1, 0);
      break;
    case STATE_MOVING:
      alvik.left_led.set_color(0, 1, 0);   // Green
      alvik.right_led.set_color(0, 1, 0);
      break;
    case STATE_ARRIVED:
      alvik.left_led.set_color(1, 0, 1);   // Magenta
      alvik.right_led.set_color(1, 0, 1);
      break;
    case STATE_RETURNING:
      alvik.left_led.set_color(0, 1, 1);   // Cyan
      alvik.right_led.set_color(0, 1, 1);
      break;
    case STATE_ADVANCING:
      alvik.left_led.set_color(1, 0.5, 0); // Orange
      alvik.right_led.set_color(1, 0.5, 0);
      break;
  }
}

// =================== HSV + Waypoint Helpers =========
static inline bool hueInRed(float h) {
  return (h >= 340.0f || h <= 20.0f);
}

static inline bool hueInBlue(float h) {
  return (h >= 190.0f && h <= 240.0f);
}

bool onTapeOrEdge() {
  alvik.get_line_sensors(L, C, R);
  return (L > LINE_TAPE_THRESH) || (C > LINE_TAPE_THRESH) || (R > LINE_TAPE_THRESH);
}

void readHSV(float &h, float &s, float &v) {
  alvik.get_color(h, s, v, HSV);
}

bool hsvStickerConfident(float s, float v) {
  return (s >= S_MIN_STICKER) && (v >= V_MIN_STICKER) && (v <= V_MAX_STICKER);
}

WaypointColor detectWaypointColor() {
  float h, s, v;
  readHSV(h, s, v);

  alvik.get_line_sensors(L, C, R);
  if ((L < OFF_TAPE_THRESH) || (C < OFF_TAPE_THRESH) || (R < OFF_TAPE_THRESH)) {
    return WP_NONE;
  }

  if (!hsvStickerConfident(s, v)) return WP_NONE;

  if (hueInRed(h))  return WP_RED;
  if (hueInBlue(h)) return WP_BLUE;

  return WP_NONE;
}

String detectColorLabelForTelemetry() {
  float h, s, v;
  readHSV(h, s, v);

  if (v < 0.07f) return "black";
  if (v < 0.35f && s < 0.25f) return "grey";
  if (s < 0.15f && v > 0.70f) return "white";

  if (hueInRed(h) && hsvStickerConfident(s, v)) return "red";
  if (hueInBlue(h) && hsvStickerConfident(s, v)) return "blue";

  char buf[48];
  snprintf(buf, sizeof(buf), "h%.0f_s%.2f_v%.2f", h, s, v);
  return String(buf);
}

// =================== Startup confirm BLUE ===========
void processStartConfirmation() {
  stopRobot();
  unsigned long now = millis();

  alvik.get_line_sensors(L, C, R);

  bool off_tape = (L < OFF_TAPE_THRESH) || (C < OFF_TAPE_THRESH) || (R < OFF_TAPE_THRESH);
  if (off_tape) {
    blue_confirm_start_ms = 0;
    setLEDState(STATE_NOT_READY);
    return;
  }

  float h, s, v;
  readHSV(h, s, v);

  bool looks_like_blue_start =
      hueInBlue(h) &&
      (s >= START_S_MIN) &&
      (v >= START_V_MIN) &&
      (v <= START_V_MAX);

  if (looks_like_blue_start) {
    if (blue_confirm_start_ms == 0) blue_confirm_start_ms = now;

    if ((now - blue_confirm_start_ms) >= START_CONFIRM_MS) {
      ready_confirmed = true;
      current_position = POS_A;

      alvik.reset_pose(0, 0, start_yaw_deg, CM, DEG);

      robot_state = STATE_IDLE;
      setLEDState(STATE_IDLE);

      last_color_detection_time = now;
      color_hit_count = 0;
      clearing_sticker = false;
      red_count = 0;
      blue_count = 0;
    }
  } else {
    blue_confirm_start_ms = 0;
    setLEDState(STATE_NOT_READY);
  }
}

// =================== Line Following =================
void followStep() {
  // Safety check
  float front = alvik.get_distance_top(CM);
  if (front > 0 && front < SAFE_STOP_CM) {
    if (!safety_hold) {
      stopRobot();
      safety_hold = true;
    }
    return;
  } else {
    safety_hold = false;
  }

  // Handle sticker clearing mode
  if (clearing_sticker) {
    if (millis() - clear_start_time >= CLEAR_TIME_MS) {
      clearing_sticker = false;
      last_color_detection_time = millis();
      color_hit_count = 0;
    }
  }

  // -------- Waypoint detection --------
  if (!clearing_sticker) {
    bool can_detect = (millis() - last_color_detection_time) > COLOR_COOLDOWN_MS;
    if (can_detect) {
      WaypointColor wp = detectWaypointColor();

      // DELIVERY: Count red stickers for D/E destinations
      if (robot_state == STATE_MOVING) {
        if (wp == WP_RED) {
          color_hit_count++;
          if (color_hit_count >= COLOR_STABLE_COUNT) {
            red_count++;
            color_hit_count = 0;

            bool reached = false;
            if (target_destination == POS_D && red_count == 1) {
              reached = true;
              current_position = POS_D;
            } else if (target_destination == POS_E && red_count == 2) {
              reached = true;
              current_position = POS_E;
            }

            if (reached) {
              stopRobot();
              robot_state = STATE_ARRIVED;
              setLEDState(STATE_ARRIVED);
              last_color_detection_time = millis();

              forced_turn_active = false;
              lost_all_start_ms = 0;
              return;
            } else {
              clearing_sticker = true;
              clear_start_time = millis();
            }
          }
        } else {
          color_hit_count = 0;
        }
      }
      // RETURNING: Count blue stickers for queue positions C, B, A
      else if (robot_state == STATE_RETURNING) {
        if (wp == WP_BLUE) {
          color_hit_count++;
          if (color_hit_count >= COLOR_STABLE_COUNT) {
            blue_count++;
            color_hit_count = 0;

            bool should_stop = false;
            Position detected_pos = POS_UNKNOWN;

            if (blue_count == 1) {
              detected_pos = POS_C;
              if (target_queue_position == POS_C || target_queue_position == POS_UNKNOWN) should_stop = true;
            } else if (blue_count == 2) {
              detected_pos = POS_B;
              if (target_queue_position == POS_B) should_stop = true;
            } else if (blue_count == 3) {
              detected_pos = POS_A;
              if (target_queue_position == POS_A) should_stop = true;
            }

            if (should_stop) {
              stopRobot();
              current_position = detected_pos;
              robot_state = STATE_IDLE;
              setLEDState(STATE_IDLE);
              last_color_detection_time = millis();
              blue_count = 0;

              forced_turn_active = false;
              lost_all_start_ms = 0;
              return;
            } else {
              clearing_sticker = true;
              clear_start_time = millis();
            }
          }
        } else {
          color_hit_count = 0;
        }
      }
      // ADVANCING: Count blue stickers for queue advancement
      else if (robot_state == STATE_ADVANCING) {
        if (wp == WP_BLUE) {
          color_hit_count++;
          if (color_hit_count >= COLOR_STABLE_COUNT) {
            blue_count++;
            color_hit_count = 0;

            bool reached_target = false;
            if (target_queue_position == POS_A && blue_count == 1) {
              reached_target = true;
              current_position = POS_A;
            } else if (target_queue_position == POS_B && blue_count == 1) {
              reached_target = true;
              current_position = POS_B;
            }

            if (reached_target) {
              stopRobot();
              robot_state = STATE_IDLE;
              setLEDState(STATE_IDLE);
              last_color_detection_time = millis();
              blue_count = 0;

              forced_turn_active = false;
              lost_all_start_ms = 0;
              return;
            } else {
              clearing_sticker = true;
              clear_start_time = millis();
            }
          }
        } else {
          color_hit_count = 0;
        }
      }
    }
  }

  // -------- Line following with ROUTE-LOCKED corner handling --------
  alvik.get_line_sensors(L, C, R);

  // Tape presence
  bool L_on = (L > TAPE_MIN);
  bool C_on = (C > TAPE_MIN);
  bool R_on = (R > TAPE_MIN);

  // Strong off-tape test (floor is ~50)
  bool all_off = (L < OFF_TAPE_THRESH) && (C < OFF_TAPE_THRESH) && (R < OFF_TAPE_THRESH);

  // Corner signature: center is gone, but one side still sees tape (edge-riding at a corner)
  bool corner_signature = (!C_on) && (L_on ^ R_on);

  // Forced turn exit criteria: require CENTER to be back on tape stably (prevents early exit)
  static unsigned long center_reacq_start_ms = 0;

  // Minimum time to enforce the forced turn before we even consider exiting
  const unsigned long MIN_FORCED_TURN_MS = 250;

  // Turning speeds
  const float TURN = 34.0f;   // pivot speed during forced turn
  const float NUDGE = 26.0f;  // small pivot while waiting to trigger

  // === Forced turn active: keep turning until center is reacquired stably ===
  if (forced_turn_active) {
    // Timeout safety
    if (millis() - forced_turn_start_ms > MAX_TURN_MS) {
      stopRobot();
      forced_turn_active = false;
      center_reacq_start_ms = 0;
      return;
    }

    // Enforce minimum duration to avoid "flicker-exit"
    if (millis() - forced_turn_start_ms < MIN_FORCED_TURN_MS) {
      if (forced_turn_dir > 0) setWheels(TURN, -TURN);
      else                     setWheels(-TURN, TURN);
      return;
    }

    // After min duration, require CENTER on tape for some time
    if (C_on) {
      if (center_reacq_start_ms == 0) center_reacq_start_ms = millis();

      // Require center to stay on for 120ms before exiting
      if (millis() - center_reacq_start_ms >= 120) {
        forced_turn_active = false;
        center_reacq_start_ms = 0;

        lost_all_start_ms = 0;
        lost_start_ms = 0;
        last_error = 0;

        // Advance route index ONLY when we complete a forced corner
        if (route_turn_idx < (ROUTE_TURN_COUNT - 1)) {
          route_turn_idx++;
        }
        // fall through to PD line follow this cycle
      } else {
        // Keep turning while center is stabilizing
        if (forced_turn_dir > 0) setWheels(TURN, -TURN);
        else                     setWheels(-TURN, TURN);
        return;
      }
    } else {
      center_reacq_start_ms = 0;
      if (forced_turn_dir > 0) setWheels(TURN, -TURN);
      else                     setWheels(-TURN, TURN);
      return;
    }
  }

  // === Corner trigger: either fully off tape OR corner signature (edge riding) ===
  if (all_off || corner_signature) {
    if (lost_all_start_ms == 0) lost_all_start_ms = millis();

    // Trigger forced turn after a short persistence window
    if (millis() - lost_all_start_ms >= CORNER_LOST_MS) {
      TurnDir d = ROUTE_TURNS[route_turn_idx];

      forced_turn_active = true;
      forced_turn_start_ms = millis();
      forced_turn_dir = (int)d;
      center_reacq_start_ms = 0;

      if (forced_turn_dir > 0) setWheels(TURN, -TURN);
      else                     setWheels(-TURN, TURN);
      return;
    }

    // While waiting for CORNER_LOST_MS, nudge in the ROUTE direction (not arbitrary)
    TurnDir nd = ROUTE_TURNS[route_turn_idx];
    if ((int)nd > 0) setWheels(NUDGE, -NUDGE);
    else             setWheels(-NUDGE, NUDGE);
    last_error = 0;
    return;
  } else {
    lost_all_start_ms = 0;
  }

  // --- Off-edge recovery: snap back if one side is off tape ---
  if ((L < OFF_TAPE_THRESH) && (C_on || R_on)) {
    lost_start_ms = 0;
    last_error = 0;
    const float HARD = 40.0f;
    const float FWD  = 18.0f;
    setWheels(FWD + HARD, FWD - HARD); // steer right hard
    return;
  }
  if ((R < OFF_TAPE_THRESH) && (C_on || L_on)) {
    lost_start_ms = 0;
    last_error = 0;
    const float HARD = 40.0f;
    const float FWD  = 18.0f;
    setWheels(FWD - HARD, FWD + HARD); // steer left hard
    return;
  }

  // Smooth PD (normal tracking)
  float weighted = (0.0f * (float)L) + (1.0f * (float)C) + (2.0f * (float)R);
  float total    = (float)L + (float)C + (float)R;
  float avg      = (total > 50.0f) ? (weighted / total) : 1.0f;

  float error      = avg - 1.0f;
  float derivative = error - last_error;

  const float KP2 = KP * 1.25f;
  const float KD2 = KD * 1.00f;

  float correction = KP2 * error + KD2 * derivative;
  last_error = error;

  float left_speed  = BASE_SPEED + correction;
  float right_speed = BASE_SPEED - correction;

  setWheels(left_speed, right_speed);
}


// =================== ROS2 Command Handler ===========
void cmd_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  String cmd = String(msg->data.data);
  cmd.trim();
  cmd.toLowerCase();

  if (!ready_confirmed) {
    if (cmd == "stop") {
      robot_state = STATE_NOT_READY;
      stopRobot();
      setLEDState(STATE_NOT_READY);
    }
    return;
  }

  if (cmd.startsWith("deliver ")) {
    char dest = cmd.charAt(8);
    if (dest == 'd') {
      target_destination = POS_D;
    } else if (dest == 'e') {
      target_destination = POS_E;
    }

    if (current_position == POS_A) {
      // Reset route index ONLY when accepting a NEW delivery job
      route_turn_idx = 0;
      forced_turn_active = false;
      lost_all_start_ms = 0;
      lost_start_ms = 0;
      last_error = 0;

      robot_state = STATE_WAITING_LOAD;
      setLEDState(STATE_WAITING_LOAD);
    }
  }
  else if (cmd == "ready") {
    if (robot_state == STATE_WAITING_LOAD) {
      robot_state = STATE_MOVING;
      setLEDState(STATE_MOVING);

      red_count = 0;
      last_color_detection_time = 0;
      color_hit_count = 0;
      clearing_sticker = false;

      // DON'T reset route_turn_idx here - already set when job was accepted
      forced_turn_active = false;
      lost_all_start_ms = 0;
      lost_start_ms = 0;
      last_error = 0;
    }
    else if (robot_state == STATE_ARRIVED) {
      robot_state = STATE_RETURNING;
      setLEDState(STATE_RETURNING);

      // Default to first available if not specified
      if (target_queue_position == POS_UNKNOWN) {
        target_queue_position = POS_C;
      }

      blue_count = 0;
      last_color_detection_time = 0;
      color_hit_count = 0;
      clearing_sticker = false;

      // DON'T reset route_turn_idx - continue from current corner position
      forced_turn_active = false;
      lost_all_start_ms = 0;
      lost_start_ms = 0;
      last_error = 0;
    }
  }
  else if (cmd.startsWith("return_to ")) {
    char pos = cmd.charAt(10);
    if (pos == 'a') target_queue_position = POS_A;
    else if (pos == 'b') target_queue_position = POS_B;
    else if (pos == 'c') target_queue_position = POS_C;
  }
  else if (cmd.startsWith("advance_to ")) {
    char target_pos = cmd.charAt(11);
    
    if (robot_state == STATE_IDLE) {
      if (target_pos == 'a' && current_position == POS_B) {
        target_queue_position = POS_A;
        robot_state = STATE_ADVANCING;
        setLEDState(STATE_ADVANCING);
        
        blue_count = 0;
        last_color_detection_time = 0;
        color_hit_count = 0;
        clearing_sticker = false;
        
        forced_turn_active = false;
        lost_all_start_ms = 0;
        lost_start_ms = 0;
        last_error = 0;
      }
      else if (target_pos == 'b' && current_position == POS_C) {
        target_queue_position = POS_B;
        robot_state = STATE_ADVANCING;
        setLEDState(STATE_ADVANCING);
        
        blue_count = 0;
        last_color_detection_time = 0;
        color_hit_count = 0;
        clearing_sticker = false;
        
        forced_turn_active = false;
        lost_all_start_ms = 0;
        lost_start_ms = 0;
        last_error = 0;
      }
    }
  }
  else if (cmd.startsWith("set_position ")) {
    char pos = cmd.charAt(13);
    if (pos == 'a') current_position = POS_A;
    else if (pos == 'b') current_position = POS_B;
    else if (pos == 'c') current_position = POS_C;
  }
  else if (cmd == "stop") {
    robot_state = STATE_IDLE;
    stopRobot();
    setLEDState(STATE_IDLE);

    forced_turn_active = false;
    lost_all_start_ms = 0;
    lost_start_ms = 0;
    last_error = 0;
  }
}

// =================== ROS2 Initialization ============
void init_transport() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    static bool blink = false;
    if (blink) {
      alvik.left_led.set_color(0, 1, 1);
      alvik.right_led.set_color(0, 1, 1);
    } else {
      alvik.left_led.set_color(0, 0, 0);
      alvik.right_led.set_color(0, 0, 0);
    }
    blink = !blink;
  }

  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
}

bool init_graph() {
  allocator = rcl_get_default_allocator();
  if (rmw_uros_ping_agent(1000, 5) != RMW_RET_OK) return false;

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, ROBOT_NAME, "", &support) != RCL_RET_OK) return false;

  if (rclc_publisher_init_default(
        &pub_status, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        T_STATUS) != RCL_RET_OK) return false;

  if (rclc_subscription_init_default(
        &sub_cmd, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        T_CMD) != RCL_RET_OK) return false;

  msg_cmd_in.data.data = cmd_buf;
  msg_cmd_in.data.size = 0;
  msg_cmd_in.data.capacity = sizeof(cmd_buf);

  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd_in, &cmd_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

  msg_status.data.data = NULL;
  msg_status.data.size = 0;
  msg_status.data.capacity = 0;

  return true;
}

// =================== Telemetry ======================
unsigned long last_status_pub = 0;

void publish_status(unsigned long now) {
  if (now - last_status_pub >= 200) {
    last_status_pub = now;

    alvik.get_line_sensors(L, C, R);

    float x, y, th;
    alvik.get_pose(x, y, th, CM, DEG);

    float roll, pitch, yaw;
    alvik.get_orientation(roll, pitch, yaw);
    float yaw_deg = (fabsf(yaw) <= 3.2f) ? yaw * 57.2957795f : yaw;

    String color = detectColorLabelForTelemetry();

    const char* state_str =
      (robot_state == STATE_NOT_READY)    ? "NOT_READY" :
      (robot_state == STATE_IDLE)         ? "IDLE" :
      (robot_state == STATE_WAITING_LOAD) ? "WAITING_LOAD" :
      (robot_state == STATE_MOVING)       ? "MOVING" :
      (robot_state == STATE_ARRIVED)      ? "ARRIVED" :
      (robot_state == STATE_RETURNING)    ? "RETURNING" : "ADVANCING";

    const char* pos_str =
      (current_position == POS_A) ? "A" :
      (current_position == POS_B) ? "B" :
      (current_position == POS_C) ? "C" :
      (current_position == POS_D) ? "D" :
      (current_position == POS_E) ? "E" : "UNKNOWN";

    static char status_buf[320];
    snprintf(status_buf, sizeof(status_buf),
      "{\"state\":\"%s\",\"position\":\"%s\",\"x\":%.2f,\"y\":%.2f,\"yaw\":%.2f,"
      "\"color\":\"%s\",\"L\":%d,\"C\":%d,\"R\":%d,\"ms\":%lu}",
      state_str, pos_str, x, y, yaw_deg, color.c_str(), L, C, R, now);

    msg_status.data.data = status_buf;
    msg_status.data.size = strlen(status_buf);
    msg_status.data.capacity = msg_status.data.size + 1;

    rcl_publish(&pub_status, &msg_status, NULL);
  }
}

// =================== Setup / Loop ===================
void setup() {
  alvik.begin();
  stopRobot();

  int id = getAlvikID();
  snprintf(ROBOT_NAME, sizeof(ROBOT_NAME), "Alvik%d", id);
  snprintf(T_STATUS, sizeof(T_STATUS), "%s_status", ROBOT_NAME);
  snprintf(T_CMD, sizeof(T_CMD), "%s_cmd", ROBOT_NAME);

  float roll, pitch, yaw;
  alvik.get_orientation(roll, pitch, yaw);
  start_yaw_deg = (fabsf(yaw) <= 3.2f) ? yaw * 57.2957795f : yaw;
  start_yaw_deg = normDeg(start_yaw_deg);

  init_transport();

  if (!init_graph()) {
    while (true) {
      alvik.left_led.set_color(1, 0, 0);
      alvik.right_led.set_color(1, 0, 0);
      delay(500);
      alvik.left_led.set_color(0, 0, 0);
      alvik.right_led.set_color(0, 0, 0);
      delay(500);
    }
  }

  ready_confirmed = false;
  blue_confirm_start_ms = 0;
  current_position = POS_UNKNOWN;
  robot_state = STATE_NOT_READY;
  setLEDState(STATE_NOT_READY);
}

void loop() {
  unsigned long now = millis();

  if (!ready_confirmed) {
    processStartConfirmation();
    publish_status(now);
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
    delay(5);
    return;
  }

  switch (robot_state) {
    case STATE_NOT_READY:
    case STATE_IDLE:
    case STATE_WAITING_LOAD:
    case STATE_ARRIVED:
      stopRobot();
      break;

    case STATE_MOVING:
    case STATE_RETURNING:
    case STATE_ADVANCING:
      followStep();
      break;
  }

  publish_status(now);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
  delay(5);
}