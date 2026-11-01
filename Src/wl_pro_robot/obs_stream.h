#pragma once
// 観測の口（obs-stream）：20 ms ごとに観測を 1 行の JSON にして WebSocket（ポート 81）に流す。
// navbot-agent 設計書 §3.1・§4.1、01_PREFLIGHT §2・§3。制御則・周期・操作の口は変えない。
//
// 送る 1 通（欄の名前・単位・順は PC 側 yose/record.py の ROBOT_FIELDS と同じ）：
// {"type":"obs","t_ms":…,"pitch":…,"pitch_rate":…,"roll":…,"yaw":…,"yaw_rate":…,"wheel_l":…,"wheel_r":…,
//  "height_cmd":…,"height_meas":-1,"battery_v":…,"go":…,"fall":…,"u":…,"pitch_zero":…,"send_us":…,
//  "loops":…,"loop_max_us":…,"leg_pos_l":…,"leg_pos_r":…,"leg_us":…}
// loops＝前の 1 通から今までに loop() が回った回数、loop_max_us＝その間でいちばん長かった 1 回の時間（µs）。
// ファームの制御の輪が 1 秒に何回回っているかを測るためだけの欄（2026-10-31 robosheep の了承、FIRMWARE_DIFF §3e）。
// 制御には触らない。PC 側は記録の 21 欄を変えず、thinking の層として残す。
//
// leg_pos_l・leg_pos_r＝左右の脚のサーボ STS3032（ID 1＝左、ID 2＝右）の今の位置（0〜4095、読めなければ -1）。
// leg_us＝その 2 個を読むのにかかった時間（µs）。送る直前に 1 回だけ読む（20 ms に 1 回）。
// 読むだけで、サーボへの指令（leg_loop）は変えない（2026-11-01 robosheep の了承、navbot-agent-record/2）。
//
// PC から {"type":"obs_ctl","on":0} を受けると 1 秒に 1 行だけ送る（送信 OFF の比べ用）。"on":1 で戻す。
#include <Arduino.h>
#include <WebSocketsServer.h>
#include <SimpleFOC.h>
#include <MPU6050_tockn.h>
#include "robot.h"
#include "Servo_STS3032.h"

#define OBS_PERIOD_MS 20        // 送信の周期（50 Hz）
#define OBS_OFF_PERIOD_MS 1000  // 送信 OFF のときの周期（1 Hz）
#define OBS_SLOW_US 10000       // 1 回の送信がこれより長くかかったら（実機で 3〜7 ms の揺れが 2 秒おきに出たので 3 ms → 10 ms）
#define OBS_PAUSE_MS 200        // この時間だけ送信を止める（TCP が詰まった時に制御の輪を守る。1 s だと転倒の瞬間が消えるので 200 ms）

extern WebSocketsServer websocket;
extern MPU6050 mpu6050;
extern BLDCMotor motor1;
extern BLDCMotor motor2;
extern float LQR_angle;
extern float LQR_gyro;
extern float YAW_angle;
extern float YAW_gyro;
extern float LQR_u;
extern float angle_zeropoint;

static bool obs_enabled = true;
static unsigned long obs_next_ms = 0;
static unsigned long obs_pause_until_ms = 0;
static unsigned long obs_last_send_us = 0;
static unsigned long obs_loops = 0;          // 前の 1 通からの loop() の回数
static unsigned long obs_loop_max_us = 0;    // その間の 1 回の最長
static unsigned long obs_loop_last_us = 0;   // 前の loop() の時刻
#define OBS_TEXT_MAX 480    // 1 通の本文の上限（脚の 3 欄を足したので 400 → 480）
static char obs_buf[WEBSOCKETS_MAX_HEADER_SIZE + OBS_TEXT_MAX];  // 先頭にヘッダ分の空きを置き、TCP の書き込みを 1 回にする

inline void obs_ctl(bool on) {
  obs_enabled = on;
}

// 零点を起動時の値に戻す（{"type":"obs_ctl","zero_reset":1}）。電源の入れ直しと同じ効果を Wi-Fi から起こすためのもの。
// 公式ファームの零点は立っている間に重心に合わせて動き（4 → 7〜10 度）、座ったあとも残る。残った零点で座り姿勢から
// 「立て」を送ると起き上がりが荒れて転ぶ（2026-10-31 の記録 on_2・on_6・off_2）。制御則そのものは変えないが、
// 「ファームの差分は観測を送るだけ」の決まりからの小さな例外（robosheep の判断、FIRMWARE_DIFF §3c）。
#define OBS_ZERO_DEFAULT 4.02f  // wl_pro_robot.ino の angle_zeropoint の初期値と同じ
inline void obs_zero_reset() {
  angle_zeropoint = OBS_ZERO_DEFAULT;
}

// 再起動（{"type":"obs_ctl","reboot":1}）。電源の入れ直しと同じで、距離の零点・積分・フィルタ・脚の状態まで起動直後に戻る。
// 零点を戻すだけでは立ち上がりの転倒が残ったため（2026-10-31 off_3）。制御には触らない。Wi-Fi は切れるので PC 側がつなぎ直す。
inline void obs_reboot() {
  motor1.target = 0;
  motor2.target = 0;
  delay(50);
  ESP.restart();
}

// ---- 脚のサーボの今の位置を読む ----
// 公式の Servo_STS3032 には書く関数しか無いので、SMS_STS の「読む」命令（0x02、番地 56＝今の位置、2 バイト）を
// ここで 1 回分だけ組む。同じ線（Serial2、1 Mbps）を leg_loop も使うが、どちらも loop() の中なので重ならない。
// 1 回は 8 バイト送って 8 バイト返るだけ（1 Mbps で各 80 µs ほど）。返事が無ければ OBS_LEG_TIMEOUT_US で諦める。
// 続けて読めないサーボは、1 秒に 1 回だけ試す（線が無い・電源が無い時に、毎回待って輪を遅くしないため）。
#define OBS_LEG_ADDR 56            // SMS_STS の今の位置の番地
#define OBS_LEG_TIMEOUT_US 1000    // 1 個の返事を待つ最長
#define OBS_LEG_FAIL_MAX 5         // これだけ続けて読めなければ、間を空ける
#define OBS_LEG_RETRY_MS 1000      // 間を空けたときの試す間隔
static int obs_leg_pos[2] = { -1, -1 };
static unsigned long obs_leg_us = 0;
static uint8_t obs_leg_fail[2] = { 0, 0 };
static unsigned long obs_leg_retry_ms[2] = { 0, 0 };

static int obs_read_leg_once(uint8_t id) {
  HardwareSerial* s = sms_sts.pSerial;
  if (s == NULL) return -1;
  uint8_t req[8] = { 0xFF, 0xFF, id, 4, 0x02, OBS_LEG_ADDR, 2, 0 };
  req[7] = ~(uint8_t)(id + 4 + 0x02 + OBS_LEG_ADDR + 2);
  while (s->read() != -1) {}  // 前の残りを捨てる
  s->write(req, 8);
  uint8_t b[8];
  int k = 0;
  unsigned long t0 = micros();
  while ((unsigned long)(micros() - t0) < OBS_LEG_TIMEOUT_US) {
    int c = s->read();
    if (c < 0) continue;
    if (k < 2) {  // 頭の FF FF を探す
      if (c == 0xFF) b[k++] = 0xFF; else k = 0;
      continue;
    }
    if (k == 2 && c == 0xFF) continue;  // FF が 3 つ以上続いた
    b[k++] = (uint8_t)c;
    if (k < 8) continue;
    k = 0;
    if (memcmp(b, req, 8) == 0) continue;  // 線の作りによっては自分の送った命令がそのまま返るので飛ばす
    uint8_t sum = ~(uint8_t)(b[2] + b[3] + b[4] + b[5] + b[6]);
    if (b[2] != id || b[3] != 4 || b[7] != sum) continue;
    int pos = (int)b[5] | ((int)b[6] << 8);  // 下位が先（SMS_STS）
    if (pos & 0x8000) pos = -(pos & 0x7FFF);  // 15 ビット目は向きの印
    return pos;
  }
  return -1;
}

static void obs_read_legs(unsigned long now_ms) {
  unsigned long t0 = micros();
  for (int i = 0; i < 2; i++) {
    if (obs_leg_fail[i] >= OBS_LEG_FAIL_MAX && (long)(now_ms - obs_leg_retry_ms[i]) < 0) {
      obs_leg_pos[i] = -1;
      continue;
    }
    int v = obs_read_leg_once((uint8_t)(i + 1));  // ID 1＝左、ID 2＝右（setup の並び）
    obs_leg_pos[i] = v;
    if (v < 0) {
      if (obs_leg_fail[i] < 255) obs_leg_fail[i]++;
      obs_leg_retry_ms[i] = now_ms + OBS_LEG_RETRY_MS;
    } else {
      obs_leg_fail[i] = 0;
    }
  }
  obs_leg_us = micros() - t0;
}

// loop() の末尾で毎回呼ぶ。20 ms たっていなければすぐ返る。
inline void obs_stream_loop() {
  unsigned long now_us = micros();
  if (obs_loop_last_us != 0) {
    unsigned long dt = now_us - obs_loop_last_us;
    if (dt > obs_loop_max_us) obs_loop_max_us = dt;
  }
  obs_loop_last_us = now_us;
  obs_loops++;
  unsigned long now = millis();
  if ((long)(now - obs_next_ms) < 0) return;
  unsigned long period = obs_enabled ? OBS_PERIOD_MS : OBS_OFF_PERIOD_MS;
  obs_next_ms += period;
  if ((long)(now - obs_next_ms) >= 0) obs_next_ms = now + period;  // 追いつけないときは今に合わせる（連射しない）
  if (websocket.connectedClients() == 0) return;                   // 接続が無ければ何もしない
  if ((long)(now - obs_pause_until_ms) < 0) return;                // 詰まった直後は休む

  obs_read_legs(now);  // 送る直前に脚の位置を読む（20 ms に 1 回）

  char* p = obs_buf + WEBSOCKETS_MAX_HEADER_SIZE;
  int n = snprintf(p, OBS_TEXT_MAX,
                   "{\"type\":\"obs\",\"t_ms\":%lu,\"pitch\":%.2f,\"pitch_rate\":%.2f,\"roll\":%.2f,"
                   "\"yaw\":%.2f,\"yaw_rate\":%.2f,\"wheel_l\":%.3f,\"wheel_r\":%.3f,"
                   "\"height_cmd\":%d,\"height_meas\":-1,\"battery_v\":%.2f,\"go\":%d,"
                   "\"fall\":%d,\"u\":%.3f,\"pitch_zero\":%.2f,\"send_us\":%lu,"
                   "\"loops\":%lu,\"loop_max_us\":%lu,"
                   "\"leg_pos_l\":%d,\"leg_pos_r\":%d,\"leg_us\":%lu}",
                   now, LQR_angle, LQR_gyro, (float)mpu6050.getAngleX(),
                   YAW_angle, YAW_gyro,
                   motor1.shaft_velocity * (2.0f * rp.m1_direction),   // 向きは ±0.5 なので ±1 に戻す
                   motor2.shaft_velocity * (2.0f * rp.m2_direction),
                   (int)wrobot.height, (float)rp.battery_voltage, wrobot.go ? 1 : 0,
                   rp.uncontrollable != 0 ? 1 : 0, LQR_u, angle_zeropoint, obs_last_send_us,
                   obs_loops, obs_loop_max_us,
                   obs_leg_pos[0], obs_leg_pos[1], obs_leg_us);
  if (n <= 0 || n >= OBS_TEXT_MAX) return;  // 脚の 3 欄で 400 → 480（いちばん長い値でも 400 文字ほど）
  obs_loops = 0;
  obs_loop_max_us = 0;

  unsigned long t0 = micros();
  // headerToPayload = true のときは、ヘッダ用の空きを含む先頭（obs_buf）を渡し、長さは本文だけ
  websocket.broadcastTXT((uint8_t*)obs_buf, (size_t)n, true);
  obs_last_send_us = micros() - t0;
  if (obs_last_send_us > OBS_SLOW_US) {
    obs_pause_until_ms = now + OBS_PAUSE_MS;
  }
}
