#pragma once
// 観測の口（obs-stream）：20 ms ごとに観測を 1 行の JSON にして WebSocket（ポート 81）に流す。
// navbot-agent 設計書 §3.1・§4.1、01_PREFLIGHT §2・§3。制御則・周期・操作の口は変えない。
//
// 送る 1 通（欄の名前・単位・順は PC 側 yose/record.py の ROBOT_FIELDS と同じ）：
// {"type":"obs","t_ms":…,"pitch":…,"pitch_rate":…,"roll":…,"yaw":…,"yaw_rate":…,"wheel_l":…,"wheel_r":…,
//  "height_cmd":…,"height_meas":-1,"battery_v":…,"go":…,"fall":…,"u":…,"pitch_zero":…,"send_us":…,
//  "loops":…,"loop_max_us":…}
// loops＝前の 1 通から今までに loop() が回った回数、loop_max_us＝その間でいちばん長かった 1 回の時間（µs）。
// ファームの制御の輪が 1 秒に何回回っているかを測るためだけの欄（2026-10-31 robosheep の了承、FIRMWARE_DIFF §3e）。
// 制御には触らない。PC 側は記録の 21 欄を変えず、thinking の層として残す。
//
// PC から {"type":"obs_ctl","on":0} を受けると 1 秒に 1 行だけ送る（送信 OFF の比べ用）。"on":1 で戻す。
#include <Arduino.h>
#include <WebSocketsServer.h>
#include <SimpleFOC.h>
#include <MPU6050_tockn.h>
#include "robot.h"

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
static char obs_buf[WEBSOCKETS_MAX_HEADER_SIZE + 400];  // 先頭にヘッダ分の空きを置き、TCP の書き込みを 1 回にする

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

  char* p = obs_buf + WEBSOCKETS_MAX_HEADER_SIZE;
  int n = snprintf(p, 400,
                   "{\"type\":\"obs\",\"t_ms\":%lu,\"pitch\":%.2f,\"pitch_rate\":%.2f,\"roll\":%.2f,"
                   "\"yaw\":%.2f,\"yaw_rate\":%.2f,\"wheel_l\":%.3f,\"wheel_r\":%.3f,"
                   "\"height_cmd\":%d,\"height_meas\":-1,\"battery_v\":%.2f,\"go\":%d,"
                   "\"fall\":%d,\"u\":%.3f,\"pitch_zero\":%.2f,\"send_us\":%lu,"
                   "\"loops\":%lu,\"loop_max_us\":%lu}",
                   now, LQR_angle, LQR_gyro, (float)mpu6050.getAngleX(),
                   YAW_angle, YAW_gyro,
                   motor1.shaft_velocity * (2.0f * rp.m1_direction),   // 向きは ±0.5 なので ±1 に戻す
                   motor2.shaft_velocity * (2.0f * rp.m2_direction),
                   (int)wrobot.height, (float)rp.battery_voltage, wrobot.go ? 1 : 0,
                   rp.uncontrollable != 0 ? 1 : 0, LQR_u, angle_zeropoint, obs_last_send_us,
                   obs_loops, obs_loop_max_us);
  if (n <= 0 || n >= 400) return;  // 欄を足したので 320 → 400（いちばん長い値でも 348 文字）
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
