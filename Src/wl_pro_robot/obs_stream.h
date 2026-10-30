#pragma once
// 観測の口（obs-stream）：20 ms ごとに観測を 1 行の JSON にして WebSocket（ポート 81）に流す。
// navbot-agent 設計書 §3.1・§4.1、01_PREFLIGHT §2・§3。制御則・周期・操作の口は変えない。
//
// 送る 1 通（欄の名前・単位・順は PC 側 yose/record.py の ROBOT_FIELDS と同じ）：
// {"type":"obs","t_ms":…,"pitch":…,"pitch_rate":…,"roll":…,"yaw":…,"yaw_rate":…,"wheel_l":…,"wheel_r":…,
//  "height_cmd":…,"height_meas":-1,"battery_v":…,"go":…,"fall":…,"u":…,"pitch_zero":…,"send_us":…}
//
// PC から {"type":"obs_ctl","on":0} を受けると 1 秒に 1 行だけ送る（送信 OFF の比べ用）。"on":1 で戻す。
#include <Arduino.h>
#include <WebSocketsServer.h>
#include <SimpleFOC.h>
#include <MPU6050_tockn.h>
#include "robot.h"

#define OBS_PERIOD_MS 20        // 送信の周期（50 Hz）
#define OBS_OFF_PERIOD_MS 1000  // 送信 OFF のときの周期（1 Hz）
#define OBS_SLOW_US 3000        // 1 回の送信がこれより長くかかったら
#define OBS_PAUSE_MS 1000       // この時間だけ送信を止める（TCP が詰まった時に制御の輪を守る）

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
static char obs_buf[WEBSOCKETS_MAX_HEADER_SIZE + 320];  // 先頭にヘッダ分の空きを置き、TCP の書き込みを 1 回にする

inline void obs_ctl(bool on) {
  obs_enabled = on;
}

// loop() の末尾で毎回呼ぶ。20 ms たっていなければすぐ返る。
inline void obs_stream_loop() {
  unsigned long now = millis();
  if ((long)(now - obs_next_ms) < 0) return;
  unsigned long period = obs_enabled ? OBS_PERIOD_MS : OBS_OFF_PERIOD_MS;
  obs_next_ms += period;
  if ((long)(now - obs_next_ms) >= 0) obs_next_ms = now + period;  // 追いつけないときは今に合わせる（連射しない）
  if (websocket.connectedClients() == 0) return;                   // 接続が無ければ何もしない
  if ((long)(now - obs_pause_until_ms) < 0) return;                // 詰まった直後は休む

  char* p = obs_buf + WEBSOCKETS_MAX_HEADER_SIZE;
  int n = snprintf(p, 320,
                   "{\"type\":\"obs\",\"t_ms\":%lu,\"pitch\":%.2f,\"pitch_rate\":%.2f,\"roll\":%.2f,"
                   "\"yaw\":%.2f,\"yaw_rate\":%.2f,\"wheel_l\":%.3f,\"wheel_r\":%.3f,"
                   "\"height_cmd\":%d,\"height_meas\":-1,\"battery_v\":%.2f,\"go\":%d,"
                   "\"fall\":%d,\"u\":%.3f,\"pitch_zero\":%.2f,\"send_us\":%lu}",
                   now, LQR_angle, LQR_gyro, (float)mpu6050.getAngleX(),
                   YAW_angle, YAW_gyro,
                   motor1.shaft_velocity * (2.0f * rp.m1_direction),   // 向きは ±0.5 なので ±1 に戻す
                   motor2.shaft_velocity * (2.0f * rp.m2_direction),
                   (int)wrobot.height, (float)rp.battery_voltage, wrobot.go ? 1 : 0,
                   rp.uncontrollable != 0 ? 1 : 0, LQR_u, angle_zeropoint, obs_last_send_us);
  if (n <= 0 || n >= 320) return;

  unsigned long t0 = micros();
  // headerToPayload = true のときは、ヘッダ用の空きを含む先頭（obs_buf）を渡し、長さは本文だけ
  websocket.broadcastTXT((uint8_t*)obs_buf, (size_t)n, true);
  obs_last_send_us = micros() - t0;
  if (obs_last_send_us > OBS_SLOW_US) {
    obs_pause_until_ms = now + OBS_PAUSE_MS;
  }
}
