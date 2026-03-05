#ifndef SENSOR_H
#define SENSOR_H

#include <MAX3010x.h>
#include "filters.h"
#include <Wire.h>
#include "Display.h"
#include "BLEHandler.h"

MAX30102 sensor;

/* ================= CONFIGURAÇÕES ================= */

const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

const unsigned long kFingerThreshold = 15000;
const unsigned int kFingerCooldownMs = 500;

const float kEdgeThreshold = -1500.0;

/* ================= FILTROS ================= */

LowPassFilter low_pass_red(5.0, kSamplingFrequency);
LowPassFilter low_pass_ir(5.0, kSamplingFrequency);

HighPassFilter high_pass_red(0.5, kSamplingFrequency);

Differentiator differentiator(kSamplingFrequency);

MovingAverageFilter<12> averager_bpm;
MovingAverageFilter<25> averager_spo2;

/* ================= ESTATÍSTICAS ================= */

MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

/* ================= VARIÁVEIS ================= */

long last_heartbeat = 0;
long finger_timestamp = 0;

bool finger_detected = false;

float last_diff = NAN;

bool crossed = false;
long crossed_time = 0;

unsigned long last_display_update = 0;

const unsigned long kDisplayInterval = 1000;

int beat_counter = 0;
const int beats_per_stat_reset = 6;

/* ================= FUNÇÕES EXTERNAS ================= */

extern void update_sensor_values(int bpm, float spo2);

/* ================= SCANNER I2C ================= */

void scan_i2c()
{
    Serial.println("Escaneando I2C...");

    byte error, address;
    int nDevices = 0;

    for(address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if(error == 0)
        {
            Serial.print("Dispositivo encontrado em 0x");
            Serial.println(address, HEX);
            nDevices++;
        }
    }

    if(nDevices == 0)
        Serial.println("Nenhum dispositivo encontrado");
    else
        Serial.println("Scan concluído");
}

/* ================= SETUP SENSOR ================= */

void sensor_setup()
{
    Wire.begin(21,22);

    scan_i2c();

    if(sensor.begin() && sensor.setSamplingRate(kSamplingRate))
    {
        Serial.println("MAX30102 iniciado com sucesso");
    }
    else
    {
        Serial.println("Erro ao iniciar MAX30102");
        while(true);
    }
}

/* ================= CÁLCULO SpO2 ================= */

float calcularSpO2()
{
    float ac_red = stat_red.maximum() - stat_red.minimum();
    float ac_ir  = stat_ir.maximum()  - stat_ir.minimum();

    float dc_red = stat_red.average();
    float dc_ir  = stat_ir.average();

    if(dc_red < 1 || dc_ir < 1) return NAN;

    float r = (ac_red / dc_red) / (ac_ir / dc_ir);

    if(r < 0.2 || r > 1.5) return NAN;

    float spo2 = 110.0 - 25.0 * r;

    spo2 = constrain(spo2, 90.0, 100.0);

    return spo2;
}

/* ================= LOOP SENSOR ================= */

void sensor_loop()
{
    auto sample = sensor.readSample(1000);

    float red = sample.red;
    float ir  = sample.ir;

    /* ===== DETECÇÃO DE DEDO ===== */

    if(red > kFingerThreshold)
    {
        if(millis() - finger_timestamp > kFingerCooldownMs)
            finger_detected = true;
    }
    else
    {
        differentiator.reset();
        averager_bpm.reset();
        averager_spo2.reset();

        low_pass_red.reset();
        low_pass_ir.reset();

        high_pass_red.reset();

        stat_red.reset();
        stat_ir.reset();

        finger_detected = false;

        finger_timestamp = millis();

        beat_counter = 0;

        return;
    }

    if(!finger_detected) return;

    /* ===== FILTROS ===== */

    red = low_pass_red.process(red);
    ir  = low_pass_ir.process(ir);

    stat_red.process(red);
    stat_ir.process(ir);

    float filtered = high_pass_red.process(red);

    float diff = differentiator.process(filtered);

    /* ===== DETECÇÃO DE BATIMENTO ===== */

    if(!isnan(diff) && !isnan(last_diff))
    {
        if(last_diff > 0 && diff < 0)
        {
            crossed = true;
            crossed_time = millis();
        }

        if(diff > 0) crossed = false;

        if(crossed && diff < kEdgeThreshold)
        {
            if(last_heartbeat != 0 && crossed_time - last_heartbeat > 300)
            {
                int bpm = 60000 / (crossed_time - last_heartbeat);

                if(bpm < 40 || bpm > 200)
                {
                    crossed = false;
                    last_heartbeat = crossed_time;
                    return;
                }

                int avg_bpm = averager_bpm.process(bpm);

                float spo2 = calcularSpO2();

                if(isnan(spo2))
                {
                    crossed = false;
                    last_heartbeat = crossed_time;
                    return;
                }

                float avg_spo2 = averager_spo2.process(spo2);

                /* ===== DISPLAY ===== */

                if(millis() - last_display_update > kDisplayInterval)
                {
                    last_display_update = millis();

                    update_sensor_values(avg_bpm, avg_spo2);
                }

                beat_counter++;

                if(beat_counter >= beats_per_stat_reset)
                {
                    stat_red.reset();
                    stat_ir.reset();
                    beat_counter = 0;
                }
            }

            crossed = false;

            last_heartbeat = crossed_time;
        }
    }

    last_diff = diff;
}

#endif
