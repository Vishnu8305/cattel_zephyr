#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>

/* ================= CONFIG ================= */
#define MPU6050_ADDR        0x68
#define I2C_NODE            DT_NODELABEL(i2c0)
#define GPS_UART_NODE       DT_NODELABEL(uart2)
#define LORA_UART_NODE      DT_NODELABEL(uart0)

#define NODE_ADDR           "0001"
#define DEST_ADDR           "0002"

#define PI_F                3.14159265358979323846f

/* Thread priorities: lower number = higher priority in Zephyr */
#define GPS_THREAD_PRIO     3
#define MPU_THREAD_PRIO     4
#define PKT_THREAD_PRIO     5
#define TX_THREAD_PRIO      5

#define STACK_SIZE          2048
#define LORA_MSG_LEN        320
#define LORA_MSGQ_DEPTH     4

/* ================= DEVICES ================= */
static const struct device *i2c_dev  = DEVICE_DT_GET(I2C_NODE);
static const struct device *gps_uart = DEVICE_DT_GET(GPS_UART_NODE);
static const struct device *lora_uart = DEVICE_DT_GET(LORA_UART_NODE);

/* ================= SHARED TELEMETRY ================= */
struct telemetry_data {
    float ax;
    float ay;
    float az;
    float pitch;
    float roll;
    int step_count;
    char posture[12];

    double lat;
    double lon;
    double speed_kmh;
    bool gps_valid;
};

static struct telemetry_data g_data;
static float prev_mag = 0.0f;

/* ================= RTOS OBJECTS ================= */
K_MUTEX_DEFINE(data_mutex);
K_SEM_DEFINE(start_sem, 0, 4);
K_MSGQ_DEFINE(lora_msgq, LORA_MSG_LEN, LORA_MSGQ_DEPTH, 4);

/* ================= THREAD STACKS ================= */
K_THREAD_STACK_DEFINE(gps_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(mpu_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(pkt_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(tx_stack, STACK_SIZE);

static struct k_thread gps_thread_data;
static struct k_thread mpu_thread_data;
static struct k_thread pkt_thread_data;
static struct k_thread tx_thread_data;

/* ================= GPS STATE ================= */
static char gps_buffer[128];
static int gps_index = 0;

/* ================= HELPERS ================= */
static void to_hex(const char *in, char *out)
{
    static const char *hex = "0123456789ABCDEF";

    while (*in) {
        unsigned char c = (unsigned char)*in++;
        *out++ = hex[(c >> 4) & 0x0F];
        *out++ = hex[c & 0x0F];
    }
    *out = '\0';
}

static double gps_convert_to_decimal(double raw)
{
    int deg = (int)(raw / 100.0);
    double min = raw - ((double)deg * 100.0);
    return deg + (min / 60.0);
}

static void lora_send_raw(const char *cmd)
{
    size_t len = strlen(cmd);
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(lora_uart, cmd[i]);
    }
    uart_poll_out(lora_uart, '\r');
    uart_poll_out(lora_uart, '\n');
}

/* ================= MPU ================= */
static void mpu_init(void)
{
    uint8_t cfg[2] = {0x6B, 0x00};
    (void)i2c_write(i2c_dev, cfg, 2, MPU6050_ADDR);
}

static bool mpu_read(float *ax, float *ay, float *az)
{
    uint8_t reg = 0x3B;
    uint8_t data[6];

    if (i2c_write_read(i2c_dev, MPU6050_ADDR, &reg, 1, data, 6) != 0) {
        return false;
    }

    int16_t ax_raw = (int16_t)((data[0] << 8) | data[1]);
    int16_t ay_raw = (int16_t)((data[2] << 8) | data[3]);
    int16_t az_raw = (int16_t)((data[4] << 8) | data[5]);

    *ax = (float)ax_raw / 16384.0f;
    *ay = (float)ay_raw / 16384.0f;
    *az = (float)az_raw / 16384.0f;
    return true;
}

/* ================= GPS ================= */
static void parse_gps_sentence(char *s)
{
    char local_copy[128];
    strncpy(local_copy, s, sizeof(local_copy) - 1);
    local_copy[sizeof(local_copy) - 1] = '\0';

    char *t = NULL;
    int f = 0;

    if (strstr(local_copy, "GGA")) {
        double la = 0.0, lo = 0.0;
        char lat_dir = 'N', lon_dir = 'E';

        t = strtok(local_copy, ",");
        while (t) {
            f++;
            if (f == 3) la = atof(t);
            if (f == 4 && t[0] != '\0') lat_dir = t[0];
            if (f == 5) lo = atof(t);
            if (f == 6 && t[0] != '\0') lon_dir = t[0];
            t = strtok(NULL, ",");
        }

        if (la != 0.0 && lo != 0.0) {
            double conv_lat = gps_convert_to_decimal(la);
            double conv_lon = gps_convert_to_decimal(lo);

            if (lat_dir == 'S') conv_lat *= -1.0;
            if (lon_dir == 'W') conv_lon *= -1.0;

            k_mutex_lock(&data_mutex, K_FOREVER);
            g_data.lat = conv_lat;
            g_data.lon = conv_lon;
            g_data.gps_valid = true;
            k_mutex_unlock(&data_mutex);
        }
    } else if (strstr(local_copy, "RMC")) {
        double speed_kmh = 0.0;

        t = strtok(local_copy, ",");
        while (t) {
            f++;
            if (f == 8) {
                double knots = atof(t);
                speed_kmh = knots * 1.852;
            }
            t = strtok(NULL, ",");
        }

        k_mutex_lock(&data_mutex, K_FOREVER);
        g_data.speed_kmh = speed_kmh;
        k_mutex_unlock(&data_mutex);
    }
}

static void gps_read_loop(void)
{
    uint8_t c;

    while (uart_poll_in(gps_uart, &c) == 0) {
        if (c == '\n') {
            gps_buffer[gps_index] = '\0';
            parse_gps_sentence(gps_buffer);
            gps_index = 0;
        } else {
            if (gps_index < (int)sizeof(gps_buffer) - 1) {
                gps_buffer[gps_index++] = (char)c;
            } else {
                gps_index = 0;
            }
        }
    }
}

/* ================= THREADS ================= */
static void gps_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    k_sem_take(&start_sem, K_FOREVER);

    while (1) {
        gps_read_loop();
        k_msleep(10);
    }
}

static void mpu_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    k_sem_take(&start_sem, K_FOREVER);

    while (1) {
        float ax, ay, az;
        if (mpu_read(&ax, &ay, &az)) {
            float mag = sqrtf(ax * ax + ay * ay + az * az);

            if (fabsf(mag - prev_mag) > 0.30f) {
                k_mutex_lock(&data_mutex, K_FOREVER);
                g_data.step_count++;
                k_mutex_unlock(&data_mutex);
            }
            prev_mag = mag;

            float pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * 180.0f / PI_F;
            float roll  = atan2f(ay, sqrtf(ax * ax + az * az)) * 180.0f / PI_F;

            const char *posture;
            if (fabsf(pitch) < 25.0f) {
                posture = "Standing";
            } else if (fabsf(pitch) < 70.0f) {
                posture = "Sitting";
            } else {
                posture = "Lying";
            }

            k_mutex_lock(&data_mutex, K_FOREVER);
            g_data.ax = ax;
            g_data.ay = ay;
            g_data.az = az;
            g_data.pitch = pitch;
            g_data.roll = roll;
            strncpy(g_data.posture, posture, sizeof(g_data.posture) - 1);
            g_data.posture[sizeof(g_data.posture) - 1] = '\0';
            k_mutex_unlock(&data_mutex);
        }

        /* Faster than your old 3-second loop; this is why RTOS matters */
        k_msleep(100);
    }
}

static void packet_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    k_sem_take(&start_sem, K_FOREVER);

    while (1) {
        struct telemetry_data snap;
        char payload[160];
        char payload_hex[320];
        char atcmd[LORA_MSG_LEN];

        k_mutex_lock(&data_mutex, K_FOREVER);
        snap = g_data;
        k_mutex_unlock(&data_mutex);

        snprintf(payload, sizeof(payload),
                 "{src:%s,lat:%.6f,lon:%.6f,spd:%.2f,steps:%d,post:%s}\n",
                 NODE_ADDR,
                 snap.lat,
                 snap.lon,
                 snap.speed_kmh,
                 snap.step_count,
                 snap.posture[0] ? snap.posture : "Unknown");

        to_hex(payload, payload_hex);

        snprintf(atcmd, sizeof(atcmd),
                 "at+ab SendData %s %s",
                 DEST_ADDR,
                 payload_hex);

        /* Queue it instead of sending directly */
        (void)k_msgq_put(&lora_msgq, atcmd, K_NO_WAIT);

        k_msleep(3000);
    }
}

static void tx_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    char txbuf[LORA_MSG_LEN];

    k_sem_take(&start_sem, K_FOREVER);

    while (1) {
        if (k_msgq_get(&lora_msgq, txbuf, K_FOREVER) == 0) {
            lora_send_raw(txbuf);
        }
    }
}

/* ================= MAIN ================= */
int main(void)
{
    if (!device_is_ready(i2c_dev) ||
        !device_is_ready(gps_uart) ||
        !device_is_ready(lora_uart)) {
        return 0;
    }

    memset(&g_data, 0, sizeof(g_data));
    strncpy(g_data.posture, "Unknown", sizeof(g_data.posture) - 1);

    mpu_init();

    k_thread_create(&gps_thread_data, gps_stack, STACK_SIZE,
                    gps_thread, NULL, NULL, NULL,
                    GPS_THREAD_PRIO, 0, K_NO_WAIT);

    k_thread_create(&mpu_thread_data, mpu_stack, STACK_SIZE,
                    mpu_thread, NULL, NULL, NULL,
                    MPU_THREAD_PRIO, 0, K_NO_WAIT);

    k_thread_create(&pkt_thread_data, pkt_stack, STACK_SIZE,
                    packet_thread, NULL, NULL, NULL,
                    PKT_THREAD_PRIO, 0, K_NO_WAIT);

    k_thread_create(&tx_thread_data, tx_stack, STACK_SIZE,
                    tx_thread, NULL, NULL, NULL,
                    TX_THREAD_PRIO, 0, K_NO_WAIT);

    /* Release all threads after init is complete */
    for (int i = 0; i < 4; i++) {
        k_sem_give(&start_sem);
    }

    while (1) {
        k_msleep(10000);
    }

    return 0;
}
