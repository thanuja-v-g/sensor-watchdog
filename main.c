#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* ---------- Hardware registers ---------- */
#define RCC_APB1ENR (*(volatile uint32_t *)0x40023840)
#define USART2_SR   (*(volatile uint32_t *)0x40004400)
#define USART2_DR   (*(volatile uint32_t *)0x40004404)
#define USART2_CR1  (*(volatile uint32_t *)0x4000440C)
#define SCB_AIRCR   (*(volatile uint32_t *)0xE000ED0C)   /* system reset control */

/* ---------- Settings ---------- */
#define INJECT_FAULT         1      /* 1 = make the sampling task hang on purpose (demo), 0 = normal run */
#define FAULT_AT_SAMPLE      12     /* hang after this many readings */
#define HEALTH_PERIOD_MS     1000   /* how often the health task checks the other tasks */
#define MAX_MISSED           2      /* checks in a row with no heartbeat before we reset */
#define HEALTH_REPORT_EVERY  4      /* print a health line every N checks */
#define QUEUE_TIMEOUT_MS     1000   /* processing task gives up waiting for data after this */

/* ---------- Serial output ---------- */
void send_char(char c)
{
    while ((USART2_SR & (1 << 7)) == 0) { }
    USART2_DR = c;
}

void send_text(const char *text)
{
    while (*text != 0)
    {
        send_char(*text);
        text++;
    }
}

void send_number(int n)
{
    char digits[10];
    int count = 0;

    if (n == 0)
    {
        send_char('0');
        return;
    }

    while (n > 0)
    {
        digits[count] = '0' + (n % 10);
        n = n / 10;
        count++;
    }

    while (count > 0)
    {
        count--;
        send_char(digits[count]);
    }
}

/* Several tasks print now, so lock the scheduler while a line is printed.
   Without this, two tasks can mix their characters together. */
static void log_begin(void) { vTaskSuspendAll(); }
static void log_end(void)   { (void)xTaskResumeAll(); }

/* ---------- Shared data ---------- */
int readings[10] = {25, 26, 27, 29, 31, 33, 32, 30, 28, 26};

QueueHandle_t reading_queue;

/* Heartbeats: each task adds 1 to its own counter every time it runs its loop.
   The health task watches these counters. If one stops moving, that task is stuck. */
static volatile uint32_t sampling_beat    = 0;
static volatile uint32_t processing_beat  = 0;
static volatile uint32_t samples_sent     = 0;
static volatile uint32_t samples_dropped  = 0;
static volatile uint32_t samples_processed = 0;

/* ---------- Recovery ---------- */
/* Ask the Cortex-M core for a full system reset (same as pressing the reset button) */
static void system_reset(void)
{
    SCB_AIRCR = 0x05FA0004;   /* key 0x05FA + SYSRESETREQ bit */
    while (1) { }
}

/* ---------- Tasks ---------- */
void sampling_task(void *param)
{
    (void)param;
    int index = 0;

    while (1)
    {
        int temp = readings[index];

        if (xQueueSend(reading_queue, &temp, 0) == pdTRUE)
        {
            samples_sent++;
        }
        else
        {
            samples_dropped++;   /* queue was full */
        }

        sampling_beat++;         /* "I am alive" */
        index = (index + 1) % 10;

#if INJECT_FAULT
        if (samples_sent == FAULT_AT_SAMPLE)
        {
            log_begin();
            send_text("[fault injected] sampling task is now stuck\r\n");
            log_end();
            while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }      /* simulated hang: no more heartbeats */
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void processing_task(void *param)
{
    (void)param;
    int window[4] = {0, 0, 0, 0};
    int w = 0;
    int temp;

    while (1)
    {
        if (xQueueReceive(reading_queue, &temp, pdMS_TO_TICKS(QUEUE_TIMEOUT_MS)) == pdTRUE)
        {
            window[w] = temp;
            w = (w + 1) % 4;
            samples_processed++;

            int average = (window[0] + window[1] + window[2] + window[3]) / 4;

            log_begin();
            send_text("temp = ");
            send_number(temp);
            send_text("  average = ");
            send_number(average);

            if (average > 30)
            {
                send_text("  ALERT: too hot");
            }
            send_text("\r\n");
            log_end();
        }
        else
        {
            /* No reading arrived in time: report it, but this task itself is healthy */
            log_begin();
            send_text("WARN: no sensor data\r\n");
            log_end();
        }

        processing_beat++;       /* "I am alive" */
    }
}

/* Health task: highest priority, wakes up once a second and checks the heartbeats.
   This is a software watchdog. */
void health_task(void *param)
{
    (void)param;
    uint32_t last_sampling = 0;
    uint32_t last_processing = 0;
    int missed_sampling = 0;
    int missed_processing = 0;
    int checks = 0;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(HEALTH_PERIOD_MS));

        uint32_t s = sampling_beat;
        uint32_t p = processing_beat;

        if (s == last_sampling)   { missed_sampling++; }   else { missed_sampling = 0; }
        if (p == last_processing) { missed_processing++; } else { missed_processing = 0; }

        last_sampling = s;
        last_processing = p;
        checks++;

        if (missed_sampling >= MAX_MISSED || missed_processing >= MAX_MISSED)
        {
            vTaskSuspendAll();   /* nothing else may print or run from here on */
            send_text("WATCHDOG: ");
            if (missed_sampling >= MAX_MISSED)
            {
                send_text("sampling task stalled ");
            }
            if (missed_processing >= MAX_MISSED)
            {
                send_text("processing task stalled ");
            }
            send_text("- resetting system\r\n");
            system_reset();
        }

        if (checks % HEALTH_REPORT_EVERY == 0)
        {
            log_begin();
            send_text("[health] OK  sent = ");
            send_number((int)samples_sent);
            send_text("  processed = ");
            send_number((int)samples_processed);
            send_text("  dropped = ");
            send_number((int)samples_dropped);
            send_text("  free heap = ");
            send_number((int)xPortGetFreeHeapSize());
            send_text("\r\n");
            log_end();
        }
    }
}

/* ---------- Start-up ---------- */
int main(void)
{
    RCC_APB1ENR |= (1 << 17);
    USART2_CR1 = (1 << 13) | (1 << 3);

    send_text("Sensor pipeline started\r\n");

    reading_queue = xQueueCreate(8, sizeof(int));

    xTaskCreate(health_task,     "health",  256, NULL, 3, NULL);   /* priority 3: highest, must always get to run */
    xTaskCreate(sampling_task,   "sample",  256, NULL, 2, NULL);   /* priority 2 */
    xTaskCreate(processing_task, "process", 256, NULL, 1, NULL);   /* priority 1 */

    vTaskStartScheduler();

    while (1) { }
}

void vApplicationStackOverflowHook(TaskHandle_t t, char *name)
{
    (void)t;
    (void)name;
    send_text("STACK OVERFLOW\r\n");
    while (1) { }
}

void vApplicationMallocFailedHook(void)
{
    send_text("MALLOC FAILED\r\n");
    while (1) { }
}