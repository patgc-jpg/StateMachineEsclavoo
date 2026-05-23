// ============================================================
// main.cpp — ESP32 ESCLAVO — Máquina de Garra (Claw Machine)
// ============================================================
// Periféricos en ESTE ESP (esclavo):
//   - Motor Y  (NEMA + DRV8825) — eje de profundidad
//   - Motor Z  (NEMA + DRV8825) — sube/baja la garra
//   - Motor C  (NEMA + DRV8825) — abre/cierra la garra
//   - 2 botones direccionales para eje Y (durante GAME)
//
// Señales del MAESTRO (entradas en este ESP):
//   MASTER_BEGIN (GPIO15): HIGH mientras maestro está en BEGIN → esclavo mueve Y al centro
//   SLAVE_TRIG   (GPIO23): HIGH = maestro pide secuencia de garra (bajar→cerrar→subir→ceroY→abrir)
//   SLAVE_DONE   (GPIO2):  SALIDA del esclavo → HIGH = terminé la fase actual
//
// Protocolo BEGIN:
//   maestro pone BEGIN=HIGH → esclavo mueve Y al centro → maestro baja BEGIN → esclavo va a GAME_Y
//
// Protocolo GARRA:
//   maestro pone TRIG=HIGH → esclavo ejecuta secuencia completa → esclavo pone DONE=HIGH
//   → maestro pone TRIG=LOW → esclavo pone DONE=LOW → regresa a IDLE
// ============================================================

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <driver/gpio.h>

#include "stepmotor.h"   // NEMA + DRV8825 (2 pines: STEP, DIR) — motores Y y Z
#include "stepper28.h"   // 28BYJ-48 + ULN2003 (4 pines)       — motor de garra
#include "dbutton.h"

// ============================================================
// CONSTANTES MECÁNICAS — ajustar según hardware real
// ============================================================
#define MM_PER_STEP         0.2f

// Eje Y
#define Y_AXIS_LENGTH_MM    550.0f
#define Y_MAX_STEPS         ((int32_t)(Y_AXIS_LENGTH_MM / MM_PER_STEP))  // 2750
#define Y_CENTER_STEPS      (Y_MAX_STEPS / 2)                            // 1375
#define Y_HOME_STEPS        0

// Eje Z (carro vertical)
#define Z_MAX_STEPS         1000   // pasos para bajar completamente — ajustar
#define Z_HOME_STEPS        0

// Motor de garra (cierre)
#define CLAW_CLOSE_STEPS    300    // pasos para cerrar completamente — ajustar
#define CLAW_OPEN_STEPS     0

// ============================================================
// VELOCIDAD
// ============================================================
#define STEP_DELAY_GAME_US  1000UL   // velocidad Y durante el juego
#define STEP_DELAY_TRAV_US  1000UL   // velocidad en secuencias automáticas

// ============================================================
// DEBOUNCE
// ============================================================
#define DEBOUNCE_US  40000UL

// ============================================================
// PINES GPIO
// ============================================================

// Motor Y
#define MY_STEP  GPIO_NUM_19
#define MY_DIR   GPIO_NUM_18

// Motor Z (sube/baja garra)
#define MZ_STEP  GPIO_NUM_4
#define MZ_DIR   GPIO_NUM_13

// Motor C (abre/cierra garra) — 28BYJ-48 con ULN2003, 4 pines de bobina
#define MC_P0  GPIO_NUM_14
#define MC_P1  GPIO_NUM_27
#define MC_P2  GPIO_NUM_16
#define MC_P3  GPIO_NUM_17

// Botones direccionales Y — pull-up: presionado = LOW (conectar a GND)
#define BTN_FWD  GPIO_NUM_25
#define BTN_BWD  GPIO_NUM_26

// Señales del maestro
#define MASTER_BEGIN  GPIO_NUM_15   // ENTRADA: HIGH = mover Y al centro
#define SLAVE_TRIG    GPIO_NUM_23   // ENTRADA: HIGH = ejecutar secuencia de garra
#define SLAVE_DONE    GPIO_NUM_2    // SALIDA:  HIGH = fase completada

// ============================================================
// MÁQUINA DE ESTADOS
// ============================================================
enum State {
    STATE_IDLE,        // Espera señal BEGIN del maestro
    STATE_BEGIN_Y,     // Mueve Y al centro (paralelo al BEGIN del maestro en X)
    STATE_GAME_Y,      // Botones controlan Y; espera TRIG para secuencia de garra
    STATE_CLAW_DOWN,   // Baja Z a fondo
    STATE_CLOSE_CLAW,  // Cierra garra
    STATE_CLAW_UP,     // Sube Z a home
    STATE_ZERO_Y,      // Regresa Y a home
    STATE_OPEN_CLAW,   // Abre garra
    STATE_DONE,        // Señala DONE al maestro y espera confirmación
    NUM_STATES
};
typedef State (*StateAction)();
struct StateNode { const char *name; StateAction on_loop; };

// ============================================================
// OBJETOS DE HARDWARE
// ============================================================
static StepperMotor    motorY(MY_STEP, MY_DIR, STEP_DELAY_GAME_US);
static StepperMotor    motorZ(MZ_STEP, MZ_DIR, STEP_DELAY_TRAV_US);
static Stepper28BYJ    motorC(MC_P0, MC_P1, MC_P2, MC_P3, STEP_DELAY_TRAV_US);
static DebouncedButton btnFwd(BTN_FWD, DEBOUNCE_US);
static DebouncedButton btnBwd(BTN_BWD, DEBOUNCE_US);

// ============================================================
// VARIABLES GLOBALES
// ============================================================
static bool    is_new_state   = true;
static int64_t state_start_us = 0;

static int32_t y_steps    = 0;
static int32_t z_steps    = 0;
static int32_t claw_steps = 0;

// ============================================================
// HELPERS
// ============================================================
static void onEnterState() {
    state_start_us = esp_timer_get_time();
}

static void stepY(int dir) {
    motorY.setDirection(dir > 0);
    if (motorY.update()) y_steps += (dir > 0) ? 1 : -1;
}

static void stepZ(int dir) {
    motorZ.setDirection(dir > 0);
    if (motorZ.update()) z_steps += (dir > 0) ? 1 : -1;
}

static void stepC(int dir) {
    motorC.setDirection(dir > 0);
    if (motorC.update()) claw_steps += (dir > 0) ? 1 : -1;
}

// ============================================================
// ESTADOS
// ============================================================

// IDLE — Espera que el maestro active MASTER_BEGIN para sincronizar el inicio
static State executeIdle() {
    if (is_new_state) onEnterState();

    if (gpio_get_level(MASTER_BEGIN) == 1)
        return STATE_BEGIN_Y;

    return STATE_IDLE;
}

// BEGIN_Y — Mueve Y al centro en paralelo con el maestro moviendo X al centro.
// Ambos se mueven simultáneamente; el esclavo espera al maestro si termina primero.
static State executeBeginY() {
    if (is_new_state) {
        onEnterState();
        motorY.setDelay(STEP_DELAY_TRAV_US);
    }

    // Seguir moviendo si no llegó al centro
    if      (y_steps < Y_CENTER_STEPS) { stepY(+1); return STATE_BEGIN_Y; }
    else if (y_steps > Y_CENTER_STEPS) { stepY(-1); return STATE_BEGIN_Y; }

    // Y en el centro — espera a que el maestro también llegue (BEGIN=LOW) para ir juntos a GAME
    if (gpio_get_level(MASTER_BEGIN) == 0)
        return STATE_GAME_Y;

    return STATE_BEGIN_Y;
}

// GAME_Y — Botones controlan Y libremente; espera TRIG del maestro para bajar la garra
static State executeGameY() {
    if (is_new_state) {
        onEnterState();
        motorY.setDelay(STEP_DELAY_GAME_US);
    }

    if (btnFwd.isPressed()) {
        if (y_steps < Y_MAX_STEPS) stepY(+1);
    } else if (btnBwd.isPressed()) {
        if (y_steps > Y_HOME_STEPS) stepY(-1);
    }

    if (gpio_get_level(SLAVE_TRIG) == 1)
        return STATE_CLAW_DOWN;

    return STATE_GAME_Y;
}

// CLAW_DOWN — Baja Z hasta el fondo
static State executeClawDown() {
    if (is_new_state) {
        onEnterState();
        motorZ.setDelay(STEP_DELAY_TRAV_US);
    }

    if (z_steps < Z_MAX_STEPS) { stepZ(+1); return STATE_CLAW_DOWN; }
    return STATE_CLOSE_CLAW;
}

// CLOSE_CLAW — Cierra la garra
static State executeCloseClaw() {
    if (is_new_state) {
        onEnterState();
        motorC.setDelay(STEP_DELAY_TRAV_US);
    }

    if (claw_steps < CLAW_CLOSE_STEPS) { stepC(+1); return STATE_CLOSE_CLAW; }
    return STATE_CLAW_UP;
}

// CLAW_UP — Sube Z de vuelta a home
static State executeClawUp() {
    if (is_new_state) {
        onEnterState();
        motorZ.setDelay(STEP_DELAY_TRAV_US);
    }

    if (z_steps > Z_HOME_STEPS) { stepZ(-1); return STATE_CLAW_UP; }
    return STATE_DONE;   // garra arriba → avisar al maestro para regresar juntos
}

// DONE — Señala al maestro que la garra subió.
// Maestro baja TRIG y empieza ZERO_X; esclavo empieza ZERO_Y al mismo tiempo.
static State executeDone() {
    if (is_new_state) {
        onEnterState();
        gpio_set_level(SLAVE_DONE, 1);
        printf("[SLAVE] Garra arriba — regresando a home con el maestro\n");
    }

    if (gpio_get_level(SLAVE_TRIG) == 0) {
        gpio_set_level(SLAVE_DONE, 0);
        return STATE_ZERO_Y;   // maestro empieza ZERO_X, esclavo empieza ZERO_Y simultáneamente
    }
    return STATE_DONE;
}

// ZERO_Y — Regresa Y a home en paralelo con el maestro regresando X a home
static State executeZeroY() {
    if (is_new_state) {
        onEnterState();
        motorY.setDelay(STEP_DELAY_TRAV_US);
    }

    if      (y_steps > Y_HOME_STEPS) { stepY(-1); return STATE_ZERO_Y; }
    else if (y_steps < Y_HOME_STEPS) { stepY(+1); return STATE_ZERO_Y; }
    return STATE_OPEN_CLAW;
}

// OPEN_CLAW — Abre la garra una vez que Y llegó a home
static State executeOpenClaw() {
    if (is_new_state) {
        onEnterState();
        motorC.setDelay(STEP_DELAY_TRAV_US);
    }

    if (claw_steps > CLAW_OPEN_STEPS) { stepC(-1); return STATE_OPEN_CLAW; }
    return STATE_IDLE;   // garra abierta, listo para la siguiente ronda
}

// ============================================================
// TABLA DE ESTADOS
// ============================================================
static StateNode state_table[NUM_STATES] = {
    { "IDLE",       executeIdle       },
    { "BEGIN_Y",    executeBeginY     },
    { "GAME_Y",     executeGameY      },
    { "CLAW_DOWN",  executeClawDown   },
    { "CLOSE_CLAW", executeCloseClaw  },
    { "CLAW_UP",    executeClawUp     },
    { "ZERO_Y",     executeZeroY      },
    { "OPEN_CLAW",  executeOpenClaw   },
    { "DONE",       executeDone       },
};

// ============================================================
// INICIALIZACIÓN DE GPIO
// ============================================================
static void setupGPIO() {
    // SLAVE_DONE — salida, arranca en LOW
    gpio_reset_pin(SLAVE_DONE);
    gpio_set_direction(SLAVE_DONE, GPIO_MODE_OUTPUT);
    gpio_set_level(SLAVE_DONE, 0);

    // SLAVE_TRIG — entrada con pull-down (evita disparo en flotante)
    gpio_reset_pin(SLAVE_TRIG);
    gpio_set_direction(SLAVE_TRIG, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SLAVE_TRIG, GPIO_PULLDOWN_ONLY);

    // MASTER_BEGIN — entrada con pull-down
    gpio_reset_pin(MASTER_BEGIN);
    gpio_set_direction(MASTER_BEGIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(MASTER_BEGIN, GPIO_PULLDOWN_ONLY);
}

// ============================================================
// ENTRADA PRINCIPAL
// ============================================================
extern "C" void app_main() {
    esp_task_wdt_deinit();

    setupGPIO();

    motorY.begin();
    motorZ.begin();
    motorC.begin();

    btnFwd.init();
    btnBwd.init();

    printf("[SLAVE] Maquina de garra esclavo iniciada\n");

    State current_state = STATE_IDLE;
    State last_state    = STATE_IDLE;

    while (true) {
        if (state_table[current_state].on_loop)
            current_state = state_table[current_state].on_loop();

        if (current_state != last_state) {
            printf("[SLAVE] Estado: %s\n", state_table[current_state].name);
            last_state   = current_state;
            is_new_state = true;
        } else {
            is_new_state = false;
        }
    }
}
