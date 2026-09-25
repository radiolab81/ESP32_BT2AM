/*
 * ESP32 AM-Modulator, 1 Traeger (NCO), 8-Bit-Parallel-DAC via I2S0
 * ESP-IDF 6.1
 *
 * ==== Bluetooth-A2DP statt WLAN/UDP ====
 *   Kein WLAN mehr. Der ESP32 meldet sich als klassischer Bluetooth-A2DP-
 *   "Sink" (Audioempfaenger, so wie ein Bluetooth-Lautsprecher) an. Handy/
 *   PC/etc. verbinden sich per Bluetooth-Pairing und streamen Audio direkt
 *   auf den ESP32 - es wird kein externer PC/ffmpeg mehr benoetigt.
 *
 *   ==== Nur ein NCO (ein Traeger) ====
 *   Ein zweiter, parallel modulierter Traeger (frueher z.B. LW+MW gleich-
 *   zeitig) hat in dieser Anwendung wenig praktischen Nutzen und wurde
 *   entfernt. Der Code arbeitet jetzt mit GENAU EINEM NCO, dessen Frequenz
 *   im Startup-Code gesetzt wird (START_FREQ_HZ). Der dadurch frei
 *   gewordene Dynamikbereich des 8-Bit-DAC-Ausgangs kommt jetzt voll dem
 *   einen verbleibenden Traeger zugute (DAC_DIVISOR wurde entsprechend
 *   halbiert, siehe Kommentar dort) - das ermoeglicht einen hoeheren
 *   Modulationsindex/-hub als vorher im Zwei-Traeger-Betrieb.
 *
 *   Modulationsquelle bleibt das per Bluetooth empfangene Stereo-PCM-
 *   Signal, aus dem der L+R-Mittelwert als Mono-Modulationssample gebildet
 *   wird (siehe bt_app_a2d_data_cb() und Kommentar dort zur 16-kHz-
 *   Annahme).
 *
 *   Die Sendefrequenz wird nur noch EINMALIG im Startup-Code gesetzt
 *   (kein Port 5000 mehr, da kein IP-Netzwerk mehr existiert).
 *
 * ALLES WAS DIE EIGENTLICHE AM-ERZEUGUNG BETRIFFT (Sinus-LUT, NCO-Phasen-
 * akkumulatoren, I2S0-Parallel-DMA-Ausgabe, Q15-Integer-Mischung, DMA-Ring
 * mit EOF-Interrupt-Nachfuellung) ist UNVERAENDERT aus der UDP-Version
 * uebernommen - siehe Originalkommentare, die bewusst 1:1 stehen bleiben.
 *
 * KALIBRIERUNG:
 *   MOD_GAIN_Q15 und DAC_DIVISOR ggf. am Oszi/SDR nachjustieren, siehe
 *   Kommentare unten (analog zu den *_int-Variablen im PC-Referenzcode).
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_intr_alloc.h"
#include "esp_rom_gpio.h"
#include "esp_task_wdt.h"

#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/dport_reg.h"
#include "soc/soc.h"
#include "hal/gpio_ll.h"
#include "esp_private/periph_ctrl.h"

// ---- Bluetooth (klassisch, A2DP-Sink) ----
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"   // AVRCP-Controller, siehe Kommentar bei bt_app_avrc_ct_cb()

#define TAG "AM_NCO_I2S_BT"

// --- Bluetooth-Geraetename (erscheint beim Pairing auf Handy/PC) ---
#define BT_DEVICE_NAME  "ESP32-BT2AM"

// --- HF-/DAC-PARAMETER ---
#define FS_HZ              4000000UL   // Ziel-Ausgangsrate DAC: 4 MSPS (unveraendert)

// ---------------------------------------------------------------------
// Audio-Abtastrate ist bei Bluetooth-A2DP KEINE feste Konstante, sondern
// wird erst zur Laufzeit ausgehandelt. Bei A2DP/SBC verhandeln Quelle
// (Handy) und Senke (dieser ESP32) die Abtastrate aus einer gemeinsamen
// Werte-Menge:
//   16000 / 32000 / 44100 / 48000 Hz (SBC-Standard, siehe A2DP-Spezifikation)
// Der ESP-IDF-A2DP-Sink-Stack meldet dem Sink zwar ueber die o.g. Menge
// welche Rate ALLE gemeinsam unterstuetzen, bietet aber (Stand ESP-IDF
// 6.1) KEINE offizielle Public-API, mit der die Senke die Quelle zwingen
// kann, gezielt NUR 16 kHz anzubieten - die Auswahl aus der gemeinsamen
// Menge trifft letztlich das sendende Geraet (Handy/PC), das i.d.R.
// 44.1 kHz bevorzugt, wenn es das kann.
//
// Fuer "typische AM-Qualitaet" (Sprachbandbreite, kein Hi-Fi) waere
// 16 kHz zwar ideal (16 kHz IST eine der vier SBC-Standardraten), 
// lässt sich aber vom Sink aus nicht
// hart erzwingen. Der Code behandelt das pragmatisch: Er fragt beim
// Verbindungsaufbau die TATSAECHLICH ausgehandelte Rate ab
// (ESP_A2D_AUDIO_CFG_EVT, siehe a2d_event_cb) und rechnet ab dann live
// mit dieser Rate weiter (audio_sample_rate_hz, volatile). Damit
// funktioniert die AM-Erzeugung unabhaengig davon, ob am Ende 16, 32,
// 44.1 oder 48 kHz ankommen - Standardwert bis zum ersten Verbindungs-
// aufbau ist 16000 Hz (deine Zielannahme).
// ---------------------------------------------------------------------
static volatile uint32_t audio_sample_rate_hz = 16000UL;

// --- Pinbelegung (I2S0-Parallelmodus), unveraendert ---
static const int data_pins[8] = {
    GPIO_NUM_4,  GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_18,
    GPIO_NUM_19, GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_23
};
#define PCLK_PIN GPIO_NUM_25

// --- DMA-Deskriptor (identisch zum internen lldesc_t-Layout) ---
typedef struct lldesc_s {
    volatile uint32_t size  : 12;
    volatile uint32_t length: 12;
    volatile uint32_t offset: 5;
    volatile uint32_t sosf  : 1;
    volatile uint32_t eof   : 1;
    volatile uint32_t owner : 1;
    volatile const uint8_t *buf;
    volatile struct lldesc_s *empty;
} lldesc_t;

// --- Mehrsegment-Ring statt Einzelpuffer: jedes Segment wird laufend
//     nachgefuellt, sobald es die DMA "verlassen" hat (eof-Interrupt). ---
#define NUM_DESC          4
#define SAMPLES_PER_DESC  1000          // 1000 Samples @ 4 MS/s = 250 us / Segment
// 12-Bit-size-Feld der Deskriptoren: max. 4095 BYTE. Bei 16-Bit-Samples
// (Padding-Byte s. ESP32Siggen.c) macht 1000 Samples = 2000 Byte -> passt.

static uint16_t *dma_buf[NUM_DESC];      // je ein 16-Bit-Sample-Block pro Segment
static lldesc_t  dma_desc[NUM_DESC];

// Benachrichtigung ISR -> Fuell-Task: Bitmaske der fertig-gemeldeten Segmente
static TaskHandle_t fill_task_handle = NULL;

// ---------------------------------------------------------------------
// INT32-LUT-NCO: 32-Bit-Phasen-
// akkumulator, 12-Bit-Sinus-LUT (Q15, +-32767).
// UNVERAENDERT aus der UDP-Version.
// ---------------------------------------------------------------------
#define LUT_BITS  12
#define LUT_SIZE  (1 << LUT_BITS)
static int16_t sine_lut[LUT_SIZE];

typedef struct {
    // phase_inc wird nur EINMAL im Startup-Code gesetzt und danach nicht
    // mehr per Netzwerk aktualisiert. Bleibt "volatile", falls spaeter doch
    // mal eine Laufzeitaenderung
    // (z.B. per Taster/Bluetooth-AVRCP) gewuenscht wird.
    volatile uint32_t phase_inc;
    uint32_t phase;                // laeuft nur im Fuell-Task (Core 1)
} nco_t;
// nur noch EINE NCO-Instanz (frueher: nco[NUM_CH] fuer zwei Traeger)
static nco_t nco;

static void init_sine_lut(void)
{
    for (int i = 0; i < LUT_SIZE; i++) {
        sine_lut[i] = (int16_t)lroundf(sinf(2.0f * (float)M_PI * i / LUT_SIZE) * 32767.0f);
    }
}

static inline uint32_t freq_to_phase_inc(uint32_t freq_hz)
{
    // Word = (Freq / FS_HZ) * 2^32 - analog zur FPGA-Variante, nur hier
    // gegen die tatsaechliche I2S-Ausgaberate FS_HZ statt eines externen Takts.
    return (uint32_t)(((double)freq_hz / (double)FS_HZ) * 4294967296.0);
}

// ---------------------------------------------------------------------
// EIN PCM-Ringpuffer als Modulationsquelle fuer den einzigen NCO.
// Struktur/Verhalten (Zero-Order-Hold bei Jitter, Sync-Wartephase am
// Start, Wiedereinstieg in den Sync-Modus bei laengerem Underrun) ist
// 1:1 aus der urspruenglichen UDP-Version uebernommen.
// ---------------------------------------------------------------------
#define AUDIO_RING_SIZE   4096   // Potenz von 2 fuer Bitmask-Wrap
#define PREBUFFER_SAMPLES 120

// underrun_counter zaehlt in PCM-Sample-Einheiten der tatsaechlich
// ausgehandelten Bluetooth-Abtastrate hoch (s. audio_sample_rate_hz).
// 50 Samples @ 16 kHz ~= 3,1 ms echte Funkstille, bevor neu synchronisiert wird.
#define UNDERRUN_LIMIT    50

typedef struct {
    uint8_t  storage[AUDIO_RING_SIZE];
    volatile uint32_t head;   // von der Bluetooth-A2DP-Datencallback geschrieben
    volatile uint32_t tail;   // vom Fuell-Task/Core1 gelesen
    uint8_t  last_sample;     // Zero-Order-Hold-Backup, Mitte=128
    uint32_t underrun_counter;
    uint32_t rate_acc;        // Akkumulator fuer FS_HZ -> audio_sample_rate_hz Ratenwandlung
    bool     is_streaming;
} pcm_ring_t;

static pcm_ring_t pcm_ring;

// ---------------------------------------------------------------------
// DTMF-Erkennung ueber Goertzel-Algorithmus.
//
// Hintergrund: Wenn ueber Bluetooth Telefonate/Sprachmemos/Tastentoene
// (z.B. vom Waehltastenfeld eines Smartphones) uebertragen werden, sind
// darin haeufig DTMF-Toene enthalten. Diese werden hier per Goertzel-
// Algorithmus dekodiert (deutlich guenstiger als eine volle FFT, da nur
// die 8 relevanten DTMF-Frequenzen gezielt abgefragt werden). Wird die
// Sondersequenz "*#nnnn#" (n=Ziffer) erkannt, wird die NCO-Traegerfrequenz
// live auf "nnnn" kHz umgestellt - so kann die Sendefrequenz vom Handy aus
// per Tastenfeld ferngesteuert werden, ganz ohne WLAN/UDP.
//
// WICHTIG: Diese Erkennung laeuft bewusst NICHT auf Core 1 (dort laeuft
// die harte Echtzeit-DSP-Schleife fill_task() mit 4 Mio. Samples/s ohne
// jeden Slack) sondern in einem eigenen, niedrig priorisierten Task auf
// Core 0 (siehe dtmf_task()). Ein zweiter, von pcm_ring UNABHAENGIGER
// Ringpuffer mit voller 16-Bit-Aufloesung dient dabei als Zwischenspeicher
// zwischen dem Bluetooth-Audio-Hotpath (Schreiber: bt_app_a2d_data_cb(),
// laeuft im Bluedroid-Kontext) und dem DTMF-Task (Leser). Die vorhandene,
// bereits auf 8 Bit quantisierte pcm_ring bleibt davon unberuehrt - fuer
// eine zuverlaessige Tonerkennung ist die volle 16-Bit-Amplitudenaufloesung
// wichtig (8 Bit waere zu grob/verrauscht fuer die Goertzel-Leistungswerte).
// ---------------------------------------------------------------------
#define DTMF_SAMPLE_RING_SIZE  8192   // Potenz von 2 fuer Bitmask-Wrap (~185 ms @ 44,1 kHz)

typedef struct {
    int16_t  storage[DTMF_SAMPLE_RING_SIZE];
    volatile uint32_t head;   // Schreiber: bt_app_a2d_data_cb (Bluedroid-Kontext)
    volatile uint32_t tail;   // Leser: dtmf_task (Core 0, niedrige Prioritaet)
} dtmf_ring_t;
static dtmf_ring_t dtmf_ring;

// --- Debug-Telemetrie (Core1 -> Core0), je 1 Schreibzugriff/Segment (250us),
//     vernachlaessigbarer Overhead, kein printf() im Realtime-Pfad. ---
static volatile int32_t  dbg_last_audio;
// Spitzenwert (Betrag) des Modulationssamples seit dem letzten Debug-
// Print. Viel aussagekraeftiger als dbg_last_audio (das nur ein zufaelliges
// Einzelsample im Moment des Log-Prints zeigt) - besonders bei dynamischer
// Musik mit vielen leisen Passagen, wo ein Einzelsample oft zufaellig nahe
// Null liegt, obwohl an anderer Stelle im selben Sekundenfenster durchaus
// kraeftig ausgesteuert wurde. Wird im 1-Sekunden-Log-Loop (app_main)
// nach jeder Ausgabe wieder auf 0 zurueckgesetzt.
static volatile int32_t  dbg_audio_peak;
static volatile bool     dbg_is_streaming;
static volatile uint32_t dbg_ring_fill;
static volatile int32_t  dbg_last_mix;
// Zaehler fuer per Bluetooth empfangene Audio-Bytes/Pakete
static volatile uint32_t dbg_bt_packets;
static volatile uint64_t dbg_bt_bytes;

// ---------------------------------------------------------------------
// GPIO-Routing: D0..D7 -> I2S0O_DATA_OUT8..15, PCLK -> I2S0O_WS_OUT
// (siehe ESP32Siggen.c: per Logic-Analyzer verifizierte Zuordnung)
// UNVERAENDERT.
// ---------------------------------------------------------------------
static void route_parallel_gpios(void)
{
    static const int data_out_sig[8] = {
        I2S0O_DATA_OUT8_IDX,  I2S0O_DATA_OUT9_IDX,  I2S0O_DATA_OUT10_IDX, I2S0O_DATA_OUT11_IDX,
        I2S0O_DATA_OUT12_IDX, I2S0O_DATA_OUT13_IDX, I2S0O_DATA_OUT14_IDX, I2S0O_DATA_OUT15_IDX,
    };
    for (int i = 0; i < 8; i++) {
        gpio_reset_pin(data_pins[i]);
        gpio_set_direction(data_pins[i], GPIO_MODE_OUTPUT);
        esp_rom_gpio_connect_out_signal(data_pins[i], data_out_sig[i], false, false);
    }
    gpio_reset_pin(PCLK_PIN);
    gpio_set_direction(PCLK_PIN, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(PCLK_PIN, I2S0O_WS_OUT_IDX, false, false);
}

static void dma_reset(void)
{
    I2S0.lc_conf.in_rst = 1; I2S0.lc_conf.in_rst = 0;
    I2S0.lc_conf.out_rst = 1; I2S0.lc_conf.out_rst = 0;
}
static void fifo_reset(void)
{
    I2S0.conf.rx_fifo_reset = 1; I2S0.conf.rx_fifo_reset = 0;
    I2S0.conf.tx_fifo_reset = 1; I2S0.conf.tx_fifo_reset = 0;
}
static void dev_reset(void)
{
    fifo_reset();
    dma_reset();
    I2S0.conf.rx_reset = 1; I2S0.conf.tx_reset = 1;
    I2S0.conf.rx_reset = 0; I2S0.conf.tx_reset = 0;
}

// ---------------------------------------------------------------------
// I2S0-EOF-Interrupt: wird bei jedem abgeschlossenen Segment ausgeloest
// (Segment-eof=1 bei ALLEN Deskriptoren gesetzt, siehe init unten).
// Ermittelt den Index des soeben fertiggestellten Segments und
// benachrichtigt den Fuell-Task, damit er es sofort nachfuellen kann.
// UNVERAENDERT.
// ---------------------------------------------------------------------
static void IRAM_ATTR i2s_isr(void *arg)
{
    uint32_t status = I2S0.int_st.val;
    if (status & I2S_OUT_EOF_INT_ST) {
        lldesc_t *finished = (lldesc_t *)I2S0.out_eof_des_addr;
        int idx = (int)(finished - dma_desc);
        if (idx >= 0 && idx < NUM_DESC && fill_task_handle) {
            BaseType_t hpw = pdFALSE;
            xTaskNotifyFromISR(fill_task_handle, (1u << idx), eSetBits, &hpw);
            if (hpw) portYIELD_FROM_ISR();
        }
    }
    I2S0.int_clr.val = status; // alle anstehenden Interruptflags loeschen
}

static void init_i2s_parallel(void)
{
    periph_module_reset(PERIPH_I2S0_MODULE);
    periph_module_enable(PERIPH_I2S0_MODULE);

    route_parallel_gpios();

    // Zirkulaeren Deskriptorring aufbauen. Jedes Segment bekommt eof=1,
    // damit wir bei JEDEM Segmentende einen Interrupt zur Nachbefuellung
    // bekommen (anders als ESP32Siggen.c, wo eof durchgehend 0 war, weil
    // dort der Inhalt sich nie aenderte).
    for (int i = 0; i < NUM_DESC; i++) {
        dma_desc[i].size   = SAMPLES_PER_DESC * sizeof(uint16_t);
        dma_desc[i].length = SAMPLES_PER_DESC * sizeof(uint16_t);
        dma_desc[i].offset = 0;
        dma_desc[i].sosf   = (i == 0) ? 1 : 0;
        dma_desc[i].eof    = 1;
        dma_desc[i].owner  = 1;
        dma_desc[i].buf    = (const uint8_t *)dma_buf[i];
        dma_desc[i].empty  = &dma_desc[(i + 1) % NUM_DESC];
    }
    ESP_LOGI(TAG, "DMA-Ring: %d Segmente a %d Samples (%.1f us/Segment)",
             NUM_DESC, SAMPLES_PER_DESC, 1e6 * SAMPLES_PER_DESC / (double)FS_HZ);

    dev_reset();

    I2S0.conf2.val = 0;
    I2S0.conf2.lcd_en = 1;

    I2S0.sample_rate_conf.val = 0;
    I2S0.sample_rate_conf.rx_bits_mod = 8;
    I2S0.sample_rate_conf.tx_bits_mod = 8;
    I2S0.sample_rate_conf.rx_bck_div_num = 1;
    I2S0.sample_rate_conf.tx_bck_div_num = 1;

    I2S0.clkm_conf.val = 0;
    I2S0.clkm_conf.clka_en = 0;
    I2S0.clkm_conf.clkm_div_a = 0;
    I2S0.clkm_conf.clkm_div_b = 0;
    uint32_t clk_div_main = APB_CLK_FREQ / FS_HZ;
    if (clk_div_main < 2) clk_div_main = 2;
    if (clk_div_main > 0xFF) clk_div_main = 0xFF;
    I2S0.clkm_conf.clkm_div_num = clk_div_main;

    I2S0.fifo_conf.val = 0;
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    I2S0.fifo_conf.tx_fifo_mod_force_en = 1;
    I2S0.fifo_conf.tx_fifo_mod = 1;      // 16-Bit-Kanal, Sample + Zero-Padding
    I2S0.fifo_conf.rx_data_num = 32;
    I2S0.fifo_conf.tx_data_num = 32;
    I2S0.fifo_conf.dscr_en = 1;

    I2S0.conf1.val = 0;
    I2S0.conf1.tx_stop_en = 0;
    I2S0.conf1.tx_pcm_bypass = 1;

    I2S0.conf_chan.val = 0;
    I2S0.conf_chan.tx_chan_mod = 1;
    I2S0.conf_chan.rx_chan_mod = 1;

    I2S0.conf.tx_right_first = 0;
    I2S0.conf.rx_right_first = 0;
    I2S0.timing.val = 0;

    // EOF-Interrupt aktivieren, damit die kontinuierliche Nachbefuellung
    // funktioniert
    I2S0.int_ena.val = 0;
    I2S0.int_ena.out_eof = 1;
    I2S0.int_clr.val = 0xFFFFFFFF;

    esp_err_t err = esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_IRAM,
                                    i2s_isr, NULL, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_intr_alloc(I2S0) fehlgeschlagen: %d", err);
        abort();
    }

    I2S0.out_link.stop = 1;
    I2S0.out_link.start = 0;
    I2S0.conf.tx_start = 0;
    dev_reset();

    I2S0.lc_conf.val = 0;
    I2S0.lc_conf.out_data_burst_en = 1;
    I2S0.lc_conf.outdscr_burst_en = 1;

    I2S0.out_link.addr = (uint32_t)&dma_desc[0];
    I2S0.out_link.start = 1;
    I2S0.conf.tx_start = 1;

    ESP_LOGI(TAG, "I2S0 Parallelausgang gestartet: Ziel %lu Hz (Teiler=%lu)",
             (unsigned long)FS_HZ, (unsigned long)clk_div_main);
}

// ---------------------------------------------------------------------
// AM-Mischung: 1x INT32-LUT-NCO, integer Q15-Mathematik
// (analog zur schnellen Schleife in am_modulator_5MSPS_integer.c, hier
// aber ohne Upsampling-Faktor, da NCO direkt mit FS_HZ getaktet wird).
// GRUNDPRINZIP unveraendert aus der UDP-Version, jedoch von 2 auf 1
// Traeger reduziert (siehe Kopfkommentar).
// ---------------------------------------------------------------------
// Q15-Traegerbasis (1.0) und Modulationsverstaerkung.
#define CARRIER_BASE_Q15   16384       // Q15, entspricht "1.0" mit Headroom
#define MOD_GAIN_Q15       128         // Modulationsindex-Skalierung (kalibrieren!)
                                        // Bei audio=127 (Vollausschlag) ergibt das einen
                                        // Modulationsindex von 127*128/16384 = 0,99 (~99%).
                                        // dac_val wird unten weiterhin hart auf ±127
                                        // geclippt (Uebersteuerungsschutz) - falls bei
                                        // sehr lauten Passagen haerte Verzerrungen/Clipping
                                        // hoerbar/sichtbar werden, hier reduzieren.
// DAC_DIVISOR nicht mehr mit NUM_CH multipliziert (frueher 230*2=460),
// da nur noch EIN Traeger zur Summe "mix" beitraegt. Dadurch bekommt dieser
// eine Traeger jetzt den vollen 8-Bit-Dynamikbereich des DAC zugewiesen,
// statt sich ihn (ungenutzt) mit einem zweiten, evtl. abgeschalteten NCO
// zu teilen.
#define DAC_DIVISOR        230         // Skalierung auf 8 Bit (kalibrieren!)

// PERFORMANCE: Bei FS_HZ=4 MHz und 240 MHz CPU-Takt bleiben nur ~60 Zyklen
// Budget pro Ausgabesample. Eine 32-Bit-Integer-Division (mix/DAC_DIVISOR)
// kostet auf der Xtensa-LX6-CPU allein schon 20-40 Zyklen und hat den
// Fill-Task real hinter die 4-MHz-Echtzeitanforderung zuruckfallen lassen
// (zu wenige pcm_pull()-Aufrufe/s -> Ring lief voll -> Audio zu langsam).
// Fix: Division durch Festkomma-Reziprok-Multiplikation ersetzen.
#define DAC_DIV_SHIFT      24
#define DAC_DIV_RECIP      ((int32_t)(((int64_t)1 << DAC_DIV_SHIFT) / DAC_DIVISOR))

static inline uint8_t pcm_pull(pcm_ring_t *r)
{
    // RATENWANDLUNG (der entscheidende Fix): pcm_pull() wird mit FS_HZ
    // (4 MHz) aufgerufen, PCM kommt aber nur mit audio_sample_rate_hz an
    // (variabel, da Bluetooth-A2DP die
    // Abtastrate erst beim Verbindungsaufbau aushandelt - siehe Kommentar
    // oben bei "audio_sample_rate_hz"). Ohne Akkumulator wuerde ein
    // komplettes Audio-Paket in wenigen Mikrosekunden "leergesaugt" statt
    // ueber die korrekte Zeitspanne verteilt gehalten zu werden - der Ring
    // waere danach bis zum naechsten Paket faelschlich lange "leer" und
    // die Wiedergabe faellt in Stille zurueck.
    // Bresenham-Akkumulator: erst wenn genug "virtuelle Zeit" vergangen ist,
    // wird das naechste PCM-Byte gezogen; sonst wird last_sample gehalten.
    r->rate_acc += audio_sample_rate_hz;
    if (r->rate_acc < FS_HZ) {
        return r->last_sample;   // aktuelles PCM-Sample weiter halten (ZOH)
    }
    r->rate_acc -= FS_HZ;

    uint32_t fill = (r->head >= r->tail) ? (r->head - r->tail)
                                          : (AUDIO_RING_SIZE - (r->tail - r->head));
    if (!r->is_streaming) {
        if (fill >= PREBUFFER_SAMPLES) {
            r->is_streaming = true;
            r->underrun_counter = 0;
        } else {
            return 128; // Stille bis Vorspannen abgeschlossen
        }
    }
    if (r->tail != r->head) {
        r->last_sample = r->storage[r->tail];
        r->tail = (r->tail + 1) & (AUDIO_RING_SIZE - 1);
        r->underrun_counter = 0;
    } else {
        r->underrun_counter++;
        if (r->underrun_counter > UNDERRUN_LIMIT) {
            r->is_streaming = false;
            r->last_sample = 128;
        }
        // sonst: Zero-Order-Hold, last_sample bleibt stehen (Micro-Jitter)
    }
    return r->last_sample;
}

// Fuellt genau EIN Segment (SAMPLES_PER_DESC Samples) mit frisch berechneten
// AM-Signalwerten. Laeuft im Fuell-Task auf Core 1.
//
// gegenueber der Zwei-Traeger-Version: nur noch EIN NCO, entsprechend
// vereinfachte Rechnung (kein Summieren zweier Traeger mehr).
static void fill_segment(int idx)
{
    uint16_t *out = dma_buf[idx];

    // Phase-Increment einmal pro Segment aus dem volatile-Feld lesen.
    // (Da die Frequenz jetzt nur noch im Startup-Code gesetzt wird, aendert
    // sie sich zur Laufzeit nicht mehr - das Auslesen pro Segment bleibt
    // trotzdem bestehen, falls spaeter doch eine Laufzeitsteuerung ergaenzt
    // wird, z.B. ueber Taster oder Bluetooth-AVRCP.)
    uint32_t inc = nco.phase_inc;
    uint32_t ph  = nco.phase;

    for (int n = 0; n < SAMPLES_PER_DESC; n++) {
        int32_t audio = (int32_t)pcm_pull(&pcm_ring) - 128; // -128..127

        // Spitzenwert (Betrag) fuer die Debug-Telemetrie mitfuehren - wird
        // bei JEDEM Sample aktualisiert (nicht nur beim letzten), damit auch
        // kurze laute Transienten innerhalb eines Segments erfasst werden.
        int32_t audio_abs = (audio < 0) ? -audio : audio;
        if (audio_abs > dbg_audio_peak) {
            dbg_audio_peak = audio_abs;
        }

        int32_t carrier = sine_lut[ph >> (32 - LUT_BITS)];
        ph += inc;

        int32_t mod = CARRIER_BASE_Q15 + audio * MOD_GAIN_Q15;

        // PERFORMANCE: mod (max ~32640) * carrier (max 32767) passt sicher
        // in int32 (Produkt max. ~1,07 Mrd., int32 reicht bis ~2,1 Mrd.).
        // int64_t hier zu verwenden zwingt den Compiler auf der Xtensa-LX6-
        // CPU (kein nativer 64x64-Bit-Multiplizierer!) zu einer teuren
        // Software-Multiplikationsroutine - mit int32_t nutzt er stattdessen
        // die schnelle native 32x32->32-Bit-MUL-Instruktion.
        int32_t mix = (mod * carrier) >> 15;

        int32_t dac_val = (mix * DAC_DIV_RECIP) >> DAC_DIV_SHIFT;
        if (dac_val > 127)  dac_val = 127;
        if (dac_val < -128) dac_val = -128;

        // Ausgabe als 8-Bit unsigned, Mitte=128 (wie ESP32Siggen.c),
        // im unteren Byte des 16-Bit-Samples (oberes Byte = Padding = 0).
        out[n] = (uint16_t)(uint8_t)(dac_val + 128);

        if (n == SAMPLES_PER_DESC - 1) {
            dbg_last_audio = audio;
            dbg_last_mix = mix;
        }
    }

    dbg_is_streaming = pcm_ring.is_streaming;
    dbg_ring_fill = (pcm_ring.head >= pcm_ring.tail) ? (pcm_ring.head - pcm_ring.tail)
                    : (AUDIO_RING_SIZE - (pcm_ring.tail - pcm_ring.head));

    // Phase fuer's naechste Segment sichern (Phasenkontinuitaet ueber
    // Segmentgrenzen hinweg - kein Neustart bei 0, sonst Phasensprung).
    nco.phase = ph;
}

// Fuell-Task: wartet auf Notify-Bits aus der I2S-ISR (welche Segmente
// gerade fertig abgespielt wurden) und rechnet sie sofort neu.
// UNVERAENDERT.
static void fill_task(void *arg)
{
    // Alle Segmente einmal initial fuellen, bevor der DMA-Ring startet.
    for (int i = 0; i < NUM_DESC; i++) fill_segment(i);

    for (;;) {
        uint32_t notified_mask = 0;
        xTaskNotifyWait(0, 0xFFFFFFFF, &notified_mask, portMAX_DELAY);
        for (int i = 0; i < NUM_DESC; i++) {
            if (notified_mask & (1u << i)) {
                fill_segment(i);
            }
        }
    }
}

// ---------------------------------------------------------------------
// Bluetooth-A2DP-Sink (ersetzt die komplette RAW-lwIP-UDP-Sektion).
//
// Ablauf:
//  1) bt_app_gap_cb()  - Pairing/Verbindungsereignisse (SSP-Autoconfirm,
//                         damit man nicht manuell einen PIN eingeben muss)
//  2) bt_app_a2d_cb()  - A2DP-Verbindungs-/Audiozustand + die tatsaechlich
//                         ausgehandelte Abtastrate (ESP_A2D_AUDIO_CFG_EVT)
//  3) bt_app_a2d_data_cb() - der eigentliche Audio-Hotpath: bekommt vom
//                         Bluedroid-Stack fertig dekodiertes SBC->PCM
//                         (16-Bit signed, interleaved Stereo) und schreibt
//                         daraus das L+R-Mono-Sample in den Ringpuffer -
//                         architektonisch die Nachfolgefunktion von
//                         audio_udp_recv_cb() aus der UDP-Version.
// ---------------------------------------------------------------------

// Wird bei GAP-Ereignissen (Pairing etc.) aufgerufen. Wir bestaetigen
// "Secure Simple Pairing"-Anfragen automatisch, damit sich Handys ohne
// manuelle PIN-Eingabe am ESP32 anmelden koennen (typisch fuer einfache
// BT-Lautsprecher/Audio-Receiver).
static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Pairing erfolgreich mit %s", param->auth_cmpl.device_name);
        } else {
            ESP_LOGW(TAG, "Pairing fehlgeschlagen, Status=%d", param->auth_cmpl.stat);
        }
        break;
    case ESP_BT_GAP_CFM_REQ_EVT:
        // Automatische Bestaetigung des Pairing-Codes (Secure Simple Pairing,
        // "Just Works"/Numeric-Comparison) - keine Nutzereingabe am ESP32 noetig.
        ESP_LOGI(TAG, "SSP-Bestaetigung, Passkey: %lu (automatisch bestaetigt)",
                 (unsigned long)param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_PIN_REQ_EVT: {
        // Fallback fuer sehr alte Geraete ohne SSP: festes 4-stelliges Pin "0000".
        ESP_LOGI(TAG, "Legacy-PIN angefragt, sende '0000'");
        esp_bt_pin_code_t pin_code = {'0', '0', '0', '0'};
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        break;
    }
    default:
        break;
    }
}

// A2DP-Verbindungs-/Audiozustand + Codec-Konfiguration.
static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
        ESP_LOGI(TAG, "A2DP Verbindungsstatus: %d", param->conn_stat.state);
        if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            // Neue Verbindung: Ring zuruecksetzen, damit keine Altlast-
            // Samples aus einer vorherigen Session moduliert werden.
            pcm_ring.head = 0;
            pcm_ring.tail = 0;
            pcm_ring.is_streaming = false;
            pcm_ring.underrun_counter = 0;
            pcm_ring.rate_acc = 0;
            pcm_ring.last_sample = 128;
        }
        break;

    case ESP_A2D_AUDIO_STATE_EVT:
        ESP_LOGI(TAG, "A2DP Audiostatus: %d", param->audio_stat.state);
        break;

    case ESP_A2D_AUDIO_CFG_EVT:
        // Hier meldet der Bluedroid-Stack, welche SBC-Parameter mit der
        // Quelle (Handy/PC) tatsaechlich ausgehandelt wurden - insbesondere
        // die Abtastrate. Diese wird NICHT vom ESP32 erzwungen (siehe
        // ausfuehrlicher Kommentar oben bei "audio_sample_rate_hz"),
        // sondern hier nur ausgelesen und fuer die Ratenwandlung in
        // pcm_pull() uebernommen.
        if (param->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
            // HINWEIS (Anpassung an die installierte ESP-IDF-Version):
            // In aktuellen ESP-IDF-Versionen liegt die SBC-Codec-Information
            // nicht mehr als rohes Byte-Array (cie.sbc[0]) vor, sondern als
            // benanntes Bitfeld-Struct esp_a2d_cie_sbc_t (samp_freq, ch_mode,
            // usw.) - siehe esp_a2dp_api.h. samp_freq ist dabei bereits die
            // 4-Bit-Bitmaske aus der A2DP-Spezifikation:
            //   Bit3=16 kHz, Bit2=32 kHz, Bit1=44.1 kHz, Bit0=48 kHz.
            uint8_t samp_freq = param->audio_cfg.mcc.cie.sbc_info.samp_freq;
            uint32_t negotiated_rate;
            if (samp_freq & 0x08)      negotiated_rate = 16000;
            else if (samp_freq & 0x04) negotiated_rate = 32000;
            else if (samp_freq & 0x02) negotiated_rate = 44100;
            else                       negotiated_rate = 48000;

            audio_sample_rate_hz = negotiated_rate;
            // Ratenwandlungs-Akkumulator zuruecksetzen, damit der Sprung
            // auf die neue Rate nicht zu einem kurzen Phasensprung im
            // Bresenham-Akkumulator fuehrt.
            pcm_ring.rate_acc = 0;

            ESP_LOGI(TAG, "A2DP-Audiokonfiguration: SBC, Abtastrate=%lu Hz "
                     "(Wunsch war 16 kHz - Quelle waehlt aus 16/32/44.1/48 kHz)",
                     (unsigned long)negotiated_rate);
        }
        break;

    default:
        break;
    }
}

// ---------------------------------------------------------------------
// Der eigentliche Audio-Hotpath: wandelt empfangenes Bluetooth-PCM in das
// Modulationssignal fuer den NCO um.
//
// Wird vom Bluedroid-Stack fuer jeden empfangenen, bereits SBC-dekodierten
// PCM-Block aufgerufen. Format: 16-Bit signed, little-endian, interleaved
// Stereo (L0,R0,L1,R1,...) - unabhaengig von der ausgehandelten Abtastrate.
//
// Vorgabe war "PCM-Modulationssample aus dem L+R-Signal bilden": hier wird
// pro Stereo-Sample-Paar der Mittelwert aus L und R gebildet ("Mono-Downmix")
// und anschliessend auf das bisherige 8-Bit-unsigned-Format (Mitte=128)
// herunterskaliert, damit der Rest der Modulatorkette (pcm_pull(),
// fill_segment()) unveraendert weiterverwendet werden kann.
// ---------------------------------------------------------------------
static void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len)
{
    dbg_bt_packets++;
    dbg_bt_bytes += len;

    const int16_t *pcm16 = (const int16_t *)data;
    uint32_t n_stereo_samples = len / (2 * sizeof(int16_t)); // 2 Kanaele * 16 Bit

    pcm_ring_t *r = &pcm_ring;

    for (uint32_t i = 0; i < n_stereo_samples; i++) {
        int32_t left  = pcm16[2 * i];
        int32_t right = pcm16[2 * i + 1];

        // Mono-Downmix: einfacher Mittelwert aus L und R (-32768..32767).
        int32_t mono16 = (left + right) / 2;

        // Herunterskalieren auf 8-Bit unsigned, Mitte=128 (arithmetisches
        // Rechtsschieben um 8 Bit nimmt das obere Byte = die 8 signifi-
        // kantesten Bits; +128 verschiebt von signed auf unsigned Mitte;
        // Ergebnis liegt garantiert im Bereich 0..255, kein Clamping noetig):
        uint8_t sample8 = (uint8_t)((mono16 >> 8) + 128);

        uint32_t next_head = (r->head + 1) & (AUDIO_RING_SIZE - 1);
        if (next_head != r->tail) {
            r->storage[r->head] = sample8;
            r->head = next_head;
        } else {
            break; // Ring voll - Rest dieses Pakets verwerfen (wie vorher bei UDP)
        }

        // Dasselbe Mono-Sample zusaetzlich (mit voller 16-Bit-
        // Praezision, unquantisiert) in den separaten DTMF-Ring schreiben -
        // siehe Kommentar bei dtmf_ring_t oben. Ein voller DTMF-Ring ist
        // unkritisch (die Erkennung braucht keine lueckenlose Historie),
        // daher hier bewusst kein "break", nur das aelteste Sample wird
        // implizit ueberschrieben/das neue verworfen.
        uint32_t dtmf_next_head = (dtmf_ring.head + 1) & (DTMF_SAMPLE_RING_SIZE - 1);
        if (dtmf_next_head != dtmf_ring.tail) {
            dtmf_ring.storage[dtmf_ring.head] = (int16_t)mono16;
            dtmf_ring.head = dtmf_next_head;
        }
    }
}

// ---------------------------------------------------------------------
// Goertzel-Algorithmus - Kernfunktion.
//
// Berechnet die spektrale Leistung EINER Zielfrequenz "freq" innerhalb
// eines Blocks von "n" Samples (Abtastrate "fs"). Deutlich guenstiger als
// eine FFT, wenn - wie hier bei DTMF - nur wenige, fest bekannte Frequenzen
// von Interesse sind, da anders als bei einer FFT nicht das gesamte
// Spektrum berechnet werden muss.
// ---------------------------------------------------------------------
static float goertzel_power(const int16_t *samples, int n, float freq, float fs)
{
    float omega = 2.0f * (float)M_PI * freq / fs;
    float coeff = 2.0f * cosf(omega);
    float q0, q1 = 0.0f, q2 = 0.0f;

    for (int i = 0; i < n; i++) {
        q0 = coeff * q1 - q2 + (float)samples[i];
        q2 = q1;
        q1 = q0;
    }
    // Leistung (nicht normiert) - fuer den Vergleich mehrerer Frequenzen
    // untereinander (Zeilen/Spalten-Maximum, Dominanz-Verhaeltnis) reicht
    // der unnormierte Wert aus, eine Division durch n ist nicht noetig.
    return q1 * q1 + q2 * q2 - q1 * q2 * coeff;
}

// DTMF-Frequenzmatrix (Telefon-Tastenfeld): 4 "Zeilen"- + 4 "Spalten"-Toene,
// jede Taste = genau eine Zeilen- + eine Spaltenfrequenz gleichzeitig.
static const float dtmf_row_freq[4] = {697.0f, 770.0f, 852.0f, 941.0f};
static const float dtmf_col_freq[4] = {1209.0f, 1336.0f, 1477.0f, 1633.0f};
static const char  dtmf_digit_map[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'},
};

// KALIBRIERUNG: Mindest-"Lautstaerke" (Goertzel-Leistung, unnormiert) und
// Mindest-Dominanzverhaeltnis (staerkste vs. zweitstaerkste Zeile/Spalte),
// ab der ein Ton als "eindeutiges DTMF" gilt statt als zufaellige
// spektrale Energie aus Musik/Sprache. Je hoeher DTMF_DOMINANCE_RATIO,
// desto strenger die Pruefung (weniger Fehlerkennungen, aber DTMF-Toene
// muessen "sauber" ankommen). Ggf. am realen Testtelefon nachjustieren.
#define DTMF_MIN_POWER        2.0e7f
#define DTMF_DOMINANCE_RATIO  2.5f

// Wertet einen Sample-Block aus und liefert die erkannte DTMF-Taste
// ('0'-'9','*','#','A'-'D') oder 0, wenn kein eindeutiger Ton erkannt wurde.
static char dtmf_process_block(const int16_t *block, int n, uint32_t fs)
{
    float row_p[4], col_p[4];
    for (int r = 0; r < 4; r++) row_p[r] = goertzel_power(block, n, dtmf_row_freq[r], (float)fs);
    for (int c = 0; c < 4; c++) col_p[c] = goertzel_power(block, n, dtmf_col_freq[c], (float)fs);

    int best_r = 0, second_r = 0;
    for (int r = 1; r < 4; r++) {
        if (row_p[r] > row_p[best_r]) { second_r = best_r; best_r = r; }
        else if (row_p[r] > row_p[second_r] || second_r == best_r) { second_r = r; }
    }
    int best_c = 0, second_c = 0;
    for (int c = 1; c < 4; c++) {
        if (col_p[c] > col_p[best_c]) { second_c = best_c; best_c = c; }
        else if (col_p[c] > col_p[second_c] || second_c == best_c) { second_c = c; }
    }

    bool row_ok = (row_p[best_r] >= DTMF_MIN_POWER) &&
                  (row_p[best_r] >= DTMF_DOMINANCE_RATIO * row_p[second_r]);
    bool col_ok = (col_p[best_c] >= DTMF_MIN_POWER) &&
                  (col_p[best_c] >= DTMF_DOMINANCE_RATIO * col_p[second_c]);

    if (row_ok && col_ok) {
        return dtmf_digit_map[best_r][best_c];
    }
    return 0; // kein eindeutiger DTMF-Ton in diesem Block
}

// ---------------------------------------------------------------------
// Sequenz-Zustandsautomat.
//
// Sucht in der erkannten Zeichenfolge nach dem Muster "*#nnnn#" (genau
// 4 Ziffern zwischen den beiden Sonderzeichen), z.B. "*#0847#" fuer
// 847 kHz. Die ungewoehnliche Einleitung ("*#") UND die feste Laenge von
// genau 4 Ziffern machen es sehr unwahrscheinlich, dass diese Sequenz
// zufaellig in normaler Musik/Sprache auftaucht (im Gegensatz zu einer
// einzelnen DTMF-Ziffer, die z.B. durch ein hochfrequentes Musikelement
// durchaus mal fehlerkannt werden koennte). Jede unerwartete Ziffer/Zeichen
// wirft die Erkennung sofort zurueck auf den Ausgangszustand.
// ---------------------------------------------------------------------
typedef enum {
    DTMF_WAIT_STAR = 0,   // wartet auf einleitendes '*'
    DTMF_WAIT_HASH1,      // wartet auf das '#' direkt nach dem '*'
    DTMF_COLLECT_DIGITS,  // sammelt genau 4 Ziffern
    DTMF_WAIT_HASH2,      // wartet auf das abschliessende '#'
} dtmf_seq_state_t;

static dtmf_seq_state_t dtmf_seq_state = DTMF_WAIT_STAR;
static char     dtmf_digits[5];      // 4 Ziffern + Nullterminierung
static int      dtmf_digit_idx = 0;
// Sequenz-Timeout: falls eine begonnene Sequenz nicht innerhalb dieser
// Zeit abgeschlossen wird (z.B. Verbindung unterbrochen, Nutzer bricht ab),
// wird sie verworfen, statt fuer immer "haengen" zu bleiben.
#define DTMF_SEQ_TIMEOUT_US   (5 * 1000 * 1000)  // 5 Sekunden
static int64_t  dtmf_seq_last_event_us = 0;

// Wird bei jeder neu bestaetigten (entprellten) DTMF-Taste aufgerufen.
static void dtmf_sequence_feed(char c)
{
    int64_t now = esp_timer_get_time();
    if (dtmf_seq_state != DTMF_WAIT_STAR &&
        (now - dtmf_seq_last_event_us) > DTMF_SEQ_TIMEOUT_US) {
        ESP_LOGI(TAG, "DTMF-Sequenz: Timeout, wird zurueckgesetzt");
        dtmf_seq_state = DTMF_WAIT_STAR;
        dtmf_digit_idx = 0;
    }
    dtmf_seq_last_event_us = now;

    switch (dtmf_seq_state) {
    case DTMF_WAIT_STAR:
        if (c == '*') {
            dtmf_seq_state = DTMF_WAIT_HASH1;
            ESP_LOGI(TAG, "DTMF-Sequenz: '*' erkannt, erwarte '#'");
        }
        break;

    case DTMF_WAIT_HASH1:
        if (c == '#') {
            dtmf_seq_state = DTMF_COLLECT_DIGITS;
            dtmf_digit_idx = 0;
            ESP_LOGI(TAG, "DTMF-Sequenz: Einleitung '*#' komplett, erwarte 4 Ziffern");
        } else if (c != '*') {
            dtmf_seq_state = DTMF_WAIT_STAR;   // unerwartetes Zeichen -> abbrechen
        }
        // ein wiederholtes '*' laesst den Zustand unveraendert (Toleranz
        // gegenueber mehrfachem Antippen der Taste)
        break;

    case DTMF_COLLECT_DIGITS:
        if (c >= '0' && c <= '9') {
            dtmf_digits[dtmf_digit_idx++] = c;
            if (dtmf_digit_idx >= 4) {
                dtmf_seq_state = DTMF_WAIT_HASH2;
            }
        } else {
            ESP_LOGW(TAG, "DTMF-Sequenz: unerwartetes Zeichen '%c' statt Ziffer - verworfen", c);
            dtmf_seq_state = DTMF_WAIT_STAR;
        }
        break;

    case DTMF_WAIT_HASH2:
        if (c == '#') {
            dtmf_digits[4] = '\0';
            uint32_t khz = (uint32_t)atoi(dtmf_digits);
            uint32_t new_freq_hz = khz * 1000UL;

            // Plausibilitaetspruefung: die Traegerfrequenz muss deutlich
            // unterhalb der halben DAC-Ausgaberate (Nyquist, FS_HZ/2)
            // liegen, sonst waere das NCO-Ergebnis nicht mehr sinnvoll.
            if (new_freq_hz > 0 && new_freq_hz < (FS_HZ / 2)) {
                nco.phase_inc = freq_to_phase_inc(new_freq_hz);
                ESP_LOGI(TAG, "DTMF-Sequenz '*#%s#' erkannt -> NCO-Frequenz auf %lu Hz gesetzt",
                         dtmf_digits, (unsigned long)new_freq_hz);
            } else {
                ESP_LOGW(TAG, "DTMF-Sequenz '*#%s#' erkannt, aber %lu Hz ausserhalb des "
                         "gueltigen Bereichs (0 < f < %lu Hz) - ignoriert",
                         dtmf_digits, (unsigned long)new_freq_hz, (unsigned long)(FS_HZ / 2));
            }
        } else {
            ESP_LOGW(TAG, "DTMF-Sequenz: abschliessendes '#' erwartet, '%c' erhalten - verworfen", c);
        }
        dtmf_seq_state = DTMF_WAIT_STAR;   // nach Versuch (Erfolg oder Fehlschlag) immer zuruecksetzen
        break;
    }
}

// ---------------------------------------------------------------------
// DTMF-Task - laeuft auf Core 0 mit niedriger Prioritaet, VOELLIG
// unabhaengig von der harten Echtzeit-DSP-Schleife auf Core 1.
//
// Liest fortlaufend Bloecke aus dtmf_ring, wertet sie per Goertzel aus
// und entprellt das Ergebnis (DTMF_CONFIRM_BLOCKS aufeinanderfolgende
// Bloecke mit demselben Zeichen, danach erst wieder neu auslesbar, sobald
// der Ton abreisst - verhindert Mehrfachauswertung bei gehaltener Taste).
// ---------------------------------------------------------------------
#define DTMF_BLOCK_MS          30    // Blocklaenge (Kompromiss Frequenzaufloesung/Reaktionszeit)
#define DTMF_MAX_BLOCK_SAMPLES 1536  // reicht bis 48 kHz * 30 ms = 1440 Samples
#define DTMF_CONFIRM_BLOCKS    2     // so viele gleiche Bloecke hintereinander = 1 bestaetigte Taste

// BUGFIX (Task-Watchdog-Absturz auf IDLE0): pdMS_TO_TICKS(5) rundet bei der
// in ESP-IDF haeufig verwendeten Tick-Rate von 100 Hz (10-ms-Tick) durch
// Ganzzahldivision auf 0 Ticks ab (5*100/1000=0). vTaskDelay(0) blockiert
// dann faktisch NICHT - der Task blieb in einer Busy-Loop haengen und hat
// Core 0 komplett belegt, sodass der IDLE0-Task nie mehr drankam und der
// Task-Watchdog ausgeloest hat (siehe Log: "IDLE0 (CPU 0)" / "CPU 0:
// dtmf_task"). Fix: laengeres Poll-Intervall (10 ms) UND zusaetzliche
// Absicherung, die "0 Ticks" unabhaengig von der konfigurierten Tick-Rate
// grundsaetzlich ausschliesst.
#define DTMF_POLL_DELAY_MS     10
static inline TickType_t dtmf_safe_delay_ticks(void)
{
    TickType_t t = pdMS_TO_TICKS(DTMF_POLL_DELAY_MS);
    return (t > 0) ? t : 1;   // niemals 0 Ticks - sonst faktisch keine Blockierung
}

static void dtmf_task(void *arg)
{
    static int16_t block[DTMF_MAX_BLOCK_SAMPLES];
    char stable_candidate = 0;
    int  stable_count = 0;
    char last_reported = 0;

    for (;;) {
        uint32_t fs = audio_sample_rate_hz;   // Momentaufnahme (aendert sich selten)
        uint32_t block_len = (fs * DTMF_BLOCK_MS) / 1000;
        if (block_len < 64) block_len = 64;
        if (block_len > DTMF_MAX_BLOCK_SAMPLES) block_len = DTMF_MAX_BLOCK_SAMPLES;

        uint32_t fill = (dtmf_ring.head >= dtmf_ring.tail)
                         ? (dtmf_ring.head - dtmf_ring.tail)
                         : (DTMF_SAMPLE_RING_SIZE - (dtmf_ring.tail - dtmf_ring.head));
        if (fill < block_len) {
            vTaskDelay(dtmf_safe_delay_ticks());   // noch nicht genug Samples fuer einen Block
            continue;
        }

        for (uint32_t i = 0; i < block_len; i++) {
            block[i] = dtmf_ring.storage[dtmf_ring.tail];
            dtmf_ring.tail = (dtmf_ring.tail + 1) & (DTMF_SAMPLE_RING_SIZE - 1);
        }

        char detected = dtmf_process_block(block, (int)block_len, fs);

        if (detected == 0) {
            // Stille/kein eindeutiger Ton: Entprellung zuruecksetzen, damit
            // dieselbe Taste beim naechsten Druck erneut erkannt werden kann.
            stable_candidate = 0;
            stable_count = 0;
            last_reported = 0;
        } else {
            if (detected == stable_candidate) {
                stable_count++;
            } else {
                stable_candidate = detected;
                stable_count = 1;
            }
            if (stable_count >= DTMF_CONFIRM_BLOCKS && last_reported != detected) {
                last_reported = detected;
                ESP_LOGI(TAG, "DTMF-Taste erkannt: '%c'", detected);
                dtmf_sequence_feed(detected);
            }
        }
    }
}

// ---------------------------------------------------------------------
// Minimaler AVRCP-Controller (AVRC-CT).
//
// Hintergrund: Reines A2DP ohne AVRCP wurde von Bluedroid selbst schon
// bemaengelt ("A2DP Enable without AVRC") und fuehrt in der Praxis dazu,
// dass viele Handys nach dem Pairing zwar die A2DP-Verbindung aufbauen
// (siehe Log: "A2DP Verbindungsstatus: 2" = CONNECTED), aber KEIN Audio
// streamen, weil sie erst per AVRCP pruefen wollen, ob die Gegenstelle
// (der ESP32) das Wiedergabe-/Fernsteuerungsprofil unterstuetzt bzw.
// weil sie auf ein "Play"-Kommando vom Sink warten (typisch fuer
// Autoradios/Bluetooth-Lautsprecher-Emulation).
//
// Die Loesung: Der ESP32 initialisiert einen AVRCP-CONTROLLER (nicht
// -Target - wir wollen ja steuern, nicht gesteuert werden) und schickt
// direkt nach erfolgreichem AVRCP-Verbindungsaufbau ein virtuelles
// "PLAY"-Tastendruck-Kommando (PASSTHROUGH PLAY, press+release) an das
// Handy - so, als haette man am (nicht vorhandenen) Bluetooth-Lautsprecher
// die Play-Taste gedrueckt. Das reicht bei den allermeisten Handys, um
// das Streaming automatisch zu starten.
// ---------------------------------------------------------------------
static void bt_app_avrc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
        if (param->conn_stat.connected) {
            ESP_LOGI(TAG, "AVRCP verbunden - sende automatisches PLAY-Kommando");
            // "tl" (transaction label) 0 reicht hier, da wir nur einzelne,
            // nicht ueberlappende Kommandos senden.
            esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_PLAY, ESP_AVRC_PT_CMD_STATE_PRESSED);
            esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_PLAY, ESP_AVRC_PT_CMD_STATE_RELEASED);
        } else {
            ESP_LOGI(TAG, "AVRCP getrennt");
        }
        break;
    default:
        break;
    }
}

// Initialisiert Bluetooth-Controller, Bluedroid-Stack und den A2DP-Sink.
static void bt_init_a2dp_sink(void)
{
    // Nur klassisches Bluetooth (BR/EDR) wird fuer A2DP benoetigt,
    // kein Bluetooth Low Energy (BLE). Den BLE-Speicheranteil vorab
    // freizugeben spart RAM - unkritisch, aber sinnvoll, da der Fuell-
    // Task auf Core 1 sowieso jedes Byte Zeitbudget braucht.
    esp_err_t err = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "esp_bt_controller_mem_release(BLE) fehlgeschlagen: %d", err);
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_bt_gap_register_callback(bt_app_gap_cb));

    // Secure Simple Pairing: "Just Works"-IO-Fähigkeit (kein Display/
    // Tastatur am ESP32 vorhanden) - Handys verbinden sich dann i.d.R.
    // ohne Nutzerinteraktion auf dem ESP32.
    // HINWEIS (Anpassung an die installierte ESP-IDF-Version): die Funktion
    // esp_bt_gap_set_pin_type() existiert in dieser IDF-Version nicht mehr
    // (Fehler "implicit declaration"). Der korrekte Aufruf zum Setzen des
    // Pairing-PIN-Modus heisst esp_bt_gap_set_pin() - mit PIN-Typ "variabel"
    // und leerem PIN-Code (wird bei Secure Simple Pairing ohnehin ignoriert,
    // greift nur als Legacy-Fallback ueber ESP_BT_GAP_PIN_REQ_EVT oben).
    esp_bt_io_cap_t io_cap = ESP_BT_IO_CAP_NONE;
    ESP_ERROR_CHECK(esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_VARIABLE, 0, NULL));
    ESP_ERROR_CHECK(esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &io_cap, sizeof(io_cap)));

    ESP_ERROR_CHECK(esp_bt_gap_set_device_name(BT_DEVICE_NAME));
    ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE));

    ESP_ERROR_CHECK(esp_a2d_register_callback(bt_app_a2d_cb));
    ESP_ERROR_CHECK(esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb));
    ESP_ERROR_CHECK(esp_a2d_sink_init());

    // AVRCP-Controller initialisieren und Callback registrieren
    // (siehe ausfuehrlicher Kommentar bei bt_app_avrc_ct_cb() oben -
    // behebt "A2DP Enable without AVRC" und startet das Streaming beim
    // Handy automatisch per virtuellem PLAY-Kommando).
    ESP_ERROR_CHECK(esp_avrc_ct_init());
    ESP_ERROR_CHECK(esp_avrc_ct_register_callback(bt_app_avrc_ct_cb));

    ESP_LOGI(TAG, "Bluetooth-A2DP-Sink aktiv, sichtbar als '%s' - bitte am "
             "Handy/PC koppeln und Audio abspielen.", BT_DEVICE_NAME);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_sine_lut();

    memset(&pcm_ring, 0, sizeof(pcm_ring));
    pcm_ring.last_sample = 128;

    // ---------------------------------------------------------------
    // Startfrequenz wird nur noch HIER im Startup-Code gesetzt (kein
    // Port 5000 mehr, kein zweiter Traeger mehr). Einfach die gewuenschte
    // Zielfrequenz eintragen.
    // ---------------------------------------------------------------
    #define START_FREQ_HZ   999000UL   // 999 kHz

    nco.phase_inc = freq_to_phase_inc(START_FREQ_HZ);
    nco.phase = 0;

    for (int i = 0; i < NUM_DESC; i++) {
        dma_buf[i] = heap_caps_malloc(SAMPLES_PER_DESC * sizeof(uint16_t),
                                       MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        if (!dma_buf[i]) {
            ESP_LOGE(TAG, "DMA-Puffer-Allokation fuer Segment %d fehlgeschlagen", i);
            abort();
        }
    }

    // Fuell-Task auf Core 1, hohe Prioritaet (haert-Realtime-nah, muss
    // jedes Segment (250us) rechtzeitig nachfuellen koennen).
    xTaskCreatePinnedToCore(fill_task, "nco_fill_task", 4096, NULL,
                             configMAX_PRIORITIES - 2, &fill_task_handle, 1);

    // Core 1 ist bewusst voll durch die Echtzeit-DSP-Schleife (fill_task)
    // ausgelastet (4 Mio. Samples/s, kein Slack). Der IDLE1-Task kommt
    // dadurch nicht mehr rechtzeitig zum Zug, um den Task-Watchdog zu
    // fuettern -> IDLE1 explizit aus der WDT-Ueberwachung herausnehmen.
    // (Bevorzugt stattdessen per sdkconfig: CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1=n)
    TaskHandle_t idle1 = xTaskGetIdleTaskHandleForCore(1);
    if (idle1) {
        esp_err_t wdt_err = esp_task_wdt_delete(idle1);
        if (wdt_err != ESP_OK && wdt_err != ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "esp_task_wdt_delete(IDLE1) fehlgeschlagen: %d", wdt_err);
        }
    }

    init_i2s_parallel();   // startet den Hardware-DMA-Ring (nutzt fill_task_handle)

    // statt wifi_init_sta() + init_raw_udp_listeners() -> Bluetooth-A2DP-Sink
    bt_init_a2dp_sink();

    // DTMF-Erkennungstask - bewusst auf Core 0 (nicht Core 1, dort
    // laeuft die harte Echtzeit-DSP-Schleife fill_task() ohne jeden Slack)
    // und mit niedriger Prioritaet, da die Goertzel-Auswertung zeitlich
    // unkritisch ist (Bloecke von ~30 ms, siehe DTMF_BLOCK_MS).
    xTaskCreatePinnedToCore(dtmf_task, "dtmf_task", 4096, NULL,
                             tskIDLE_PRIORITY + 1, NULL, 0);

    ESP_LOGI(TAG, "AM-Modulator laeuft: 1x INT32-LUT-NCO @ %lu Hz Ausgaberate, "
             "Traegerfrequenz=%lu Hz, Modulationsquelle = Bluetooth-A2DP.",
             (unsigned long)FS_HZ, (unsigned long)START_FREQ_HZ);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Peak-Wert vor der Ausgabe atomar "abholen" und sofort fuer
        // das naechste 1-Sekunden-Fenster zuruecksetzen. Ein kurzer Read-
        // Reset-Race mit fill_segment() (Core 1) ist hier unkritisch - im
        // schlimmsten Fall geht ein einzelnes Sample zwischen Reset und
        // naechstem Vergleich "verloren", was fuer eine reine Debug-Anzeige
        // vernachlaessigbar ist.
        int32_t peak = dbg_audio_peak;
        dbg_audio_peak = 0;

        ESP_LOGI(TAG,
                 "DBG streaming=%d fill=%4lu audio=%4ld peak=%4ld mix=%ld | BT: rate=%lu Hz pkts=%lu bytes=%llu",
                 dbg_is_streaming, (unsigned long)dbg_ring_fill, (long)dbg_last_audio,
                 (long)peak, (long)dbg_last_mix, (unsigned long)audio_sample_rate_hz,
                 (unsigned long)dbg_bt_packets, (unsigned long long)dbg_bt_bytes);
    }
}
