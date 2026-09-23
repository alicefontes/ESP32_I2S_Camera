#include "OV7670.h"

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClient.h>
#include "BMP.h"

#include "driver/pcnt.h"
#include "esp32-hal-rmt.h"

const int SIOD = 21;
const int SIOC = 22;

const int VSYNC = 34;
const int HREF = 35;

const int XCLK = 32;
const int PCLK = 33;

const int D0 = 27;
const int D1 = 18;
const int D2 = 19;
const int D3 = 15;
const int D4 = 14;
const int D5 = 13;
const int D6 = 25;
const int D7 = 4;

const int SLAVE_READY = 26;

//const int TRIGGER = 26;
//const int TRIGGER_IN = 23;

// Sincronismo local (Câmera 1 - Mestre)
const int VSYNC_LOCAL = VSYNC; // Pino 34

// Sincronismo remoto (Câmera 2 - Escravo) 
// Usando o GPIO 23 liberado do antigo trigger físico!
// const int VSYNC_REMOTO = 23;

const int TFT_DC = 2;
const int TFT_CS = 5;

rmt_obj_t *rmtVsyncIn = nullptr;

#define VSYNC_EVENT_IN 23

rmt_data_t rmtBuffer[8];
size_t rmtBufferSize = 8;

rmt_obj_t *rmtVsyncLocal = nullptr;

#define VSYNC_LOCAL_RMT VSYNC

rmt_data_t rmtLocalBuffer[8];
size_t rmtLocalBufferSize = 8;


uint32_t timestampVsyncMestre = 0;
uint32_t timestampVsyncEscrava = 0;

bool vsyncMestreRecebido = false;
bool vsyncEscravaRecebido = false;

// ============================================================
// PAREAMENTO TEMPORAL DOS VSYNCs
// ============================================================

#define VSYNC_PERIODO_ESTIMADO_US 80000UL
#define VSYNC_MIN_INTERVALO_US    20000UL

uint32_t ultimoVsyncLocal = 0;
uint32_t penultimoVsyncLocal = 0;

bool existeUltimoVsyncLocal = false;
bool existePenultimoVsyncLocal = false;

uint32_t contadorFrameLocal = 0;
uint32_t contadorFrameRemoto = 0;

#define ssid1        "Wonderland"
#define password1    "Fontes1995!"


Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, 0);
OV7670 *camera;

WiFiMulti wifiMulti;
WiFiServer server(80);

unsigned char bmpHeader[BMP::headerSize];

volatile bool captureRequested = false;
bool xclkAtivo = false;

volatile bool eventoEscravaRecebido = false;
volatile uint32_t timestampEventoEscrava = 0;

void monitorarReadyEscrava()
{
  bool ready = digitalRead(SLAVE_READY);

  if (ready && !xclkAtivo)
  {
    Serial.println("[CLOCK MASTER] READY voltou HIGH.");
    Serial.println("[CLOCK MASTER] Religando XCLK...");

    ClockEnable(XCLK, 10000000);

    xclkAtivo = true;

    Serial.println("[CLOCK MASTER] XCLK ON.");
  }

  if (!ready && xclkAtivo)
  {
    Serial.println("[CLOCK MASTER] READY caiu para LOW.");
    Serial.println("[CLOCK MASTER] Desligando XCLK...");

    ClockDisable();

    xclkAtivo = false;

    Serial.println("[CLOCK MASTER] XCLK OFF.");
  }
}


/*
 * =========================================================
 * MEDICAO PCLK
 * =========================================================
 *
 * O PCNT do ESP32 conta as bordas de subida do PCLK
 * diretamente em hardware.
 *
 * Isso permite observar a frequencia do PCLK sem
 * executar uma interrupcao a cada borda.
 *
 * NAO altera:
 * - configuracao da camera
 * - I2S
 * - DMA
 * =========================================================
 */

void medirPCLK()
{
  Serial.println();
  Serial.println("========== MEDICAO PCLK ==========");
  Serial.println("[PCLK] Contando bordas de subida durante 1 ms...");

  pcnt_config_t pcntConfig = {};

  pcntConfig.pulse_gpio_num = PCLK;
  pcntConfig.ctrl_gpio_num = PCNT_PIN_NOT_USED;

  pcntConfig.channel = PCNT_CHANNEL_0;
  pcntConfig.unit = PCNT_UNIT_0;

  pcntConfig.pos_mode = PCNT_COUNT_INC;
  pcntConfig.neg_mode = PCNT_COUNT_DIS;

  pcntConfig.lctrl_mode = PCNT_MODE_KEEP;
  pcntConfig.hctrl_mode = PCNT_MODE_KEEP;

  pcntConfig.counter_h_lim = 32767;
  pcntConfig.counter_l_lim = 0;

  pcnt_unit_config(&pcntConfig);

  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);

  pcnt_counter_resume(PCNT_UNIT_0);

  unsigned long inicio = micros();

  /*
   * Janela de medicao de aproximadamente 1 ms.
   *
   * Durante esse intervalo o PCNT conta as bordas
   * diretamente em hardware.
   */
  while ((micros() - inicio) < 1000)
  {
    // contador trabalhando em hardware
  }

  unsigned long fim = micros();

  pcnt_counter_pause(PCNT_UNIT_0);

  int16_t contador = 0;
  pcnt_get_counter_value(PCNT_UNIT_0, &contador);

  unsigned long duracao = fim - inicio;

  float frequencia =
      ((float)contador * 1000000.0f) /
      (float)duracao;

  Serial.printf(
    "[PCLK] Bordas contadas = %d\n",
    contador
  );

  Serial.printf(
    "[PCLK] Janela de medicao = %lu us\n",
    duracao
  );

  Serial.printf(
    "[PCLK] Frequencia aproximada = %.3f MHz\n",
    frequencia / 1000000.0f
  );

  Serial.println("[PCLK] Medicao concluida.");
  Serial.println("========== FIM PCLK ==========");
  Serial.println();

  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);
}

// void medirVSYNC()
// {
//   Serial.println();
//   Serial.println("========== MEDICAO VSYNC ==========");
//   Serial.println("[VSYNC] Medindo HIGH/LOW por 10 ciclos...");

//   unsigned long t0, t1;
//   unsigned long highTime, lowTime;
//   unsigned long period;

//   // Garante que começamos em LOW
//   while (digitalRead(VSYNC) == HIGH);

//   for (int i = 0; i < 10; i++)
//   {
//     // Espera subida
//     while (digitalRead(VSYNC) == LOW);
//     t0 = micros();

//     // Espera descida
//     while (digitalRead(VSYNC) == HIGH);
//     t1 = micros();

//     highTime = t1 - t0;

//     // Espera próxima subida
//     t0 = micros();
//     while (digitalRead(VSYNC) == LOW);
//     t1 = micros();

//     lowTime = t1 - t0;

//     period = highTime + lowTime;

//     Serial.print("[VSYNC] Ciclo ");
//     Serial.print(i + 1);
//     Serial.print(" | HIGH = ");
//     Serial.print(highTime);
//     Serial.print(" us | LOW = ");
//     Serial.print(lowTime);
//     Serial.print(" us | PERIODO = ");
//     Serial.print(period);
//     Serial.println(" us");
//   }

//   Serial.println("[VSYNC] Medicao concluida.");
//   Serial.println("========== FIM VSYNC ==========");
// }

void medirVSYNC() {
  Serial.println("\n========== MEDICAO VSYNC FILTRADA ==========");
  pinMode(VSYNC, INPUT);

  for (int i = 0; i < 10; i++) {
    // 1. Espera subida REAL (High > 50us para evitar ruído)
    unsigned long tStartHigh = 0;
    while (true) {
      while (digitalRead(VSYNC) == LOW);
      tStartHigh = micros();
      delayMicroseconds(10); 
      if (digitalRead(VSYNC) == HIGH) break; // Confirma que é HIGH real
    }

    // 2. Espera descida REAL
    unsigned long tStartLow = 0;
    while (true) {
      while (digitalRead(VSYNC) == HIGH);
      tStartLow = micros();
      delayMicroseconds(10);
      if (digitalRead(VSYNC) == LOW) break; // Confirma que é LOW real
    }

    // 3. Espera próxima subida
    unsigned long tEnd = 0;
    while (true) {
      while (digitalRead(VSYNC) == LOW);
      tEnd = micros();
      delayMicroseconds(10);
      if (digitalRead(VSYNC) == HIGH) break;
    }

    unsigned long highTime = tStartLow - tStartHigh;
    unsigned long lowTime = tEnd - tStartLow;
    unsigned long periodo = highTime + lowTime;

    Serial.printf("[VSYNC] Ciclo %d | HIGH = %lu us | LOW = %lu us | PERIODO = %lu us\n", 
                  i + 1, highTime, lowTime, periodo);
  }
  Serial.println("========== FIM VSYNC ==========");
}

// void medirVSYNCRemoto()
// {
//   Serial.println();
//   Serial.println("========== MEDICAO VSYNC REMOTO ==========");
//   Serial.println("[VSYNC #2] Medindo periodo por 10 ciclos...");

//   pinMode(VSYNC_REMOTO, INPUT);

//   unsigned long t0, t1;
//   unsigned long periodo;

//   // Garante que começamos em LOW
//   while (digitalRead(VSYNC_REMOTO) == HIGH);

//   for (int i = 0; i < 10; i++)
//   {
//     // Espera subida
//     while (digitalRead(VSYNC_REMOTO) == LOW);
//     t0 = micros();

//     // Espera próxima subida
//     while (digitalRead(VSYNC_REMOTO) == HIGH);
//     while (digitalRead(VSYNC_REMOTO) == LOW);

//     t1 = micros();

//     periodo = t1 - t0;

//     Serial.print("[VSYNC #2] Ciclo ");
//     Serial.print(i + 1);
//     Serial.print(" | PERIODO = ");
//     Serial.print(periodo);
//     Serial.println(" us");
//   }

//   Serial.println("[VSYNC #2] Medicao concluida.");
//   Serial.println("========== FIM VSYNC #2 ==========");
// }


// void medirDefasagemVSYNC()
// {
//   Serial.println();
//   Serial.println("========== DEFASAGEM VSYNC ==========");
//   Serial.println("[FASE] Medindo 20 ciclos...");

//   pinMode(VSYNC_LOCAL, INPUT);
//   pinMode(VSYNC_REMOTO, INPUT);

//   bool estadoLocalAnterior = digitalRead(VSYNC_LOCAL);
//   bool estadoRemotoAnterior = digitalRead(VSYNC_REMOTO);

//   unsigned long tLocal[20];
//   unsigned long tRemoto[20];

//   int nLocal = 0;
//   int nRemoto = 0;

//   while (nLocal < 20 || nRemoto < 20)
//   {
//     bool estadoLocal = digitalRead(VSYNC_LOCAL);
//     bool estadoRemoto = digitalRead(VSYNC_REMOTO);

//     // Detecta subida do VSYNC da câmera 1
//     if (estadoLocal == HIGH && estadoLocalAnterior == LOW)
//     {
//       if (nLocal < 20)
//       {
//         tLocal[nLocal] = micros();
//         nLocal++;
//       }
//     }

//     // Detecta subida do VSYNC da câmera 2
//     if (estadoRemoto == HIGH && estadoRemotoAnterior == LOW)
//     {
//       if (nRemoto < 20)
//       {
//         tRemoto[nRemoto] = micros();
//         nRemoto++;
//       }
//     }

//     estadoLocalAnterior = estadoLocal;
//     estadoRemotoAnterior = estadoRemoto;
//   }

//   Serial.println();
//   Serial.println("[FASE] Timestamps capturados.");
//   Serial.print("[FASE] Camera 1: ");
//   Serial.println(nLocal);

//   Serial.print("[FASE] Camera 2: ");
//   Serial.println(nRemoto);

//   Serial.println();
//   Serial.println("[FASE] Diferenca entre VSYNCs:");

//   for (int i = 0; i < 20; i++)
//   {
//     long delta = (long)tRemoto[i] - (long)tLocal[i];

//     // Corrige caso os dois sinais tenham sido
//     // associados a ciclos consecutivos.

//     while (delta > 39900)
//       delta -= 79800;

//     while (delta < -39900)
//       delta += 79800;


//     Serial.print("[FASE] Ciclo ");
//     Serial.print(i + 1);
//     Serial.print(" | VSYNC2 - VSYNC1 = ");
//     Serial.print(delta);
//     Serial.println(" us");
//   }

//   Serial.println("[FASE] Medicao concluida.");
//   Serial.println("========== FIM DEFASAGEM ==========");
// }

// void medirVSYNCsJuntas()
// {
//   Serial.println();
//   Serial.println("========== MEDICAO DOS DOIS VSYNCs ==========");

//   pinMode(VSYNC_LOCAL, INPUT);
//   pinMode(VSYNC_REMOTO, INPUT);

//   int estadoLocalAnterior  = digitalRead(VSYNC_LOCAL);
//   int estadoRemotoAnterior = digitalRead(VSYNC_REMOTO);

//   unsigned long inicio = micros();

//   unsigned long ultimoLocal  = inicio;
//   unsigned long ultimoRemoto = inicio;

//   int ciclosLocal = 0;
//   int ciclosRemoto = 0;

//   while ((micros() - inicio) < 1000000UL)
//   {
//     unsigned long agora = micros();

//     int local  = digitalRead(VSYNC_LOCAL);
//     int remoto = digitalRead(VSYNC_REMOTO);

//     // Detecta subida do VSYNC da câmera #1
//     if (local == HIGH && estadoLocalAnterior == LOW)
//     {
//       if (ciclosLocal > 0)
//       {
//         Serial.print("[CAM 1] Periodo = ");
//         Serial.print(agora - ultimoLocal);
//         Serial.println(" us");
//       }

//       ultimoLocal = agora;
//       ciclosLocal++;
//     }

//     // Detecta subida do VSYNC da câmera #2
//     if (remoto == HIGH && estadoRemotoAnterior == LOW)
//     {
//       if (ciclosRemoto > 0)
//       {
//         Serial.print("[CAM 2] Periodo = ");
//         Serial.print(agora - ultimoRemoto);
//         Serial.println(" us");
//       }

//       ultimoRemoto = agora;
//       ciclosRemoto++;
//     }

//     estadoLocalAnterior  = local;
//     estadoRemotoAnterior = remoto;
//   }

//   Serial.println();
//   Serial.print("[CAM 1] Ciclos detectados: ");
//   Serial.println(ciclosLocal);

//   Serial.print("[CAM 2] Ciclos detectados: ");
//   Serial.println(ciclosRemoto);

//   Serial.println("========== FIM MEDICAO ==========");
// }

/*
 * =========================================================
 * SERVIDOR WEB
 * =========================================================
 */

void serve()
{
  WiFiClient client = server.available();

  if (client)
  {
    String currentLine = "";

    while (client.connected())
    {
      if (client.available())
      {
        char c = client.read();

        if (c == '\n')
        {
          if (currentLine.length() == 0)
          {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            client.print(
              "<style>"
              "body{margin:0}"
              "button{font-size:24px;padding:12px 24px;margin:10px}"
              "img{height:100%;width:auto}"
              "</style>"

              "<button onclick='fetch(\"/trigger\")'>CAPTURAR</button>"

              "<img id='a' src='/camera' "
              "onload='this.style.display=\"initial\"; "
              "var b=document.getElementById(\"b\"); "
              "b.style.display=\"none\"; "
              "b.src=\"camera?\"+Date.now();'>"

              "<img id='b' style='display:none' src='/camera' "
              "onload='this.style.display=\"initial\"; "
              "var a=document.getElementById(\"a\"); "
              "a.style.display=\"none\"; "
              "a.src=\"camera?\"+Date.now();'>"
            );

            client.println();
            break;
          }
          else
          {
            currentLine = "";
          }
        }
        else if (c != '\r')
        {
          currentLine += c;
        }

        // TRIGGER
        if (currentLine.endsWith("GET /trigger"))
        {
          triggerCapture();

          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/plain");
          client.println();
          client.println("TRIGGER OK");
        }

        // CAMERA
        if (currentLine.endsWith("GET /camera"))
        {
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:image/bmp");
          client.println();

          client.write(bmpHeader, BMP::headerSize);

          client.write(
            camera->frame,
            camera->xres * camera->yres * 2
          );
        }
      }
    }

    client.stop();
  }
}


/*
 * =========================================================
 * TRIGGER
 * =========================================================
 */

void triggerCapture()
{
  Serial.println("[TRIGGER] Disparo!");
  captureRequested = true;
}

// void triggerCapture()
// {
//   Serial.println("[TRIGGER] Disparo!");

//   digitalWrite(TRIGGER, HIGH);

//   Serial.println("[TRIGGER] Pulso iniciado.");

//   unsigned long inicio = millis();
//   bool triggerDetectado = false;

//   while (millis() - inicio < 20)
//   {
//     if (digitalRead(TRIGGER_IN) == HIGH)
//     {
//       triggerDetectado = true;
//       break;
//     }
//   }

//   digitalWrite(TRIGGER, LOW);

//   Serial.println("[TRIGGER] Pulso finalizado.");

//   if (triggerDetectado)
//   {
//     Serial.println("[TRIGGER] Sinal detectado no GPIO23!");

//     captureRequested = true;
//   }
//   else
//   {
//     Serial.println(
//       "[TRIGGER] ERRO: sinal nao detectado no GPIO23!"
//     );
//   }
// }

// void medirDefasagemFase() {
//   pinMode(VSYNC_LOCAL, INPUT);
//   pinMode(VSYNC_REMOTO, INPUT);

//   // 1. Aguarda a borda de subida do VSYNC local
//   while (digitalRead(VSYNC_LOCAL) == LOW);
//   unsigned long t1 = micros();

//   // 2. Aguarda a borda de subida do VSYNC remoto
//   while (digitalRead(VSYNC_REMOTO) == LOW);
//   unsigned long t2 = micros();

//   long defasagem = t2 - t1;

//   Serial.println("=================================");
//   Serial.print("[DEFASAGEM TEMPORAL] dt = ");
//   Serial.print(defasagem);
//   Serial.println(" us");
//   Serial.println("=================================");
// }

// void medirDefasagemFase()
// {
//   pinMode(VSYNC_LOCAL, INPUT);
//   pinMode(VSYNC_REMOTO, INPUT);

//   Serial.println();
//   Serial.println("========== MEDICAO DE DEFASAGEM ==========");
//   Serial.println("[FASE] Medindo 20 ciclos...");

//   // Garantir que começamos com os dois sinais em LOW
//   while (digitalRead(VSYNC_LOCAL) == HIGH);
//   while (digitalRead(VSYNC_REMOTO) == HIGH);

//   for (int i = 0; i < 20; i++)
//   {
//     // Espera a subida do VSYNC local
//     while (digitalRead(VSYNC_LOCAL) == LOW);
//     unsigned long t1 = micros();

//     // Espera a subida do VSYNC remoto
//     while (digitalRead(VSYNC_REMOTO) == LOW);
//     unsigned long t2 = micros();

//     long defasagem = (long)(t2 - t1);

//     Serial.print("[FASE] Ciclo ");
//     Serial.print(i + 1);
//     Serial.print(" | VSYNC local → remoto = ");
//     Serial.print(defasagem);
//     Serial.println(" us");

//     // Espera os dois sinais voltarem para LOW
//     while (digitalRead(VSYNC_LOCAL) == HIGH);
//     while (digitalRead(VSYNC_REMOTO) == HIGH);
//   }

//   Serial.println("[FASE] Medicao concluida.");
//   Serial.println("========== FIM DEFASAGEM ==========");
//   Serial.println();
// }

/*
 * =========================================================
 * SETUP
 * =========================================================
 */

// void setup()
// {
//   Serial.begin(115200);

//   // =====================================================
//   // TRIGGER
//   // =====================================================

//   //pinMode(TRIGGER, OUTPUT);
//   //digitalWrite(TRIGGER, LOW);

//   //pinMode(TRIGGER_IN, INPUT);

//   //Serial.println("[TRIGGER] GPIO26 = OUTPUT");
//   //Serial.println("[TRIGGER] GPIO23 = INPUT");

//   // =====================================================
//   // SETUP ORIGINAL
//   // =====================================================
//   // pinMode(SLAVE_READY, INPUT);
//   pinMode(SLAVE_READY, INPUT_PULLDOWN);
//   pinMode(VSYNC_REMOTO, INPUT); // GPIO 23 precisa ser entrada de alta impedância!

//   Serial.println("[SETUP] Inicio");

//   wifiMulti.addAP(ssid1, password1);

//   Serial.println("[WIFI] Conectando...");

//   if (wifiMulti.run() == WL_CONNECTED)
//   {
//     Serial.println("");
//     Serial.println("[WIFI] WiFi connected");
//     Serial.println("[WIFI] IP address: ");
//     Serial.println(WiFi.localIP());
//   }

//   // =====================================================
//   // PAUSA DE SINCRONISMO DE BOOT (MESTRE / ESCRAVO)
//   // =====================================================
//   // Aguarda 5 segundos para garantir que o ESP32 #2 (Escravo) 
//   // já tenha ligado e terminado o boot ANTES do XCLK ser ativado.
//   //Serial.println("[CLOCK MASTER] Aguardando 5s para o boot do ESP32 #2 estabilizar...");
//   //delay(5000); 

//   // Serial.println("[CAMERA #1] Antes de criar OV7670 (Ativando XCLK Mestre)");

//   // =====================================================
//   // HANDSHAKE COM A ESCRAVA
//   // =====================================================

//   Serial.println("[CLOCK MASTER] Aguardando READY da Escrava...");

//   while (digitalRead(SLAVE_READY) == LOW)
//   {
//     delay(10);
//   }

//   Serial.println("[CLOCK MASTER] READY recebido!");
//   Serial.println("[CLOCK MASTER] Escrava inicializada.");
//   Serial.println("[CLOCK MASTER] Ativando XCLK...");

//   Serial.println("[CAMERA #1] Antes de criar OV7670 (Ativando XCLK Mestre)");

//   camera = new OV7670(
//       OV7670::Mode::QQVGA_RGB565,
//       SIOD, SIOC, VSYNC, HREF, XCLK, PCLK,
//       D0, D1, D2, D3, D4, D5, D6, D7
//   );
//   // xclkAtivo = true;

//   Serial.println("[CAMERA] OV7670 criada");

//   Serial.println("[BMP] Antes");

//   BMP::construct16BitHeader(
//       bmpHeader,
//       camera->xres,
//       camera->yres
//   );

//   Serial.println("[BMP] OK");

//   Serial.println("[TFT] Antes");

//   tft.initR(INITR_BLACKTAB);
//   tft.fillScreen(0);

//   Serial.println("[TFT] OK");

//   Serial.println("[SERVER] Antes");

//   server.begin();

//   Serial.println("[SERVER] OK");

//   Serial.println("[TRIGGER] Sistema pronto.");

//   // medirVSYNC();

//   // medirVSYNCRemoto();
//   // testarVSYNCRemoto();

//   // medirDefasagemVSYNC();
//   // medirVSYNCsJuntas();
// }

void medirVSYNCLocal() {
  Serial.println("\n========== MEDICAO VSYNC LOCAL (CAM 1 - MESTRE) ==========");
  pinMode(VSYNC_LOCAL, INPUT);

  for (int i = 0; i < 10; i++) {
    // 1. Espera subida REAL (HIGH > 10us)
    unsigned long tStartHigh = 0;
    while (true) {
      while (digitalRead(VSYNC_LOCAL) == LOW) { delayMicroseconds(1); }
      tStartHigh = micros();
      delayMicroseconds(10); 
      if (digitalRead(VSYNC_LOCAL) == HIGH) break;
    }

    // 2. Espera descida REAL (LOW > 10us)
    unsigned long tStartLow = 0;
    while (true) {
      while (digitalRead(VSYNC_LOCAL) == HIGH) { delayMicroseconds(1); }
      tStartLow = micros();
      delayMicroseconds(10);
      if (digitalRead(VSYNC_LOCAL) == LOW) break;
    }

    // 3. Espera próxima subida REAL
    unsigned long tEnd = 0;
    while (true) {
      while (digitalRead(VSYNC_LOCAL) == LOW) { delayMicroseconds(1); }
      tEnd = micros();
      delayMicroseconds(10);
      if (digitalRead(VSYNC_LOCAL) == HIGH) break;
    }

    unsigned long highTime = tStartLow - tStartHigh;
    unsigned long lowTime = tEnd - tStartLow;
    unsigned long periodo = highTime + lowTime;

    Serial.printf("[VSYNC CAM1] Ciclo %d | HIGH = %lu us | LOW = %lu us | PERIODO = %lu us\n", 
                  i + 1, highTime, lowTime, periodo);
  }
  Serial.println("========== FIM VSYNC LOCAL ==========\n");
}

void IRAM_ATTR eventoEscravaISR()
{
  timestampEventoEscrava = micros();
  eventoEscravaRecebido = true;
}

// ============================================================
// NORMALIZA DELTA PELO PERIODO DO VSYNC
//
// Resultado:
//   -40000 us ... +40000 us
//
// Assim:
//   +1000 us = escrava 1 ms depois da mestre
//   -1000 us = escrava 1 ms antes da mestre
// ============================================================

int32_t normalizarDeltaVSYNC(
    int32_t delta,
    uint32_t periodo
)
{
  int32_t meioPeriodo = periodo / 2;

  while (delta > meioPeriodo)
  {
    delta -= periodo;
  }

  while (delta < -meioPeriodo)
  {
    delta += periodo;
  }

  return delta;
}

void setup()
{
  Serial.begin(115200);

  // ============================================================
  // CONFIGURAÇÃO DOS PINOS
  // ============================================================

  pinMode(SLAVE_READY, INPUT_PULLDOWN);

  // GPIO23 recebe o pulso/evento de VSYNC enviado pela escrava.
  pinMode(VSYNC_EVENT_IN, INPUT);

  // GPIO34 = VSYNC da câmera mestre.
  // Será capturado pelo RMT, não por polling.
  pinMode(VSYNC_LOCAL_RMT, INPUT);

  wifiMulti.addAP(ssid1, password1);

  if (wifiMulti.run() == WL_CONNECTED)
  {
    Serial.println("[WIFI] Conectado!");
  }

  // ============================================================
  // HANDSHAKE
  // ============================================================

  Serial.println("[CLOCK MASTER] Aguardando READY da Escrava...");

  while (digitalRead(SLAVE_READY) == LOW)
  {
    delay(10);
  }

  Serial.println("[CLOCK MASTER] READY recebido! Ativando XCLK...");

  // ============================================================
  // CÂMERA LOCAL
  // ============================================================

  camera = new OV7670(
      OV7670::Mode::QQVGA_RGB565,
      SIOD, SIOC, VSYNC, HREF, XCLK, PCLK,
      D0, D1, D2, D3, D4, D5, D6, D7
  );

  BMP::construct16BitHeader(
      bmpHeader,
      camera->xres,
      camera->yres
  );

  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(0);

  server.begin();

  xclkAtivo = true;


  // ============================================================
  // RMT RX - VSYNC DA ESCRAVA
  // GPIO32 escrava -> GPIO23 mestre
  // ============================================================

  rmtVsyncIn = rmtInit(
      VSYNC_EVENT_IN,
      false,       // RX
      RMT_MEM_64
  );

  if (rmtVsyncIn == nullptr)
  {
    Serial.println("[RMT] ERRO ao inicializar RX GPIO23.");
  }
  else
  {
    float tickReal = rmtSetTick(rmtVsyncIn, 100.0);

    Serial.print("[RMT] RX GPIO23 inicializado. Tick = ");
    Serial.print(tickReal);
    Serial.println(" ns");

    // 2 us = 20 ticks de 100 ns
    rmtSetRxThreshold(rmtVsyncIn, 20);

    Serial.println("[RMT] RX escrava aguardando eventos...");

    rmtReadAsync(
        rmtVsyncIn,
        rmtBuffer,
        rmtBufferSize,
        nullptr,
        false,
        0
    );
  }


  // ============================================================
  // RMT RX - VSYNC DA MESTRE
  // GPIO34 = VSYNC da câmera mestre
  // ============================================================

  rmtVsyncLocal = rmtInit(
      VSYNC_LOCAL_RMT,
      false,       // RX
      RMT_MEM_64
  );

  if (rmtVsyncLocal == nullptr)
  {
    Serial.println("[RMT] ERRO ao inicializar RX GPIO34.");
  }
  else
  {
    float tickRealLocal = rmtSetTick(
        rmtVsyncLocal,
        100.0
    );

    Serial.print("[RMT] RX GPIO34 inicializado. Tick = ");
    Serial.print(tickRealLocal);
    Serial.println(" ns");

    // 2 us = 20 ticks de 100 ns
    rmtSetRxThreshold(
        rmtVsyncLocal,
        20
    );

    Serial.println(
        "[RMT] RX VSYNC mestre aguardando eventos..."
    );

    rmtReadAsync(
        rmtVsyncLocal,
        rmtLocalBuffer,
        rmtLocalBufferSize,
        nullptr,
        false,
        0
    );
  }
}
/*
 * =========================================================
 * DISPLAY
 * =========================================================
 */

void displayRGB565(
  unsigned char * frame,
  int xres,
  int yres
)
{
  tft.setAddrWindow(0, 0, yres - 1, xres - 1);

  int i = 0;

  for (int x = 0; x < xres; x++)
    for (int y = 0; y < yres; y++)
    {
      i = (y * xres + x) << 1;

      tft.pushColor(
        frame[i] |
        (frame[i + 1] << 8)
      );
    }
}



/*
 * =========================================================
 * LOOP
 * =========================================================
 */

// void loop()
// {
//   monitorarReadyEscrava();
//   serve();

//   static int vsyncAnterior = LOW;
//   static int eventoEscravaAnterior = LOW;

//   int vsyncAtual = digitalRead(VSYNC);
//   int eventoEscravaAtual = digitalRead(VSYNC_EVENT_IN);

//   // VSYNC da própria mestre
//   if (vsyncAtual == HIGH && vsyncAnterior == LOW)
//   {
//     uint32_t tMestre = micros();

//     Serial.print("[SYNC] VSYNC MESTRE: ");
//     Serial.println(tMestre);
//   }

//   // Pulso enviado pela escrava
//   if (eventoEscravaAtual == HIGH && eventoEscravaAnterior == LOW)
//   {
//     uint32_t tEscrava = micros();

//     Serial.print("[SYNC] EVENTO ESCRAVA: ");
//     Serial.println(tEscrava);
//   }

//   vsyncAnterior = vsyncAtual;
//   eventoEscravaAnterior = eventoEscravaAtual;

//   if (captureRequested)
//   {
//     captureRequested = false;

//     Serial.println("[CAPTURE] Trigger recebido.");

//     Serial.println(
//       "[CAPTURE] Capturando proximo frame..."
//     );

//     camera->oneFrame();

//     Serial.println(
//       "[CAPTURE] Frame capturado."
//     );

//     displayRGB565(
//       camera->frame,
//       camera->xres,
//       camera->yres
//     );
//   }
// }
void loop()
{
  monitorarReadyEscrava();
  serve();


  // ============================================================
  // RMT RX - VSYNC DA MESTRE
  // GPIO34
  // ============================================================

  // ============================================================
  // RMT RX - VSYNC DA MESTRE
  // GPIO34
  //
  // O RMT captura a transição por hardware.
  // Não há digitalRead() nem attachInterrupt().
  // ============================================================

  if (rmtVsyncLocal != nullptr &&
      rmtReceiveCompleted(rmtVsyncLocal))
  {
    uint32_t agora = micros();

    bool novoVsyncValido = false;

    // ----------------------------------------------------------
    // Primeiro VSYNC válido
    // ----------------------------------------------------------

    if (!existeUltimoVsyncLocal)
    {
      ultimoVsyncLocal = agora;

      existeUltimoVsyncLocal = true;

      contadorFrameLocal++;

      novoVsyncValido = true;
    }

    // ----------------------------------------------------------
    // Próximos VSYNCs
    //
    // Ignora capturas muito próximas.
    // Isso elimina a segunda captura espúria observada
    // anteriormente (~470 us).
    // ----------------------------------------------------------

    else
    {
      uint32_t intervalo =
          agora - ultimoVsyncLocal;

      if (intervalo >= VSYNC_MIN_INTERVALO_US)
      {
        penultimoVsyncLocal = ultimoVsyncLocal;
        existePenultimoVsyncLocal = true;

        ultimoVsyncLocal = agora;

        contadorFrameLocal++;

        novoVsyncValido = true;
      }
    }

    // ----------------------------------------------------------
    // Informação do VSYNC aceito
    // ----------------------------------------------------------

    if (novoVsyncValido)
    {
      Serial.print("[RMT LOCAL] VSYNC MESTRE = ");
      Serial.print(ultimoVsyncLocal);

      if (existePenultimoVsyncLocal)
      {
        uint32_t periodo =
            ultimoVsyncLocal - penultimoVsyncLocal;

        Serial.print(" | periodo = ");
        Serial.print(periodo);
        Serial.println(" us");
      }
      else
      {
        Serial.println(" | primeiro VSYNC valido");
      }
    }

    // ----------------------------------------------------------
    // Rearma imediatamente o RMT
    // ----------------------------------------------------------

    rmtLocalBufferSize = 8;

    rmtReadAsync(
        rmtVsyncLocal,
        rmtLocalBuffer,
        rmtLocalBufferSize,
        nullptr,
        false,
        0
    );
  }


  // ============================================================
  // RMT RX - VSYNC DA ESCRAVA
  // GPIO23
  //
  // Cada evento remoto será pareado com o VSYNC mestre
  // temporalmente mais próximo.
  // ============================================================

  if (rmtVsyncIn != nullptr &&
      rmtReceiveCompleted(rmtVsyncIn))
  {
    timestampVsyncEscrava = micros();

    contadorFrameRemoto++;

    Serial.print("[RMT REMOTO] VSYNC ESCRAVA = ");
    Serial.println(timestampVsyncEscrava);

    // ----------------------------------------------------------
    // Mostra o símbolo recebido
    // ----------------------------------------------------------

    if (rmtBufferSize > 0)
    {
      Serial.print("[RMT REMOTO] L0=");
      Serial.print(rmtBuffer[0].level0);

      Serial.print(" T0=");
      Serial.print(rmtBuffer[0].duration0);

      Serial.print(" | L1=");
      Serial.print(rmtBuffer[0].level1);

      Serial.print(" T1=");
      Serial.println(rmtBuffer[0].duration1);
    }

    // ----------------------------------------------------------
    // PAREAMENTO TEMPORAL
    // ----------------------------------------------------------

    if (existeUltimoVsyncLocal)
    {
      uint32_t periodoAtual =
          VSYNC_PERIODO_ESTIMADO_US;

      // Se já temos dois VSYNCs locais válidos,
      // usamos o período realmente medido.
      if (existePenultimoVsyncLocal)
      {
        periodoAtual =
            ultimoVsyncLocal - penultimoVsyncLocal;

        // Proteção contra algum valor absurdo.
        if (periodoAtual < 60000 ||
            periodoAtual > 100000)
        {
          periodoAtual =
              VSYNC_PERIODO_ESTIMADO_US;
        }
      }

      // --------------------------------------------------------
      // Diferença em relação ao último VSYNC local
      // --------------------------------------------------------

      int32_t deltaUltimo =
          (int32_t)(
              timestampVsyncEscrava -
              ultimoVsyncLocal
          );

      deltaUltimo =
          normalizarDeltaVSYNC(
              deltaUltimo,
              periodoAtual
          );

      // --------------------------------------------------------
      // Diferença em relação ao penúltimo VSYNC local
      // --------------------------------------------------------

      int32_t deltaPenultimo = 0;

      if (existePenultimoVsyncLocal)
      {
        deltaPenultimo =
            (int32_t)(
                timestampVsyncEscrava -
                penultimoVsyncLocal
            );

        deltaPenultimo =
            normalizarDeltaVSYNC(
                deltaPenultimo,
                periodoAtual
            );
      }

      // --------------------------------------------------------
      // Escolhe o VSYNC mestre mais próximo
      // --------------------------------------------------------

      uint32_t vsyncPareado;
      int32_t deltaFase;

      if (!existePenultimoVsyncLocal ||
          abs(deltaUltimo) <= abs(deltaPenultimo))
      {
        vsyncPareado = ultimoVsyncLocal;
        deltaFase = deltaUltimo;
      }
      else
      {
        vsyncPareado = penultimoVsyncLocal;
        deltaFase = deltaPenultimo;
      }

      // --------------------------------------------------------
      // FASE NORMALIZADA
      // --------------------------------------------------------

      float faseGraus =
          ((float)deltaFase /
          (float)periodoAtual) * 360.0f;

      Serial.print("[FASE] Mestre = ");
      Serial.print(vsyncPareado);

      Serial.print(" | Escrava = ");
      Serial.print(timestampVsyncEscrava);

      Serial.print(" | DELTA = ");
      Serial.print(deltaFase);

      Serial.print(" us");

      Serial.print(" | PERIODO = ");
      Serial.print(periodoAtual);

      Serial.print(" us");

      Serial.print(" | FASE = ");
      Serial.print(faseGraus, 3);

      Serial.println(" graus");
    }
    else
    {
      Serial.println(
          "[FASE] Aguardando primeiro VSYNC mestre valido."
      );
    }

    // ----------------------------------------------------------
    // Rearma imediatamente o RMT remoto
    // ----------------------------------------------------------

    rmtBufferSize = 8;

    rmtReadAsync(
        rmtVsyncIn,
        rmtBuffer,
        rmtBufferSize,
        nullptr,
        false,
        0
    );
  }

  // ============================================================
  // CAPTURA NORMAL DA CÂMERA
  // ============================================================

  if (captureRequested)
  {
    captureRequested = false;

    Serial.println("[CAPTURE] Trigger recebido.");

    Serial.println(
      "[CAPTURE] Capturando proximo frame..."
    );

    camera->oneFrame();

    Serial.println(
      "[CAPTURE] Frame capturado."
    );

    displayRGB565(
      camera->frame,
      camera->xres,
      camera->yres
    );
  }
}