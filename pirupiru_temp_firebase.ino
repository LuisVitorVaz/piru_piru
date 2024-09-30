#include <WiFi.h>
#include <FirebaseESP32.h>
#include <HardwareSerial.h>  // Biblioteca para a serial de hardware
#include <TinyGPS++.h>       // Biblioteca do GPS
#include "esp_sleep.h"       // Biblioteca para modos de sono do ESP32

// Definições WiFi
const char* ssid = "REDE20";             // Colocar o nome da rede Wi-Fi
const char* password = "20#UERGSNET99";  // Colocar a senha da rede Wi-Fi
#define qtd_coletas_ate_enviar 8
ushort coletas_ate_enviar = 0;

// Definições Firebase
#define FIREBASE_HOST "bancodedados-a7591-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "09fFbaRrhJkNPoDVwRE3TszPG2m7TeUZKWuoAJUF"
const String FIREBASE_PATH = "data"; // mostradas pino 4 esp32
// const String FIREBASE_PATH = "dados"; //torres  pino 12 esp32

// Configuração Firebase
FirebaseData firebaseData;
FirebaseConfig config;
FirebaseAuth auth;
bool firebase_flag = true;

// Criação de uma instância de HardwareSerial para o GPS
HardwareSerial serial1(1);  // Utilize UART1 do ESP32
TinyGPSPlus gps1;           // Criando o GPS
#define tempo_gps_try 5000

// Medição da temperatura
 #define PINO_SENSOR 4
// #define PINO_SENSOR 12
#define LED_BUILT_IN 2
#define DELAY_COLETA 900000  // Delay de 15min entre cada medida
int cont_amostras = 0, n_amostral = 200, cont_dados = 0, bufferIndex = 0;
float vetor_on[200] = { 0 }, vetor_off[200] = { 0 };

// Contagem de tempo
int ano;
byte mes, dia, hora, minuto;

// Criação do buffer
#define TAM_BUFFER 6000
typedef struct {
  float temperatura;
  byte ano, mes, dia, hora, minuto;
} Data;
Data dados_finais[TAM_BUFFER];

void setup() {
  pinMode(PINO_SENSOR, INPUT);
  pinMode(LED_BUILT_IN, OUTPUT);
  Serial.begin(9600);

  // Inicializa a serial para o GPS
  serial1.begin(9600, SERIAL_8N1, 19, 18);  // RX=18, TX=19

  for (int i = 0; i < TAM_BUFFER; i++) {
    dados_finais[i].temperatura = 0;
    dados_finais[i].ano = 0;
    dados_finais[i].mes = 0;
    dados_finais[i].dia = 0;
    dados_finais[i].hora = 0;
    dados_finais[i].minuto = 0;
  }
}

void light_sleep() {
  // Configura o timer para despertar o ESP32 após 15 minutos (900000000 microsegundos)
  Serial.println("ESP32 entrando em Light Sleep por 15 minutos...");
  esp_sleep_enable_timer_wakeup(DELAY_COLETA * 1000);  // 15 minutos em microsegundos
  esp_light_sleep_start();                             // Coloca o ESP32 em Light Sleep
  Serial.println("ESP32 acordou do Light Sleep!");
  Serial.println("----------------------------------------");
}

void wifi() {
  byte count_wifi_attempt = 0;
  Serial.println("Tentando conectar ao WiFi...");
  while (count_wifi_attempt < 10 && WiFi.status() != WL_CONNECTED) {
    // Conectando ao WiFi
    Serial.print(".");
    WiFi.begin(ssid, password);
    count_wifi_attempt++;
    delay(1000);
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nFalha ao tentar conectar ao WiFi. Realizando novas medidas...");
    Serial.println("----------------------------------------");
  } else {
    Serial.println("Conectado ao WiFi!");
  }
}

void gps() {
  unsigned long gps_try_start_time = millis();
  Serial.println("\nAguardando a conexão com os satélites...");

  // Leitura do GPS
  while ((millis() - gps_try_start_time) < tempo_gps_try) {
    Serial.print(".");
    while (serial1.available() > 0) {  // Realiza diversas leituras até conseguir se conectar
      char cIn = serial1.read();
      gps1.encode(cIn);
    }

    // Verifica se os dados do GPS foram atualizados
    if (gps1.location.isUpdated()) {
      Serial.println("----------------------------------------");

      // Imprime a idade da informação
      Serial.print("Idade da Informação (ms): ");
      Serial.println(gps1.location.age());

      // Obtém o Dia e Hora no GMT
      ano = gps1.date.year();
      mes = gps1.date.month();
      dia = gps1.date.day();
      hora = gps1.time.hour();
      minuto = gps1.time.minute();

      // Imprimindo os dados
      Serial.print("Data (GMT): ");
      Serial.print(dia);
      Serial.print("/");
      Serial.print(mes);
      Serial.print("/");
      Serial.println(ano);

      Serial.print("Horário (GMT): ");
      Serial.print(hora);
      Serial.print(":");
      Serial.print(minuto);

      break;  // Sai do loop se os dados do GPS forem atualizados
    }
    delay(1000);  // Adiciona um pequeno delay para evitar travar o loop
  }
  if (!gps1.location.isUpdated()) {
    Serial.println("\nNão foi possível obter dados do GPS dentro do tempo limite.");
  }
}

void firebase() {
  // Configurando Firebase
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  Serial.println("Inicializando Firebase...");
  if (Firebase.ready()) {
    Serial.println("Firebase inicializado com sucesso!");
  } else {
    Serial.printf("Erro ao inicializar Firebase: %s\n", firebaseData.errorReason().c_str());
  }

  for (int i = 0; i < cont_dados; i++) {
    // Criar JSON para enviar
    FirebaseJson json;
    json.set("data", String(dados_finais[i].dia) + "/" + String(dados_finais[i].mes) + "/20" + String(dados_finais[i].ano));
    json.set("hora", String(dados_finais[i].hora) + ":" + String(dados_finais[i].minuto));
    json.set("temperatura", dados_finais[i].temperatura);

    // Enviar dados ao Firebase
    Serial.println("Enviando dados ao Firebase...");
    if (Firebase.pushJSON(firebaseData, FIREBASE_PATH, json)) {
      Serial.println("Dados enviados com sucesso");
    } else {
      Serial.printf("Erro ao enviar dados: %s\n", firebaseData.errorReason().c_str());
    }
    delay(1000);
  }
}

void temperatura() {  // Função que realiza as coletas de tempo de alto e baixo da onda do oscilador
  if (cont_dados < TAM_BUFFER) {
    int estado_pino;
    float tempo_on = 0, tempo_off = 0;
    bool flag_coleta = true;
    Serial.println("\nIniciando uma nova medição de temperatura...");
    estado_pino = digitalRead(PINO_SENSOR);  // Lê o estado do pino (HIGH ou LOW)

    while (flag_coleta) {
      // Captura o tempo de onda alta
      while (estado_pino == HIGH) {
        tempo_on += 1;
        estado_pino = digitalRead(PINO_SENSOR);
      }

      // Captura o tempo de onda baixa
      while (estado_pino == LOW) {
        tempo_off += 1;
        estado_pino = digitalRead(PINO_SENSOR);
      }

      vetor_on[cont_amostras] = tempo_on;
      vetor_off[cont_amostras] = tempo_off;

      cont_amostras++;
      if (cont_amostras == n_amostral) {
        duty_cycle();
        cont_dados++;
        cont_amostras = 0;
        flag_coleta = false;
        coletas_ate_enviar++;
        Serial.print("Quantidade de coletas até enviar: ");
        Serial.print(coletas_ate_enviar);
        Serial.println("\n----------------------------------------");
      }
    }
  } else {
    Serial.println("Buffer cheio!");
  }
}

void duty_cycle() {  // Função que faz as médias das coletas de tempo, calcula a temperatura guarda tudo no buffer
  float duty_cycle = 0, tempo_alto = 0, soma_tempo_alto = 0, tempo_baixo = 0, soma_tempo_baixo = 0, temperatura;

  Serial.println("Calculando a temperatura...");
  for (int i = 0; i < n_amostral; i++) {
    soma_tempo_alto += vetor_on[i];
  }
  tempo_alto = soma_tempo_alto / n_amostral;
  for (int i = 0; i < n_amostral; i++) {
    soma_tempo_baixo += vetor_off[i];
  }
  tempo_baixo = soma_tempo_baixo / n_amostral;
  duty_cycle = tempo_alto / tempo_baixo;
  if (duty_cycle < 1.6) {
    temperatura = (duty_cycle - 1.0182) / 0.0119;
  } else {
    temperatura = (duty_cycle + 0.45) / 0.04;
  }
  // temperatura--;  // "Fator de correção" (O ESP32 está medindo temperaturas em torno de 1ºC a 1,5ºC maiores que o Arduino UNO, por isso o decréscimo)
  dados_finais[cont_dados].temperatura = temperatura;
  dados_finais[cont_dados].ano = ano % 2000;
  dados_finais[cont_dados].mes = mes;
  dados_finais[cont_dados].dia = dia;
  dados_finais[cont_dados].hora = hora;
  dados_finais[cont_dados].minuto = minuto;

  digitalWrite(LED_BUILT_IN, HIGH);  // Pisca o LED da ESP sempre que uma nova coleta é concluída

  Serial.print("Dados coletados: ");
  Serial.print(dados_finais[cont_dados].dia);
  Serial.print("/");
  Serial.print(dados_finais[cont_dados].mes);
  Serial.print("/");
  Serial.print(dados_finais[cont_dados].ano);
  Serial.print(" ");
  Serial.print(dados_finais[cont_dados].hora);
  Serial.print(":");
  Serial.print(dados_finais[cont_dados].minuto);
  Serial.print(" -> ");
  Serial.print(dados_finais[cont_dados].temperatura);
  Serial.println("ºC\n");

  delay(100);
  digitalWrite(LED_BUILT_IN, LOW);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {  // Apenas coleta dados se o Wi-Fi não estiver conectado
    gps();
    temperatura();
    light_sleep();
    if (coletas_ate_enviar >= qtd_coletas_ate_enviar) {
      wifi();
    }
  } else {  // Se o Wi-Fi for conectado, envia todos os dados que estiverem no buffer
    if (firebase_flag) {
      firebase();
      firebase_flag = false;
    } else {
      // Serial.println("Os dados já foram enviados...");
      WiFi.disconnect(true);  // Passar true desabilita o modo Wi-Fi completamente
      Serial.println("Desconectado do WiFi...");
      coletas_ate_enviar = 0;
      firebase_flag = true;
      cont_dados = 0;
    }
  }
}
