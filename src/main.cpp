// ============================================================================
// Robô autônomo desviador de obstáculos
// Arduino Uno + L298N + HC-SR04 sobre servo SG90
// Chassi 2WD: motores nas rodas traseiras, rodízio giratório na frente
// Servo: 90° = frente, 0° = direita, 180° = esquerda
// ============================================================================

#include <Arduino.h>
#include <Servo.h>

// ----------------------------------------------------------------------------
// Modos de operação
// ----------------------------------------------------------------------------
const bool DEBUG             = true;   // Serial: estado, escaneamento e ângulo escolhido
const bool MODO_CALIBRACAO   = false;  // gira 90°, para 3 s e repete (ajuste de MS_POR_GRAU)
const bool MODO_TESTE_SENSOR = false;  // só imprime as distâncias, com os motores desligados
const bool MODO_TESTE_SERVO  = false;  // alterna esquerda, frente e direita, motores desligados

// ----------------------------------------------------------------------------
// Pinagem
// A biblioteca Servo usa o Timer1 e desativa o PWM nos pinos 9 e 10 do Uno,
// por isso ENA/ENB não podem usar esses pinos.
// ----------------------------------------------------------------------------
const uint8_t ENA   = 5;   // PWM do motor esquerdo (motor A do L298N)
const uint8_t IN1   = 7;
const uint8_t IN2   = 8;
const uint8_t IN3   = 11;
const uint8_t IN4   = 12;
const uint8_t ENB   = 6;   // PWM do motor direito (motor B do L298N)
const uint8_t SERVO = 9;
const uint8_t TRIG  = A0;
const uint8_t ECHO  = A1;

// ----------------------------------------------------------------------------
// Velocidades (PWM 0..255)
// Os motores são de 6 V e recebem ~10 V: NUNCA subir PWM_LIMITE acima de 170.
// ----------------------------------------------------------------------------
const int PWM_LIMITE = 170;  // teto absoluto, aplicado em setMotores()
const int VEL_MAX    = 150;  // velocidade normal
const int VEL_LENTA  = 110;  // velocidade com obstáculo a menos de DIST_LENTO
const int VEL_GIRO   = 130;  // giro no próprio eixo
const int VEL_RE     = 130;  // marcha à ré

// Compensação se o robô puxar para um lado. Valores somados ao PWM de cada
// motor: se ele puxa para a direita, o motor esquerdo está mais forte, então
// use VEL_ESQ_TRIM negativo (ou VEL_DIR_TRIM positivo).
const int VEL_ESQ_TRIM = 0;
const int VEL_DIR_TRIM = 0;

// Troque para true se a roda girar ao contrário do comando.
const bool INVERTER_ESQ = false;
const bool INVERTER_DIR = false;

// ----------------------------------------------------------------------------
// Distâncias (cm)
// ----------------------------------------------------------------------------
const int DIST_PARADA      = 35;   // leitura frontal abaixo disso: para e desvia

// Leitura em 60°/120° abaixo disso: para e desvia. A 30° do eixo, o objeto fica a
// metade da distância para o lado: acima de 22 cm ele passa fora do robô, e parar
// por isso impediria de andar em linha reta até o próximo obstáculo.
const int DIST_PARADA_LATERAL = 22;

const int DIST_LENTO       = 60;   // abaixo disso: anda em VEL_LENTA
const int DIST_LIVRE       = 50;   // mínimo para considerar uma direção livre (> DIST_PARADA)
const int DIST_SEM_ECO     = 400;  // valor assumido quando não há eco
const int DIST_MINIMA      = 2;    // leituras menores que isso são descartadas
const int VARIACAO_TRAVADO = 3;    // variação frontal mínima para provar movimento

// ----------------------------------------------------------------------------
// Tempos (ms, exceto onde indicado)
// ----------------------------------------------------------------------------
const float         MS_POR_GRAU         = 2.0;    // CALIBRAR no piso real (aceita fração, ex.: 2.4)
const unsigned long TEMPO_RAMPA        = 300;    // aceleração de 0 até a velocidade alvo
const unsigned long INTERVALO_VARREDURA = 150;    // tempo entre passos da varredura
const unsigned long TEMPO_TRAVADO       = 3000;   // sem variação por esse tempo = travado

// Sem eco a leitura fica fixa em 400 cm mesmo com o robô andando em local amplo,
// então exige mais tempo. Não é ignorado porque parede inclinada também não
// devolve eco, e o robô ficaria preso nela com os motores forçando.
const unsigned long TEMPO_TRAVADO_SEM_ECO = 8000;

const unsigned long TEMPO_FREIO         = 100;    // freio antes de engatar a ré
const unsigned long TEMPO_RE            = 200;    // duração da ré
const unsigned long PAUSA_RODIZIO       = 150;    // espera o rodízio acomodar após a ré
const unsigned long PAUSA_GIRO          = 150;    // pausa depois de girar
const unsigned long ESPERA_SERVO        = 250;    // servo chegar e parar de vibrar
const unsigned long PAUSA_ENTRE_DIRECOES = 500;   // aguarda entre esquerda, frente e direita
const unsigned long INTERVALO_LEITURAS  = 60;     // entre pings, evita pegar eco do anterior
const unsigned long ESPERA_INICIAL      = 2000;   // tempo para colocar o robô no chão
const unsigned long PAUSA_CALIBRACAO    = 3000;   // parado entre giros em MODO_CALIBRACAO
const unsigned long PAUSA_TESTE_SERVO   = 2000;   // parado em cada posição em MODO_TESTE_SERVO
const unsigned long TIMEOUT_ECO_US      = 25000;  // timeout do pulseIn, em microssegundos

// ----------------------------------------------------------------------------
// Posições do servo (largura de pulso em microssegundos)
// Menos pulso = mais para a direita. Para virar ainda mais, reduza PULSO_DIREITA
// de 50 em 50; se o servo zumbir parado na direita, chegou ao batente: volte 50.
// PULSO_FRENTE corrige o sensor se ele não apontar reto em 90°.
// Se esquerda e direita saírem trocadas (servo de cabeça para baixo), troque os
// valores de PULSO_DIREITA e PULSO_ESQUERDA entre si.
// ----------------------------------------------------------------------------
const int PULSO_DIREITA  = 544;
const int PULSO_FRENTE   = 1472;
const int PULSO_ESQUERDA = 2400;

// ----------------------------------------------------------------------------
// Comportamento
// ----------------------------------------------------------------------------
const bool VARREDURA_EM_MOVIMENTO = true;  // false: sensor fixo em 90° ao avançar

// ----------------------------------------------------------------------------
// Constantes internas
// ----------------------------------------------------------------------------
const int ANGULO_FRENTE    = 90;
const int LEITURA_INVALIDA = -1;    // retorno de medirDistancia() para leitura descartada
const int BECO_SEM_SAIDA   = -1;    // retorno de escanear() quando não há saída
const int SEM_REFERENCIA   = -100;  // nenhuma leitura de referência para o travamento
const int GRAUS_MEIA_VOLTA = 180;
const int GRAUS_CORRECAO   = 30;    // passo extra quando o giro não alinhou com a saída
const int MAX_CORRECOES    = 4;     // passos extras antes de desistir e escanear de novo
const int NUM_AMOSTRAS     = 3;     // leituras por mediana

const int ANGULOS_ESCANEAMENTO[] = {180, 90, 0};  // esquerda, frente, direita
const char* const NOMES_DIRECAO[] = {"Esquerda", "Frente", "Direita"};
const int NUM_ANGULOS = sizeof(ANGULOS_ESCANEAMENTO) / sizeof(ANGULOS_ESCANEAMENTO[0]);

// Começa em 90° porque, com a varredura desligada, só a primeira posição é usada.
const int ANGULOS_VARREDURA[] = {90, 120, 90, 60};
const int NUM_PASSOS_VARREDURA = sizeof(ANGULOS_VARREDURA) / sizeof(ANGULOS_VARREDURA[0]);

static_assert(MODO_CALIBRACAO + MODO_TESTE_SENSOR + MODO_TESTE_SERVO <= 1, "Ative apenas um modo por vez");
static_assert(VEL_MAX <= PWM_LIMITE && VEL_LENTA <= PWM_LIMITE, "Velocidade acima de PWM_LIMITE");
static_assert(VEL_GIRO <= PWM_LIMITE && VEL_RE <= PWM_LIMITE, "Velocidade acima de PWM_LIMITE");
static_assert(PWM_LIMITE <= 170, "Motores de 6 V: PWM_LIMITE nao pode passar de 170");

// ----------------------------------------------------------------------------
// Tipos e protótipos
// ----------------------------------------------------------------------------
enum Estado { AVANCAR, PARAR, ESCANEAR, GIRAR, CONFIRMAR };

// Sensor
int medirDistancia();
int medirMediana();

// Motores
void aplicarMotor(uint8_t pinoEn, uint8_t pinoA, uint8_t pinoB, int vel, int trim, bool inverter);
void setMotores(int esq, int dir);
void frente(int vel);
void re(int vel);
void girarDireita(int vel);
void girarEsquerda(int vel);
void parar();
void darRe(unsigned long duracao);

// Escaneamento
void moverServo(int angulo);
int escanear();

// Máquina de estados
void mudarEstado(Estado novo);
void iniciarAvanco();
int menorDistanciaVarredura();
bool detectouTravamento(int distFrontal, unsigned long agora);
void passoAvancar();
void passoParar();
void passoEscanear();
void passoGirar();
void passoConfirmar();

// Modos auxiliares
void loopTesteSensor();
void loopTesteServo();
void loopCalibracao();

// ----------------------------------------------------------------------------
// Estado global
// ----------------------------------------------------------------------------
const char* const NOMES_ESTADO[] = {"AVANCAR", "PARAR", "ESCANEAR", "GIRAR", "CONFIRMAR"};

Servo servo;
Estado estado = AVANCAR;

unsigned long inicioRampa   = 0;
unsigned long ultimaLeitura = 0;
int passoVarredura          = 0;
int distVarredura[NUM_PASSOS_VARREDURA];  // última leitura de cada passo da varredura

int distRefTravado            = SEM_REFERENCIA;
unsigned long tempoRefTravado = 0;

// A última parada veio de algo que seguir reto não resolve (ver escanear).
bool evitarFrente = false;

int grausGiro        = 0;
int correcoesGiro    = 0;
bool giroParaDireita = true;

// ----------------------------------------------------------------------------
// Inicialização
// ----------------------------------------------------------------------------
void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  setMotores(0, 0);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIG, LOW);

  moverServo(ANGULO_FRENTE);  // antes do attach, para não dar tranco na partida
  servo.attach(SERVO, min(PULSO_DIREITA, PULSO_ESQUERDA), max(PULSO_DIREITA, PULSO_ESQUERDA));

  Serial.begin(9600);
  delay(ESPERA_INICIAL);

  if (!MODO_TESTE_SENSOR && !MODO_TESTE_SERVO && !MODO_CALIBRACAO) {
    mudarEstado(AVANCAR);
  }
}

void loop() {
  if (MODO_TESTE_SENSOR) {
    loopTesteSensor();
    return;
  }
  if (MODO_TESTE_SERVO) {
    loopTesteServo();
    return;
  }
  if (MODO_CALIBRACAO) {
    loopCalibracao();
    return;
  }

  switch (estado) {
    case AVANCAR:   passoAvancar();   break;
    case PARAR:     passoParar();     break;
    case ESCANEAR:  passoEscanear();  break;
    case GIRAR:     passoGirar();     break;
    case CONFIRMAR: passoConfirmar(); break;
  }
}

// ----------------------------------------------------------------------------
// Sensor HC-SR04
// ----------------------------------------------------------------------------

// Retorna a distância em cm, DIST_SEM_ECO sem eco, ou LEITURA_INVALIDA se a
// leitura for absurda.
int medirDistancia() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  unsigned long duracao = pulseIn(ECHO, HIGH, TIMEOUT_ECO_US);
  if (duracao == 0) {
    return DIST_SEM_ECO;
  }

  int cm = (int)(duracao / 58UL);
  if (cm < DIST_MINIMA) {
    return LEITURA_INVALIDA;
  }
  return min(cm, DIST_SEM_ECO);
}

// Mediana de NUM_AMOSTRAS leituras válidas. Tenta o dobro de vezes para
// compensar descartes; se nenhuma for válida, retorna 0 (trata como bloqueado,
// já que leitura menor que 2 cm indica algo colado no sensor).
int medirMediana() {
  int amostras[NUM_AMOSTRAS];
  int n = 0;

  for (int tentativa = 0; tentativa < NUM_AMOSTRAS * 2 && n < NUM_AMOSTRAS; tentativa++) {
    int d = medirDistancia();
    if (d != LEITURA_INVALIDA) {
      int i = n;
      while (i > 0 && amostras[i - 1] > d) {  // mantém o vetor ordenado
        amostras[i] = amostras[i - 1];
        i--;
      }
      amostras[i] = d;
      n++;
    }
    delay(INTERVALO_LEITURAS);
  }

  if (n == 0) {
    return 0;
  }
  return amostras[(n - 1) / 2];  // com n par, fica com o menor dos centrais
}

// ----------------------------------------------------------------------------
// Motores
// ----------------------------------------------------------------------------

// O PWM é escrito antes dos pinos de direção para que a troca de direção nunca
// aconteça com um PWM anterior mais alto.
void aplicarMotor(uint8_t pinoEn, uint8_t pinoA, uint8_t pinoB, int vel, int trim, bool inverter) {
  if (vel == 0) {
    analogWrite(pinoEn, 0);
    digitalWrite(pinoA, LOW);
    digitalWrite(pinoB, LOW);
    return;
  }

  int pwm = constrain(abs(vel) + trim, 0, PWM_LIMITE);
  bool sentidoDireto = (vel > 0) != inverter;

  analogWrite(pinoEn, pwm);
  digitalWrite(pinoA, sentidoDireto ? HIGH : LOW);
  digitalWrite(pinoB, sentidoDireto ? LOW : HIGH);
}

// Valores de -255 a 255; positivo = para frente. PWM_LIMITE sempre é respeitado.
void setMotores(int esq, int dir) {
  aplicarMotor(ENA, IN1, IN2, esq, VEL_ESQ_TRIM, INVERTER_ESQ);
  aplicarMotor(ENB, IN3, IN4, dir, VEL_DIR_TRIM, INVERTER_DIR);
}

void frente(int vel) {
  setMotores(vel, vel);
}

void re(int vel) {
  setMotores(-vel, -vel);
}

void girarDireita(int vel) {
  setMotores(vel, -vel);
}

void girarEsquerda(int vel) {
  setMotores(-vel, vel);
}

// Freio ativo: com IN iguais e EN ligado, o L298N curto-circuita o motor. Não
// aplica tensão nele, ao contrário de setMotores(0, 0), que só deixa a roda livre.
void parar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, PWM_LIMITE);
  analogWrite(ENB, PWM_LIMITE);
}

void darRe(unsigned long duracao) {
  re(VEL_RE);
  delay(duracao);
  parar();
  delay(PAUSA_RODIZIO);
}

// ----------------------------------------------------------------------------
// Escaneamento
// ----------------------------------------------------------------------------

// 0° = PULSO_DIREITA, 90° = PULSO_FRENTE, 180° = PULSO_ESQUERDA. Cada metade é
// interpolada separadamente para que ajustar um lado não desloque a frente.
void moverServo(int angulo) {
  long pulso = (angulo <= ANGULO_FRENTE)
                   ? map(angulo, 0, ANGULO_FRENTE, PULSO_DIREITA, PULSO_FRENTE)
                   : map(angulo, ANGULO_FRENTE, 180, PULSO_FRENTE, PULSO_ESQUERDA);
  servo.writeMicroseconds((int)pulso);
}

// Verifica uma direção por vez, na ordem de ANGULOS_ESCANEAMENTO, e fica com a
// primeira livre, sem olhar as seguintes. Retorna o ângulo ou BECO_SEM_SAIDA.
//
// Com evitarFrente, 90° é pulado. Só acontece depois de um travamento: o sensor
// pode ver a frente livre (obstáculo baixo) e seguir reto prenderia o robô de novo.
int escanear() {
  int escolhido = BECO_SEM_SAIDA;
  bool primeira = true;

  for (int i = 0; i < NUM_ANGULOS && escolhido == BECO_SEM_SAIDA; i++) {
    int angulo = ANGULOS_ESCANEAMENTO[i];
    if (evitarFrente && angulo == ANGULO_FRENTE) {
      continue;
    }

    // A pausa também cobre o maior movimento do servo: 180° ao pular a frente.
    moverServo(angulo);
    delay(primeira ? ESPERA_SERVO : PAUSA_ENTRE_DIRECOES);
    primeira = false;

    int d = medirMediana();

    if (DEBUG) {
      Serial.print(F("  "));
      Serial.print(NOMES_DIRECAO[i]);
      Serial.print(F(": "));
      Serial.print(d);
      Serial.println(F(" cm"));
    }

    if (d >= DIST_LIVRE) {
      escolhido = angulo;
    }
  }

  moverServo(ANGULO_FRENTE);  // volta ao centro enquanto o robô gira
  evitarFrente = false;

  if (DEBUG) {
    if (escolhido == BECO_SEM_SAIDA) {
      Serial.println(F("  Beco sem saida"));
    } else {
      Serial.print(F("  Angulo escolhido: "));
      Serial.println(escolhido);
    }
  }
  return escolhido;
}

// ----------------------------------------------------------------------------
// Máquina de estados
// ----------------------------------------------------------------------------

void mudarEstado(Estado novo) {
  estado = novo;

  if (DEBUG) {
    Serial.print(F("Estado: "));
    Serial.println(NOMES_ESTADO[novo]);
  }

  if (novo == AVANCAR) {
    iniciarAvanco();
  }
}

void iniciarAvanco() {
  unsigned long agora = millis();

  inicioRampa = agora;
  ultimaLeitura = agora;
  tempoRefTravado = agora;
  distRefTravado = SEM_REFERENCIA;

  passoVarredura = 0;
  for (int i = 0; i < NUM_PASSOS_VARREDURA; i++) {
    distVarredura[i] = DIST_SEM_ECO;
  }
  moverServo(ANGULOS_VARREDURA[0]);
}

// Usa o ciclo inteiro da varredura: olhar só a última leitura faria a velocidade
// oscilar a cada passo quando houver algo perto de um lado só.
int menorDistanciaVarredura() {
  int menor = DIST_SEM_ECO;
  for (int i = 0; i < NUM_PASSOS_VARREDURA; i++) {
    if (distVarredura[i] < menor) {
      menor = distVarredura[i];
    }
  }
  return menor;
}

bool detectouTravamento(int distFrontal, unsigned long agora) {
  if (abs(distFrontal - distRefTravado) >= VARIACAO_TRAVADO) {
    distRefTravado = distFrontal;
    tempoRefTravado = agora;
    return false;
  }

  unsigned long limite = (distFrontal >= DIST_SEM_ECO) ? TEMPO_TRAVADO_SEM_ECO : TEMPO_TRAVADO;
  return agora - tempoRefTravado >= limite;
}

// Não bloqueia: roda a cada volta do loop() e só mede a cada INTERVALO_VARREDURA.
void passoAvancar() {
  unsigned long agora = millis();

  int alvo = (menorDistanciaVarredura() < DIST_LENTO) ? VEL_LENTA : VEL_MAX;
  unsigned long decorrido = agora - inicioRampa;
  if (decorrido < TEMPO_RAMPA) {
    alvo = (int)((unsigned long)alvo * decorrido / TEMPO_RAMPA);
  }
  frente(alvo);

  if (agora - ultimaLeitura < INTERVALO_VARREDURA) {
    return;
  }
  ultimaLeitura = agora;

  // Mede antes de mover: o servo teve o intervalo inteiro para chegar à posição.
  int passoAtual = passoVarredura;
  int angulo = ANGULOS_VARREDURA[passoAtual];
  int d = medirDistancia();

  if (VARREDURA_EM_MOVIMENTO) {
    passoVarredura = (passoVarredura + 1) % NUM_PASSOS_VARREDURA;
    moverServo(ANGULOS_VARREDURA[passoVarredura]);
  }

  if (d == LEITURA_INVALIDA) {
    return;
  }
  distVarredura[passoAtual] = d;

  if (d < (angulo == ANGULO_FRENTE ? DIST_PARADA : DIST_PARADA_LATERAL)) {
    if (DEBUG) {
      Serial.print(F("Obstaculo a "));
      Serial.print(d);
      Serial.print(F(" cm em "));
      Serial.print(angulo);
      Serial.println(F(" graus"));
    }
    mudarEstado(PARAR);
    return;
  }

  if (angulo == ANGULO_FRENTE && detectouTravamento(d, agora)) {
    if (DEBUG) {
      Serial.println(F("Travamento detectado"));
    }
    evitarFrente = true;
    mudarEstado(PARAR);
  }
}

void passoParar() {
  parar();
  delay(TEMPO_FREIO);

  if (evitarFrente) {
    darRe(TEMPO_RE);  // travado: precisa se soltar antes de conseguir girar
  }
  mudarEstado(ESCANEAR);
}

// Frente livre: segue reto. Esquerda ou direita livre: vira para lá e anda.
// Tudo bloqueado: pequena ré, meia-volta e analisa de novo.
void passoEscanear() {
  int angulo = escanear();
  correcoesGiro = 0;

  if (angulo == ANGULO_FRENTE) {
    mudarEstado(AVANCAR);
    return;
  }

  if (angulo == BECO_SEM_SAIDA) {
    darRe(TEMPO_RE);
    grausGiro = GRAUS_MEIA_VOLTA;
    giroParaDireita = true;
  } else {
    grausGiro = abs(angulo - ANGULO_FRENTE);
    giroParaDireita = angulo < ANGULO_FRENTE;
  }
  mudarEstado(GIRAR);
}

void passoGirar() {
  if (grausGiro > 0) {
    if (giroParaDireita) {
      girarDireita(VEL_GIRO);
    } else {
      girarEsquerda(VEL_GIRO);
    }
    delay((unsigned long)(grausGiro * MS_POR_GRAU));
    parar();
    delay(PAUSA_GIRO);
  }
  mudarEstado(grausGiro == GRAUS_MEIA_VOLTA ? ESCANEAR : CONFIRMAR);
}

void passoConfirmar() {
  moverServo(ANGULO_FRENTE);
  delay(ESPERA_SERVO);
  int d = medirMediana();

  if (DEBUG) {
    Serial.print(F("  Frente: "));
    Serial.print(d);
    Serial.println(F(" cm"));
  }

  if (d >= DIST_LIVRE) {
    mudarEstado(AVANCAR);
    return;
  }

  // O giro é por tempo e pode parar antes da saída. Continua no mesmo sentido até
  // o sensor ver o caminho livre: escanear agora poderia escolher o lado oposto e
  // desfazer o giro, deixando o robô indo de um lado para o outro sem sair.
  if (grausGiro > 0 && correcoesGiro < MAX_CORRECOES) {
    correcoesGiro++;
    grausGiro = GRAUS_CORRECAO;
    mudarEstado(GIRAR);
    return;
  }

  mudarEstado(ESCANEAR);
}

// ----------------------------------------------------------------------------
// Modos auxiliares
// ----------------------------------------------------------------------------

void loopTesteSensor() {
  int d = medirDistancia();

  if (d == LEITURA_INVALIDA) {
    Serial.println(F("Leitura descartada"));
  } else if (d >= DIST_SEM_ECO) {
    Serial.println(F("Sem eco (400 cm)"));
  } else {
    Serial.print(d);
    Serial.println(F(" cm"));
  }
  delay(200);
}

// Para em cada direção por tempo suficiente para conferir a montagem do sensor.
void loopTesteServo() {
  for (int i = 0; i < NUM_ANGULOS; i++) {
    moverServo(ANGULOS_ESCANEAMENTO[i]);
    delay(ESPERA_SERVO);

    Serial.print(NOMES_DIRECAO[i]);
    Serial.print(F(": "));
    Serial.print(medirMediana());
    Serial.println(F(" cm"));

    delay(PAUSA_TESTE_SERVO);
  }
}

// Sempre para a direita: 4 giros devem fechar uma volta completa.
void loopCalibracao() {
  girarDireita(VEL_GIRO);
  delay((unsigned long)(90 * MS_POR_GRAU));
  parar();
  delay(PAUSA_CALIBRACAO);
}
