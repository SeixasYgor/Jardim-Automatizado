#include "RTClib.h" //INCLUSÃO DA BIBLIOTECA
#include <dht.h>  
#include "Ultrasonic.h"
#include <LiquidCrystal_I2C.h> // Biblioteca utilizada para fazer a comunicação com o display 20x4 

#define col 16 // Serve para definir o numero de colunas do display utilizado
#define lin  2 // Serve para definir o numero de linhas do display utilizado
#define ende  0x27 // Serve para definir o endereço do display.


const int NUM_BOMBS = 2;
const int Relay_Bomb_Pin[NUM_BOMBS] = {3, 4}; // Pinos dos relés das bombas

const int Relay_Lamp_1  = 6;
const int Relay_Lamp_2  = 7;
const int Relay_Higro   = 13;
const int inDHT         = 5;
const int botao1        = 8;
const int botao2        = 9;
const int botao3        = 10;

//--------------------------------------------------------------
//CONSTANTES PARA SENSORES DE UMIDADE DO SOLO
const int pinoSensorHigro1  = A0; //PINO UTILIZADO PELO SENSOR
const int pinoSensorHigro2  = A1; //PINO UTILIZADO PELO SENSOR
//--------------------------------------------------------------

//--------------------------------------------------------------
//CONSTANTES PARA SENSOR DE LUMINOSIDADE
const int pinoSensorLDR     = A2; //PINO UTILIZADO PELO SENSOR
//--------------------------------------------------------------

//--------------------------------------------------------------
//DEFINIÇÃO DE HORARIO FINAL E INICIAL DOS MOMENTOS QUE SE PODE LIGAR AS BOMBAS
const int HOUR_START_01 = 6;
const int MINUTE_START_01 = 30;
const int HOUR_END_01 = 9;
const int MINUTE_END_01 = 30;

const int HOUR_START_02 = 16;
const int MINUTE_START_02 = 0;
const int HOUR_END_02 = 18;
const int MINUTE_END_02 = 0;
//--------------------------------------------------------------

//--------------------------------------------------------------
//DEFINIÇÃO DE PARAMETROS DE SOLO SECO E MOLHADO
const int analogSoloSeco = 400; //VALOR MEDIDO COM O SOLO SECO (VOCÊ PODE FAZER TESTES E AJUSTAR ESTE VALOR)
const int analogSoloMolhado = 150; //VALOR MEDIDO COM O SOLO MOLHADO (VOCÊ PODE FAZER TESTES E AJUSTAR ESTE VALOR)
const int percSoloSeco = 0; //MENOR PERCENTUAL DO SOLO SECO (0% - NÃO ALTERAR)
const int percSoloMolhado = 100; //MAIOR PERCENTUAL DO SOLO MOLHADO (100% - NÃO ALTERAR)
const int percMinSoloSeco = 60; //Valor minimo para considerar um solo seco e acionar uma bomba
const int percMaxSoloSeco = 90; //Valor maximo para NÃO considerar um solo seco e acionar uma bomba
//--------------------------------------------------------------

//--------------------------------------------------------------
//SETUP PARA MENSAGENS DO SISTEMA
const int NUM_MESSAGES = 6;      //TOTAL DE MENSAGENS SUPORTADAS
const int MESSAGE_LAMP_01 = 0;    //POSIÇÃO DA MENSAGEM DA LAMPADA 01
const int MESSAGE_LAMP_02 = 1;    //POSIÇÃO DA MENSAGEM DA LAMPADA 02
const int MESSAGE_BOMB_01 = 2;    //POSIÇÃO DA MENSAGEM DA BOMBA 01
const int MESSAGE_BOMB_02 = 3;    //POSIÇÃO DA MENSAGEM DA BOMBA 02
const int MESSAGE_TERRA_SECA_01 = 4;    //POSIÇÃO DA MENSAGEM DA BOMBA 02
const int MESSAGE_TERRA_SECA_02 = 5;    //POSIÇÃO DA MENSAGEM DA BOMBA 02

const char* messages[NUM_MESSAGES]; //DECLARÇÃO DO ARRAY QUE VAI SALVAR AS MENSAGENS
//--------------------------------------------------------------

int readDataDHT;
int temp;
int umid;

char Dados_Serial;

char daysOfTheWeek[7][12] = {"Domingo", "Segunda", "Terça", "Quarta", "Quinta", "Sexta", "Sábado"};


//--------------------------------------------------------------
//CONTROLE DE TEMPO E EXIBIÇÃO DE MENSAGENS DO SISTEMA
const unsigned long interval_01 = 1000; // Define o intervalo de 5 segundos
unsigned long previousMillis_01 = 0; // Variável que armazena o tempo da última execução
int messageIndex = 0; // Índice da mensagem atual
//--------------------------------------------------------------

//--------------------------------------------------------------
//CONTROLE DE TEMPO DA LEITURA PERIODICA DOS HIGROMETROS.
//const unsigned long interval_02 = 1200000; // Define o intervalo de 20 min
const unsigned long interval_02 = 5000; // Define o intervalo de 
unsigned long previousMillis_02 = 0; // Variável que armazena o tempo da última execução
//--------------------------------------------------------------

//--------------------------------------------------------------
//VARIAVEIS PARA CONTROLE DE UMIDADE DO SOLO
long valor_umid_solo_01 = 0;
long valor_umid_solo_02 = 0;
bool terraSeca_01 = true;
bool terraSeca_02 = true;
//--------------------------------------------------------------

//--------------------------------------------------------------
//VARIAVEIS PARA CONTROLE DAS LAMPADAS
bool sinal_lamp_01;
bool sinal_lamp_02;
//--------------------------------------------------------------

//--------------------------------------------------------------
//VARIAVEIS PARA CONTROLE DAS BOMBAS
bool sinal_bomb_01;
bool sinal_bomb_02;
//--------------------------------------------------------------

//--------------------------------------------------------------
//VARIAVEL PARA RECEBER VALOR DE LUMINISIDADE
int valor_luminosidade;
//--------------------------------------------------------------

//--------------------------------------------------------------
//VARIAVEL PARA RECEBER ESTADO DOS BOTOES
bool estadoBotao1;
bool estadoBotao2;
bool estadoBotao3;
//--------------------------------------------------------------

dht DHT;
HC_SR04 sensor1(11,12); 
RTC_DS1307 rtc; //OBJETO DO TIPO RTC_DS1307
LiquidCrystal_I2C lcd(ende,col,lin); // Chamada da funcação LiquidCrystal para ser usada com o I2C
//--------------------------------------------------------------

//--------------------------------------------------------------
void setup() {
  
  lcd.init(); // Serve para iniciar a comunicação com o display já conectado
  lcd.backlight(); // Serve para ligar a luz do display
  lcd.clear(); // Serve para limpar a tela do display
  
  lcd.setCursor(0,0); // Coloca o cursor do display na coluna 0 e linha 0
  lcd.print("Iniciando em"); // Comando de saída com a mensagem que deve aparecer na coluna 2 e linha 1.
  lcd.setCursor(0,1); // Coloca o cursor do display na coluna 0 e linha 1
  lcd.print("05 segundos");
  Serial.begin(9600);
  
  Serial.println ("Iniciando sistema em 5 segundos");
  for(int i = 0; i < 5; i++) { // loop que se repete 10 vezes
    lcd.print(".");
    delay(1000); // espera 1 segundo
  }
  lcd.clear();

  pinMode(Relay_Lamp_1, OUTPUT);
  pinMode(Relay_Lamp_2, OUTPUT); 

  pinMode(Relay_Bomb_Pin[0], OUTPUT); 
  pinMode(Relay_Bomb_Pin[1], OUTPUT); 
  pinMode(Relay_Higro, OUTPUT); 

  pinMode(botao1,INPUT_PULLUP);
  pinMode(botao2,INPUT_PULLUP);
  pinMode(botao3,INPUT_PULLUP);

  pinMode(pinoSensorHigro1, INPUT);
  pinMode(pinoSensorHigro2, INPUT);
  pinMode(pinoSensorLDR, INPUT);

  if (!rtc.begin()) { // SE O RTC NÃO FOR INICIALIZADO, FAZ
    
    lcd.clear();
    lcd.setCursor(0,0); // Coloca o cursor do display na coluna 0 e linha 0
    lcd.print("Falha DS1307!"); // Comando de saída com a mensagem que deve aparecer na coluna 2 e linha 1.
    while(1); //SEMPRE ENTRE NO LOOP
  }
  if (rtc.isrunning()) { //SE RTC NÃO ESTIVER SENDO EXECUTADO, FAZ
        
    lcd.clear();
    lcd.setCursor(0,0); // Coloca o cursor do display na coluna 0 e linha 0
    lcd.print("DS1307 rodando!"); // Comando de saída com a mensagem que deve aparecer na coluna 2 e linha 1.
    delay(5000);
    lcd.clear();

    //REMOVA O COMENTÁRIO DE UMA DAS LINHAS ABAIXO PARA INSERIR AS INFORMAÇÕES ATUALIZADAS EM SEU RTC
    //rtc.adjust(DateTime(F(_DATE), F(TIME_))); //CAPTURA A DATA E HORA EM QUE O SKETCH É COMPILADO
    //rtc.adjust(DateTime(2023, 2, 15, 21, 54, 00)); //(ANO), (MÊS), (DIA), (HORA), (MINUTOS), (SEGUNDOS)
  }else {
    lcd.clear();
    lcd.setCursor(0,0); // Coloca o cursor do display na coluna 0 e linha 0
    lcd.print("Falha RTC!"); 
    while(1); //SEMPRE ENTRE NO LOOP
  }

  lcd.clear();
}
//--------------------------------------------------------------

//--------------------------------------------------------------
void imprime_data_hora (){
  DateTime now = rtc.now(); //CHAMADA DE FUNÇÃO
  
  lcd.setCursor(0, 1); //Coloca o cursor do display na coluna 1 e linha 2
  lcd.print(daysOfTheWeek[now.dayOfTheWeek()]);  // Comando de saida com a mensagem que deve aparecer na coluna 2 e linha 2
  lcd.print(" ");
  if ((now.hour()) <10) {
    lcd.print("0");
  }
  lcd.print( now.hour()); //CHAMADA DE FUNÇÃO, DEC);
  lcd.print(":");

  if ((now.minute()) <10) {
    lcd.print("0");
  }
  lcd.print(now.minute());
  lcd.print(":");

  if ((now.second()) <10) {
    lcd.print("0");
  }
  lcd.print( now.second(), DEC);
}
//--------------------------------------------------------------

//--------------------------------------------------------------
void control_lampadas () {
  //verificar hora e minuto e liga lampada
  DateTime now = rtc.now(); //CHAMADA DE FUNÇÃO
  int hora_inicio = 6, minuto_inicio = 30, hora_final = 15, minuto_final = 30;
  //rotina liga lampada

  Serial.print("now.hour():");
  Serial.println(now.hour());

  Serial.print("now.minute():");
  Serial.println(now.minute());
  
  if ( (now.hour() > hora_inicio || ((now.hour() == hora_inicio && now.minute() >= minuto_inicio))) && (now.hour() < hora_final || (now.hour() == hora_final && now.minute() <= minuto_final))){
    sinal_lamp_01 = HIGH;
  } else{
    sinal_lamp_01 = LOW;
  }
  
  if (sinal_lamp_01 == HIGH) {
    //Serial.println("Relé Lamp 1: LIGADO");
    digitalWrite(Relay_Lamp_1, HIGH);
  } else {
    //Serial.println("Relé Lamp 1: DESLIGADO");
    digitalWrite(Relay_Lamp_1, LOW);
  }

  sinal_lamp_02 = LOW;
  if (sinal_lamp_02 == HIGH) {
    //Serial.println("Relé Lamp 2: LIGADO");
    digitalWrite(Relay_Lamp_2, HIGH);
  } else {
    //Serial.println("Relé Lamp 2: DESLIGADO");
    digitalWrite(Relay_Lamp_2, LOW);
  }

  //NÃO ESTÁ SENDO FEITO O CONTROLE DA SAIDA LAMP2
}
//--------------------------------------------------------------

//--------------------------------------------------------------
//FUNÇÃO PARA CONTROLAR UMIDADE DO SOLO
void leitura_umid_solo () {
  long aux1 = 0;
  long aux2 = 0;
  valor_umid_solo_01 = 0;
  valor_umid_solo_02 = 0;
  
  digitalWrite(Relay_Higro, HIGH);
  delay (200);
    Serial.print ("ANOLG solo 01: ");
  Serial.println (analogRead(pinoSensorHigro1));
  Serial.print ("ANOLG solo 02: ");
  Serial.println (analogRead(pinoSensorHigro2));

  for (int i = 0; i < 1000; i++) {
    aux1 = map(analogRead(pinoSensorHigro1), analogSoloMolhado, analogSoloSeco, percSoloSeco, percSoloMolhado);
    valor_umid_solo_01 = valor_umid_solo_01 + aux1;

    aux2 = map(analogRead(pinoSensorHigro2), analogSoloMolhado, analogSoloSeco, percSoloSeco, percSoloMolhado);
    valor_umid_solo_02 = valor_umid_solo_02 + aux2;
  }
    Serial.print ("DEPOIS ANOLG solo 01: ");
  Serial.println (analogRead(pinoSensorHigro1));
  Serial.print ("DEPOIS ANOLG solo 02: ");
  Serial.println (analogRead(pinoSensorHigro2));

  digitalWrite(Relay_Higro, LOW);

  Serial.print ("ANOLG solo 01: ");
  Serial.println (analogRead(pinoSensorHigro1));
  Serial.print ("ANOLG solo 02: ");
  Serial.println (analogRead(pinoSensorHigro2));

  valor_umid_solo_01 = valor_umid_solo_01 / 1000;
  valor_umid_solo_02 = valor_umid_solo_02 / 1000;
  Serial.print ("media solo 01: ");
  Serial.println (valor_umid_solo_01);
  Serial.print ("media solo 02: ");
  Serial.println (valor_umid_solo_02);
  
  if (valor_umid_solo_01 <= percMinSoloSeco) {
    terraSeca_01 = true;
  } 
  if (valor_umid_solo_01 >= percMaxSoloSeco) {
    terraSeca_01 = false;
  }
}

void control_umid_solo (bool sinal_bomb1, bool sinal_bomb2) {

  if (sinal_bomb1 || sinal_bomb2) {
    Serial.println("Leitura durante o acionamento da bomba");
    leitura_umid_solo ();
  } else {
      
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis_02 >= interval_02) {
      Serial.println("Leitura Periodica");
      leitura_umid_solo ();
      previousMillis_02 = currentMillis;
    }
  }  
}
//--------------------------------------------------------------

//--------------------------------------------------------------
//FUNÇÃO PARA CONTROLAR ACIONAMENTO DAS BOMBAS
void control_bombas () {
 DateTime now = rtc.now(); // Obtém a hora atual do RTC

  // Verifica se o horário de início é menor do que o horário de fim
  if (HOUR_START_01 > HOUR_END_01 || (HOUR_START_01 == HOUR_END_01 && MINUTE_START_01 >= MINUTE_END_01) ||
      HOUR_START_02 > HOUR_END_02 || (HOUR_START_02 == HOUR_END_02 && MINUTE_START_02 >= MINUTE_END_02)) {
    Serial.println("Erro: Horário de início maior ou igual ao horário de fim.");
    return;
  }

  // Verifica se o horário atual está dentro do intervalo correto
  bool inInterval = false;
  if ((now.hour() > HOUR_START_01 || (now.hour() == HOUR_START_01 && now.minute() >= MINUTE_START_01)) &&
      (now.hour() < HOUR_END_01 || (now.hour() == HOUR_END_01 && now.minute() <= MINUTE_END_01))) {
    inInterval = true;
  }
  else if ((now.hour() > HOUR_START_02 || (now.hour() == HOUR_START_02 && now.minute() >= MINUTE_START_02)) &&
           (now.hour() < HOUR_END_02 || (now.hour() == HOUR_END_02 && now.minute() <= MINUTE_END_02))) {
    inInterval = true;
  }

  // Define os sinais das bombas
  if (inInterval && terraSeca_01) {
    sinal_bomb_01 = true;
  }
  if (inInterval && terraSeca_02) {
    sinal_bomb_02 = true;
  }

  // Atualiza os relés das bombas
    digitalWrite(Relay_Bomb_Pin[0], sinal_bomb_01);
    digitalWrite(Relay_Bomb_Pin[1], sinal_bomb_02);
}
//--------------------------------------------------------------

//--------------------------------------------------------------
//FUNÇÕES PARA EXIBIÇÃO DE MENSAGENS DO SISTEMA
void update_messages() {
  if (sinal_lamp_01 == HIGH) {
    messages[MESSAGE_LAMP_01] = "Lamp01 ON";
  } else {
    messages[MESSAGE_LAMP_01] = "Lamp01 OFF";
  };

  if (sinal_lamp_02 == HIGH) {
    messages[MESSAGE_LAMP_02] = "Lamp02 ON";
  } else {
    messages[MESSAGE_LAMP_02] = "Lamp02 OFF";
  }

  if (sinal_bomb_01 == HIGH) {
    messages[MESSAGE_BOMB_01] = "Bomb01 ON";
  } else {
    messages[MESSAGE_BOMB_01] = "Bomb01 OFF";
  }

  if (sinal_bomb_02 == HIGH) {
    messages[MESSAGE_BOMB_02] = "Bomb02 ON";
  } else {
    messages[MESSAGE_BOMB_02] = "Bomb02 OFF";
  }

  if (terraSeca_01) {
    messages[MESSAGE_TERRA_SECA_01] = "Terra01 SECA";
  } else {
    messages[MESSAGE_TERRA_SECA_01] = "Terra01 UMID";
  }

  if (terraSeca_02) {
    messages[MESSAGE_TERRA_SECA_02] = "Terra02 SECA";
  } else {
    messages[MESSAGE_TERRA_SECA_02] = "Terra02 UMID";
  }
  
}

void print_messages() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(messages[messageIndex]);

  Serial.println("-----------------------------------");
  Serial.println("Mensagem do Sistema:");

  for (int i = 0; i < NUM_MESSAGES; i++) {
    Serial.println(messages[i]);
  }

  Serial.println("-----------------------------------");

  messageIndex++;
  if (messageIndex >= NUM_MESSAGES) {
    messageIndex = 0;
  }
}

void mensagens_sistema() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis_01 >= interval_01) {
    update_messages();
    print_messages();
    previousMillis_01 = currentMillis;
  }
  imprime_data_hora();
}
//--------------------------------------------------------------

void loop() {
  //--------------------------------------------------------------
  //CONTROLE UMIDADE DO SOLO
    control_umid_solo (sinal_bomb_01, sinal_bomb_02);
  //--------------------------------------------------------------
  
  control_lampadas ();
  control_bombas ();
  mensagens_sistema();
  
  delay(1000); 
}
