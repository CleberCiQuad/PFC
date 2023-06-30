/* 11/04 - modifiquei a ordem dos sensores
 *  identifiquei pinos SPI para possível SD
 * reunião dia 24
 * Sensores analógicos
 * velocidade
 * peso das lampadas
 * testes
 * 
 * 
 *   The following pins are usable for PinChangeInterrupt:

  Arduino Leonardo/Micro: 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI)
 * 
 */


#include<ADS1115_WE.h> 
#include<Wire.h>
#include "PinChangeInterrupt.h"

#define I2C_ADDRESS_1  0x48
#define I2C_ADDRESS_2  0x49

#define Interruot_Direita 8  
#define Interrupt_Esquerda 7

ADS1115_WE adc_1 = ADS1115_WE(I2C_ADDRESS_1);
ADS1115_WE adc_2 = ADS1115_WE(I2C_ADDRESS_2);

//MOTOR 1
int Motores_EN = 4;  // pino 4 habilita todos os enables
int R_PWM1 = 5;
int L_PWM1 = 6;
//MOTOR 2
int R_PWM2 = 9;
int L_PWM2 = 10;

//LAMPADAS
int L1 = 14;       // MISO
int L2 = A0;
int L3 = A1;
int L4 = A2;

//INTERRUPÇÃO ENCODER
unsigned long IntTime = 0;
float Ref_time;
unsigned long timeold;
unsigned long timeold2;
float tempo_inicio;
float tempo_fim;
int flag_start = 0;
int flag_teste = 1;
int flag_fim = 0;

int rpm1;
int rpm2;

//CONTADORES DE PULSOS DAS INTERRUPÇÕES

volatile byte pulsosA1;
volatile byte pulsosA2;

volatile byte pulsosB1;
volatile byte pulsosB2;

//VARIAVEIS CONFIGURAVEIS///////////////////////////////// 

unsigned Divisoes_Encoder1 = 120;
unsigned Divisoes_Encoder2 = 120;

float Distancia_teste = 5;//metros

int Velocidade = 150;//0 a 255
int Corte = 50;
//---------------VARIÁVEIS DE NORMALIZAÇÃO DAS LEITURAS -------------------//


float ERRO = 0;
float SOMA = 0;
float Ultimo_ERRO;

int Status = 0;
int Ult_Status = 0;


int S1_Min = 200;
int S1_Max = 0;
int S2_Min = 200;
int S2_Max = 0;
int S3_Min = 200;
int S3_Max = 0;
int S4_Min = 200;
int S4_Max = 0;
int S5_Min = 200;
int S5_Max = 0;
int S6_Min = 200;
int S6_Max = 0;
int S7_Min = 200;
int S7_Max = 0;
int S8_Min = 200;
int S8_Max = 0;

// - Inicialmente vou deixar com esses valores fixos, mas posteriormente sera realizado uma calibração antes da utilização
//-----------------------------PID--------------------------------//

int PWM_Speed = 150; //Velocidade máxima do PID

//Configuração de variaveis  KP, KI e KD  
//                 funcionou na reta        para curvas   novos 
float Kp = 3;//10 //        2                     6         1.5
float Ki = 0.167;   //       0.5                   0.5          0.5
float Kd = 0.042;//100//      0.5                    10           2

int error, last_error; //--> Armazena valor de erro dos sensores

     int right_Forward = 0;
   
     int right_Backward = 0;

     int left_Forward = 0;
  
     int left_Backward = 0;
//---------------------------------------------------------------//   
int Aux_giro = 0;
int Passada = 0;
int Read_flag = 0;


int Page = 0; //  Auxiliar para Display Nextion

int IR = 15;// aciona sensores infravermelhos  - SCK
//sinais dos sensores ja tratados
int S1;
int S2;
int S3;
int S4;
int S5;
int S6;
int S7;
int S8;

int NovoS1;
int NovoS2;
int NovoS3;
int NovoS4;
int NovoS5;
int NovoS6;
int NovoS7;
int NovoS8;

int regua = 0;

//Variaveis de armazenamento e manipulação dos sensores
int Sensor_ADC_0[8];


int MaisDireito;
int MaisEsquerdo;

//BOTOES DAS LAMPADAS
int EstadoL1 = 0;
int EstadoL2 = 0;
int EstadoL3 = 0;
int EstadoL4 = 0;

int EstadoMov = 0;


void setup(void) 
{
    Wire.begin();
    Serial.begin(115200);
    Serial1.begin(115200);
    
     pinMode(IR, OUTPUT);//leds IR como saída

     //MOTORES
     pinMode(Motores_EN, OUTPUT);
     //MOTOR 1
     pinMode(R_PWM1, OUTPUT);
     pinMode(L_PWM1, OUTPUT);
     //MOTOR 2
     pinMode(R_PWM2, OUTPUT);
     pinMode(L_PWM2, OUTPUT);
     
     //LAMPADAS
     pinMode(L1, OUTPUT);
     pinMode(L2, OUTPUT);
     pinMode(L3, OUTPUT);
     pinMode(L4, OUTPUT);
     
     //COLOCAR ESSA PARTE NO CÓDIGO
     digitalWrite(Motores_EN, HIGH);

     //inicializa com sensores desligados
     digitalWrite(15, HIGH); 
     
    //CONFIGURAÇÃO DOS ADC's
    if(!adc_1.init()){
      Serial.print("ADS1115 No 1 not connected!");
    }
    adc_1.setVoltageRange_mV(ADS1115_RANGE_6144);
    adc_1.setMeasureMode(ADS1115_CONTINUOUS); 
    adc_1.setCompareChannels(ADS1115_COMP_0_GND);
    adc_1.setConvRate(ADS1115_860_SPS);
    
    if(!adc_2.init()){
      Serial.print("ADS1115 No 2 not connected!");
    }
    adc_2.setVoltageRange_mV(ADS1115_RANGE_6144);
    adc_2.setMeasureMode(ADS1115_CONTINUOUS); 
    adc_2.setCompareChannels(ADS1115_COMP_0_GND);
    adc_2.setConvRate(ADS1115_860_SPS);
    
   /* Set the conversion rate in SPS (samples per second)
   * Options should be self-explaining: 
   * 
   *  ADS1115_8_SPS 
   *  ADS1115_16_SPS  
   *  ADS1115_32_SPS 
   *  ADS1115_64_SPS  
   *  ADS1115_128_SPS (default) - 7 amostras no programa
   *  ADS1115_250_SPS - 13 amostras no programa
   *  ADS1115_475_SPS - 16 amostras no programa
   *  ADS1115_860_SPS - 20 amostras no programa
   */
    

    rpm1 = 0;
    rpm2 = 0;
    pulsosA1 = 0;
    pulsosB1 = 0;
    timeold = 0;

    //INTERRUPÇÃO
    pinMode(Interruot_Direita, INPUT_PULLUP);
    pinMode(Interrupt_Esquerda, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Interrupt_Esquerda), Interrupt_B, RISING);
    attachPCINT(digitalPinToPCINT(Interruot_Direita), Interrupt_A, RISING);//Arduino Leonardo/Micro: 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI)

}

void loop(void) 
{
  
  
//----------PAGE ZERO---------------------//
  if(Serial1.available()>0)// Verifica se tem algo na serial
  {
  int Recived = Serial1.read(); // armazena valor da serial na variavel 'Recived'
  Serial.println(Recived);      // Imprime a variavel
  
        if(Recived == 1){
          Serial.println(Recived);
             Page = 1;  // PAGINA DOS SENSORES
        }
        if(Recived == 2){
          Serial.println(Recived);
             Page = 2;  // PAGINA DOS MOTORES
        }
        if(Recived == 3){
          Serial.println(Recived);
             Page = 3;  // PAGINA DO MOVIMENTO E LUZES
        }       
  }
//--------------PAGE 1-------------------//
while(Page == 1){ //sensores
          digitalWrite(15, LOW); // liga leds infravermelho
          //Serial.println(Page);
          LerSensores();
} 
//--------------PAGE 2-------------------//
while(Page == 2){ //motores
          //Serial.println(Page);
          
          if(Serial1.available()>0)// Verifica se tem algo na serial
                  {
                  int Recived = Serial1.read(); // armazena valor da serial na variavel 'Recived'
                  Serial.println(Recived);      // Imprime a variavel
                  // TESTE DOS MOTORES
                  if(Recived == 1){
                    Serial.println(Recived);
                       analogWrite(R_PWM1, Velocidade);
                       delay(3000);
                       analogWrite(R_PWM1, 0);
                  }
                  if(Recived == 2){
                    Serial.println(Recived);
                       analogWrite(L_PWM1, Velocidade);
                       delay(3000);
                       analogWrite(L_PWM1, 0);
                  }
                  if(Recived == 3){
                    Serial.println(Recived);
                       analogWrite(R_PWM2, Velocidade);
                       delay(3000);
                       analogWrite(R_PWM2, 0);
                  }
                  if(Recived == 4){
                    Serial.println(Recived);
                       analogWrite(L_PWM2, Velocidade);
                       delay(3000);
                       analogWrite(L_PWM2, 0);
                  }
                  // VELOCIDADE ---- TIRAR UM BOTÃO DO DISPLAY
                  if(Recived == 5){
                    Serial.println(Recived);
                      int Kp = 5;//10
                      int Ki = 1;
                      int Kd = 50;//100
                      Serial.println(Velocidade);
                  }
                  if(Recived == 7){
                    Serial.println(Recived);
                      int Kp = 20;//10
                      int Ki = 4;
                      int Kd = 200;//100
                      Serial.println(Velocidade);
                  }
                  if(Recived == 8){
                    Serial.println(Recived);
                      int Kp = 0.6;//10
                      int Ki = 0.3;
                      int Kd = 50;//100
                      Serial.println(Velocidade);
                  }
                  if(Recived == 9){
                    Serial.println(Recived);
                      int Kp = 0.3;//10
                      int Ki = 0.6;
                      int Kd = 0.9;//100
                      Serial.println(Velocidade);
                  }
                  //VOLTAR
                  if(Recived == 6){
                       Page = 0;
                  }
          }
                   

} 
//--------------PAGE 3-------------------//
while(Page == 3){
          //Serial.println(Page);
            if(Serial1.available()>0)// Verifica se tem algo na serial
                    {
                    int Recived = Serial1.read(); // armazena valor da serial na variavel 'Recived'
                    Serial.println(Recived);      // Imprime a variavel

                          if(Recived == 7){// fazer um while no movimento
                               Page = 4;
                               //Movimento();
                          }
                          if(Recived == 1){//luz 1
                                  EstadoL1 = !EstadoL1;  // inverte o estado da lampada 1
                                  Serial.println(EstadoL1);
                                  digitalWrite(L1, EstadoL1);
                          }
                          if(Recived == 2){//luz 2
                                  EstadoL2 = !EstadoL2;  // inverte o estado da lampada 2
                                  Serial.println(EstadoL2);
                                  digitalWrite(L2, EstadoL2);
                          }
                          if(Recived == 3){//luz 3
                                  EstadoL3 = !EstadoL3;  // inverte o estado da lampada 3
                                  Serial.println(EstadoL3);
                                  digitalWrite(L3, EstadoL3);
                          }
                          if(Recived == 4){//luz 4
                                  EstadoL4 = !EstadoL4;  // inverte o estado da lampada 4
                                  Serial.println(EstadoL4);
                                  digitalWrite(L4, EstadoL4);
                          }
                    
                          if(Recived == 6){//VOLTAR
                            Serial.println(Recived);
                               Page = 0;
                          }
                    }
          
} 
while(Page == 4){ //Ligar
          digitalWrite(15, LOW);
          //Serial.println(Page);
          Movimento_simples();
      
} 
//---------------------------------------//
 
//----------------FIM DO MENU-----------------//
}
void LerSensores(){ 
//  TRECHO RESPONSÁVEL POR OUVIR A SERIAL CONECTADA AO DISPLAY E RECEBER COMANDOS   //////////////////
                  if(Serial1.available()>0)// Verifica se tem algo na serial
                  {
                  int Recived = Serial1.read(); // armazena valor da serial na variavel 'Recived'
                  Serial.println(Recived);      // Imprime a variavel
                  
                        if(Recived == 6){
                             digitalWrite(15, HIGH);   // turn the LED on (HIGH is the voltage level)
                                      analogWrite(R_PWM1, 0);
                                       analogWrite(L_PWM1, 0);
                                  //left_Forward
                                       analogWrite(L_PWM2, 0);
                                       analogWrite(R_PWM2, 0);
                             Page = 0;
                        }
                  }
///////////////////////////////////////////////////////////////////////////////////////////////////////
                  
              
                  if(millis() - timeold >= 50){
                  Calcula_RPM();
                   
                  Read_Sensor(); // LE OS SENSORES NO AD
                  
                  Maximos_E_Minimos(); //Salva os valores máximos e mínimos lidos em cada sensore
                 
                  
                  
                        //AD1
                        //Entrada 0 - sensor4 do Nextion          
                        Serial.print(NovoS4);
                        Serial.print(";"); 
                        Serial1.print("sensor1.val=");
                        Serial1.print(NovoS4);
                        Serial1.write(0xff);
                        Serial1.write(0xff);
                        Serial1.write(0xff);
                        
                        //Entrada 1 - sensor3 do Nextion  
                        Serial.print(NovoS3);
                        Serial.print(";");
                        Serial1.print("sensor2.val=");
                        Serial1.print(NovoS3);
                        Serial1.write(0xff);
                        Serial1.write(0xff);
                        Serial1.write(0xff);
                        
                        //Entrada 2 - sensor2 do Nextion       
                        Serial.print(NovoS2);
                        Serial.print(";");
                        Serial1.print("sensor3.val=");
                        Serial1.print(NovoS2);
                        Serial1.write(0xff);
                        Serial1.write(0xff);
                        Serial1.write(0xff);
                 
                        //Entrada 3 - sensor1 do Nextion                       
                        Serial.print(NovoS1);
                        Serial.print(";");
                        Serial1.print("sensor4.val=");
                        Serial1.print(NovoS1);
                        Serial1.write(0xff);
                        Serial1.write(0xff);
                        Serial1.write(0xff);

                        //AD2
                        //Entrada 0 - sensor5 do Nextion          
                        Serial.print(NovoS8);
                        Serial.print(";");
                        Serial1.print("sensor5.val=");
                        Serial1.print(NovoS8);
                        Serial1.write(0xff);
                        Serial1.write(0xff);
                        Serial1.write(0xff);

                        //Entrada 1 - sensor6 do Nextion  
                        Serial.print(NovoS7);
                        Serial.print(";");
                        Serial1.print("sensor6.val=");
                        Serial1.print(NovoS7);
                        Serial1.write(0xff);
                        Serial1.write(0xff);
                        Serial1.write(0xff);

                        //Entrada 2 - sensor7 do Nextion       
                        Serial.print(NovoS6);
                        Serial.print(";");
                        Serial1.print("sensor7.val=");
                        Serial1.print(NovoS6);
                        Serial1.write(0xff);
                        Serial1.write(0xff);
                        Serial1.write(0xff);
                 
                       //Entrada 3 - sensor8 do Nextion
                        Serial.print(NovoS5);
                        Serial.println(";");
                        Serial1.print("sensor8.val=");
                        Serial1.print(NovoS5);
                        Serial1.write(0xff);
                        Serial1.write(0xff);
                        Serial1.write(0xff);
Serial.println("ERRO");
Serial.println(ERRO);
                  }
         
}

//Funções para ler os sensores nos ADs
float readChannel_1(ADS1115_MUX channel) {
          float voltage = 0.0;
          adc_1.setCompareChannels(channel);
          voltage = adc_1.getResult_V(); // alternative: getResult_mV for Millivolt
          return voltage;
}
float readChannel_2(ADS1115_MUX channel) {
          float voltage = 0.0;
          adc_2.setCompareChannels(channel);
          voltage = adc_2.getResult_V(); // alternative: getResult_mV for Millivolt
          return voltage;
}

void Movimento_simples(){//Laço principal do movimento do robô


  if(Serial1.available()>0)// Verifica se tem algo na serial
  {
  int Recived = Serial1.read(); // armazena valor da serial na variavel 'Recived'
  Serial.println(Recived);      // Imprime a variavel
  
        if(Recived == 6){
             delay(1000);
             Page = 3;
             digitalWrite(15, HIGH);   // turn the LED on (HIGH is the voltage level)
             digitalWrite(L1, LOW);
             digitalWrite(L2, LOW);
             digitalWrite(L3, LOW);
             digitalWrite(L4, LOW);
             analogWrite(R_PWM1, 0);
             analogWrite(L_PWM1, 0);
             analogWrite(R_PWM2, 0);
             analogWrite(L_PWM2, 0);
             Page = 3;
        }

 }else{
    if(flag_teste == 1){

      if(millis() - timeold >= 50){//Estabelece um periodo de amostragem e correção de 50ms
                  Read_Sensor();                    
                  Serial.println(ERRO);  
                  //Escolha do controlador
                  //on_off_Back();
                  //on_off_Flow();
                  Ajusta_PID();
      }
    }
    if((NovoS4 > Corte) && (NovoS5 > Corte) && (flag_start == 0)){
    tempo_inicio = millis();
    flag_start = 1;
    Serial.println("INICIO DO TESTE! ");

    }
    Ref_time = millis() - tempo_inicio;
        if((NovoS4 > Corte) && (NovoS5 > Corte) && (flag_start == 1) && (Ref_time > 3000)&& (flag_fim == 0)){
    tempo_fim = Ref_time;
    Serial.println("FIM DO TESTE: ");
         flag_teste = 0;
         flag_fim = 1;
         analogWrite(R_PWM1, 0);
         analogWrite(L_PWM1, 0);
         analogWrite(R_PWM2, 0);
         analogWrite(L_PWM2, 0);

         float Velocite = Distancia_teste/(tempo_fim/1000);
         
         Serial.print("Tempo total do teste: ");
         Serial.print(tempo_fim/1000);
         Serial.print("s   Veleocidade calculadda: ");
         Serial.print(Velocite);
         Serial.print("m/s    valor do pwm: ");
         Serial.println(Velocidade);
 
    }


     }
}
void on_off_Flow(){ //Controlador on/off fluído
      if((NovoS1 > Corte) && (NovoS8 > Corte)){
         analogWrite(R_PWM1, Velocidade-(Velocidade*0.05));
         analogWrite(L_PWM1, 0);
         analogWrite(R_PWM2, Velocidade);
         analogWrite(L_PWM2, 0);
         Ult_Status = 0;
    }
        else if((NovoS8 < Corte && NovoS1 > Corte)){
         analogWrite(R_PWM1, 0);
         analogWrite(L_PWM1, 0);
         analogWrite(R_PWM2, Velocidade);
         analogWrite(L_PWM2, 0);
         Ult_Status = 2;
    }
    else if(((NovoS8 > Corte) && (NovoS1 < Corte))){
         analogWrite(R_PWM1, Velocidade-(Velocidade*0.05));
         analogWrite(L_PWM1, 0);
         analogWrite(R_PWM2, 0);
         analogWrite(L_PWM2, 0);   
         Ult_Status = 1;
    }

else if(NovoS8 < Corte && NovoS1 < Corte){
   // Mantém o último status registrado
   if(Ult_Status == 1){
      analogWrite(R_PWM1, Velocidade-(Velocidade*0.05));
      analogWrite(L_PWM1, 0);
      analogWrite(R_PWM2, 0);
      analogWrite(L_PWM2, 0);
   }
   else if(Ult_Status == 2){
      analogWrite(R_PWM1, 0);
      analogWrite(L_PWM1, 0);
      analogWrite(R_PWM2, Velocidade);
      analogWrite(L_PWM2, 0);
   }
}

}
void on_off_Back(){//Controlador on/off com reversão
      if((NovoS1 > Corte) && (NovoS8 > Corte)){
         analogWrite(R_PWM1, Velocidade-(Velocidade*0.05));
         analogWrite(L_PWM1, 0);
         analogWrite(R_PWM2, Velocidade);
         analogWrite(L_PWM2, 0);
         Ult_Status = 0;
    }
    else if((NovoS8 > Corte) && (NovoS1 < Corte)){
         analogWrite(R_PWM1, Velocidade-(Velocidade*0.05));
         analogWrite(L_PWM1, 0);
         analogWrite(R_PWM2, 0);
         analogWrite(L_PWM2, Velocidade);   
         Ult_Status = 1;
    }
    else if(NovoS8 < Corte && NovoS1 > Corte){
         analogWrite(R_PWM1, 0);
         analogWrite(L_PWM1, Velocidade-(Velocidade*0.05));
         analogWrite(R_PWM2, Velocidade);
         analogWrite(L_PWM2, 0);
         Ult_Status = 2;
    }
else if(NovoS8 < Corte && NovoS1 < Corte){
   // Mantém o último status registrado
   if(Ult_Status == 1){
         analogWrite(R_PWM1, Velocidade-(Velocidade*0.05));
         analogWrite(L_PWM1, 0);
         analogWrite(R_PWM2, 0);
         analogWrite(L_PWM2, Velocidade); 
   }
   else if(Ult_Status == 2){
         analogWrite(R_PWM1, 0);
         analogWrite(L_PWM1, Velocidade-(Velocidade*0.05));
         analogWrite(R_PWM2, Velocidade);
         analogWrite(L_PWM2, 0);
   }
}
}
void Ajusta_PID(){
    int PWM_Max_R = PWM_Speed-(PWM_Speed*0.05);//Compensação da velocidade do motor direito
    int PWM_Max_L = PWM_Speed;

    //Calculos do PID
    float P = ERRO;
    float I = I + ERRO;
    float D = ERRO - last_error;
    float PID = (Kp * P) + (Ki * I) + (Kd * D); 
    //........................................

    //Aplica resultado do PID nos motores.
    float PWM_R = PWM_Max_R - PID;
    float PWM_L = PWM_Max_L + PID;
    PWM_R = PWM_R - PID;
    PWM_L = PWM_L + PID; 

    last_error = ERRO;

    //Habilita um dos sentidos para cada motor
    if (PWM_R >= 0) {
      right_Backward = 0;
      right_Forward = 1;
    } else {
      right_Forward = 0;
      right_Backward = 1;
    }

    if (PWM_L >= 0) {
      left_Backward = 0;
      left_Forward = 1;
    } else {
      left_Forward = 0;
      left_Backward = 1;
    }

    if (PWM_R < 0) PWM_R = 0 - PWM_R;
    if (PWM_L < 0) PWM_L = 0 - PWM_L;
    if (PWM_R > 255) PWM_R = 255;
    if (PWM_L > 255) PWM_L = 255;

    if (right_Forward == 1) {
      PWM_R = abs(PWM_R);
    } else {
      PWM_R = 255 - PWM_R;
    }

    if (left_Forward == 1) {
      PWM_L = abs(PWM_L);
    } else {
      PWM_L = 255 - PWM_L;
    }

    //Aplica os valores calculados aos motores
    if(right_Backward == 0 && right_Forward == 1){
         analogWrite(L_PWM1, 0);
         analogWrite(R_PWM1, PWM_R);
    }    
    if(right_Backward == 1 && right_Forward == 0){
         analogWrite(R_PWM1, 0);
         analogWrite(L_PWM1, PWM_R);
    }

    if(left_Backward == 0 && left_Forward == 1){
         analogWrite(L_PWM2, 0);
         analogWrite(R_PWM2, PWM_L);
    }    
    if(left_Backward == 1 && left_Forward == 0){
         analogWrite(R_PWM2, 0);
         analogWrite(L_PWM2, PWM_L);
    }
}

void Interrupt_A(void) {
  pulsosA1++;
}
void Interrupt_B(void){
  pulsosB1++;
}
void Read_Sensor(){//Aqui é realizada leitura dos sinais bem como o calculo do erro
 
                   float voltage = 0.0;
                   int err0 = 0;
                 //AD1
                 //Entrada 3 - sensor1 do Nextion  // X X X O X X X X
                        voltage = readChannel_1(ADS1115_COMP_0_GND);
                        S1 = voltage*100;
                        NovoS1 = map(S1, S1_Min, S1_Max, 0, 100);
                        if (NovoS1<25){
                          NovoS1 = 0;
                        }
                        err0 = NovoS1*(7);

                 //Entrada 2 - sensor2 do Nextion  // X X O X X X X X     
                        voltage = readChannel_1(ADS1115_COMP_1_GND);
                        S2 = voltage*100;
                        NovoS2 = map(S2, S2_Min, S2_Max, 0, 100);
                        if (NovoS2<25){
                          NovoS2 = 0;
                        }
                        err0 = err0 + NovoS2*(21);

                 //Entrada 1 - sensor3 do Nextion  // X O X X X X X X
                        voltage = readChannel_1(ADS1115_COMP_2_GND);
                        S3 = voltage*100;
                        NovoS3 = map(S3, S3_Min, S3_Max, 0, 100);
                        if (NovoS3<25){
                          NovoS3 = 0;
                        }
                        err0 = err0 + NovoS3*(35);

                 //Entrada 0 - sensor4 do Nextion  // O X X X X X X X        
                        voltage = readChannel_1(ADS1115_COMP_3_GND);
                        S4 = voltage*100;
                        NovoS4 = map(S4, S4_Min, S4_Max, 0, 100);
                        if (NovoS4<25){
                          NovoS4 = 0;
                        }
                        err0 = err0 + NovoS4*(49);
                  
                 //AD2
                 //Entrada 3 - sensor5 do Nextion  // X X X X X X X O  
                        voltage = readChannel_2(ADS1115_COMP_0_GND);
                        S5 = voltage*100;
                        NovoS5 = map(S5, S5_Min, S5_Max, 0, 100);
                        if (NovoS5<25){
                          NovoS5 = 0;
                        }
                        err0 = err0 + NovoS5*(-49);

                 //Entrada 2 - sensor6 do Nextion  // X X X X X X O X    
                        voltage = readChannel_2(ADS1115_COMP_1_GND);
                        S6 = voltage*100;
                        NovoS6 = map(S6, S6_Min, S6_Max, 0, 100);
                        if (NovoS6<25){
                          NovoS6 = 0;
                        }
                        err0 = err0 + NovoS6*(-35);

                 //Entrada 1 - sensor7 do Nextion  // X X X X X O X X
                        voltage = readChannel_2(ADS1115_COMP_2_GND);
                        S7 = voltage*100;
                        NovoS7 = map(S7, S7_Min, S7_Max, 0, 100);
                        if (NovoS7<25){
                          NovoS7 = 0;
                        }
                        err0 = err0 + NovoS7*(-21);

                 //Entrada 0 - sensor4 do Nextion  // X X X X O X X X       
                        voltage = readChannel_2(ADS1115_COMP_3_GND);
                        S8 = voltage*100;
                        NovoS8 = map(S8, S8_Min, S8_Max, 0, 100);
                        if (NovoS8<25){
                          NovoS8 = 0;
                        }
                        err0 = err0 + NovoS8*(-7);

                      
                       SOMA = (NovoS1+NovoS2+NovoS3+NovoS4+NovoS5+NovoS6+NovoS7+NovoS8);
                       ERRO = err0/SOMA;

                       //Mantem erro máximo quando a linha não está sob a placa
                       if(SOMA < 50){
                        ERRO = Ultimo_ERRO;
                       }else{
                        Ultimo_ERRO = ERRO;
                       }                       
}

void Calcula_RPM(){
                    pulsosA2 = pulsosA1;
                    pulsosA1 = 0;
                    pulsosB2 = pulsosB1;
                    pulsosB1 = 0;
                    rpm1 = (60 * 1000 / Divisoes_Encoder1) / (millis() - timeold) * pulsosA2;
                    rpm2 = (60 * 1000 / Divisoes_Encoder2) / (millis() - timeold) * pulsosB2;// pino 7
                    timeold = millis();
                    pulsosA2 = 0;
                    pulsosB2 = 0;

}
void Maximos_E_Minimos(){//Função para calibrar cada sensor com leituras maximas e mínimas
  //MINIMOS
                 if(S1 < S1_Min){
                  S1_Min = S1;
                 }
                 if(S2 < S2_Min){
                  S2_Min = S2;
                 }
                 if(S3 < S3_Min){
                  S3_Min = S3;
                 }
                 if(S4 < S4_Min){
                  S4_Min = S4;
                 }
                 if(S5 < S5_Min){
                  S5_Min = S5;
                 }
                 if(S6 < S6_Min){
                  S6_Min = S6;
                 }
                 if(S7 < S7_Min){
                  S7_Min = S7;
                 }
                 if(S8 < S8_Min){
                  S8_Min = S8;
                 }
//MAXIMOS
                 if(S1 > S1_Max){
                  S1_Max = S1;
                 }
                 if(S2 > S2_Max){
                  S2_Max = S2;
                 }
                 if(S3 > S3_Max){
                  S3_Max = S3;
                 }
                 if(S4 > S4_Max){
                  S4_Max = S4;
                 }
                 if(S5 > S5_Max){
                  S5_Max = S5;
                 }
                 if(S6 > S6_Max){
                  S6_Max = S6;
                 }
                 if(S7 > S7_Max){
                  S7_Max = S7;
                 }
                 if(S8 > S8_Max){
                  S8_Max = S8;
                 }
}
