// Inclusão das bibliotecas

#include <AccelStepper.h> // Biblioteca do motor de passo
#include <math.h> // Biblioteca de matemática
#include <Servo.h> // Biblioteca de servo motor

// Define stepper motor connections and steps per revolution:

// Stepper motor J1

#define dirJ1 55 // Pino de direção
#define stepJ1 54 // Pino de Pulso
#define enlJ1 38 // Pino de Permissão
#define velJ1 1000 // Velocidade do motor J1 (Pulsos/s)

// Stepper motor J2

#define dirJ2 61 // Pino de direção
#define stepJ2 60 // Pino de Pulso
#define enlJ2 56 // Pino de Permissão
#define velJ2 1000 // Velocidade do motor J2 (Pulsos/s)

// Stepper motor J3

#define dirJ3 48 // Pino de direção
#define stepJ3 46 // Pino de Pulso
#define enlJ3 62 // Pino de Permissão
#define velJ3 1000 // Velocidade do motor J3 (Pulsos/s)

#define motorInterfaceType 1 // Interface tipo 1 

// Servo motor J4

#define pinJ4 11 // Pino de PWM

// Servo motor J5

#define pinJ5 6 // Pino de PWM

// Servo motor J6

#define pinJ6 5 // Pino de PWM

// Reduções

#define red1 5
#define red2 8.6
#define red3 7.5

// Delay

#define delayServo 10

float valorJ1 = 0;
float valorJ2 = 0;
float valorJ3 = 0;
float valorJ4 = 0;
float valorJ5 = 0;
float valorJ6 = 0;
float valorM;

// Definir motores de passo
AccelStepper stepperJ1 = AccelStepper(motorInterfaceType, stepJ1, dirJ1); // Cria o objeto stepperJ1 - Motor J1
AccelStepper stepperJ2 = AccelStepper(motorInterfaceType, stepJ2, dirJ2); // Cria o objeto stepperJ2 - Motor J2
AccelStepper stepperJ3 = AccelStepper(motorInterfaceType, stepJ3, dirJ3); // Cria o objeto stepperJ3 - Motor J3

// Definir servos motores

Servo servoJ4; // Cria o objeto servoJ4 - Motor J4
Servo servoJ5; // Cria o objeto servoJ5 - Motor J5
Servo servoJ6; // Cria o objeto servoJ6 - Motor J6

void setup() {

  Serial.begin(9600); // Inicia comunicacao serial

  while(!Serial){
    // Aguarda até que a comunicação serial esteja pronta
  }
  stepperJ1.setMaxSpeed(2000); // Velocidade máxima do motor 2000 passos/segundo
  stepperJ2.setMaxSpeed(2000); // Velocidade máxima do motor 2000 passos/segundo
  stepperJ3.setMaxSpeed(2000); // Velocidade máxima do motor 2000 passos/segundo

  pinMode(enlJ1, OUTPUT); // Pino enable J1 como Output
  pinMode(enlJ2, OUTPUT); // Pino enable J2 como Output
  pinMode(enlJ3, OUTPUT); // Pino enable J3 como Output
  
  digitalWrite(enlJ1, LOW); // Pino enable J1 como Low - 0V
  digitalWrite(enlJ2, LOW); // Pino enable J2 como Low - 0V
  digitalWrite(enlJ3, LOW); // Pino enable J3 como Low - 0V

  servoJ4.attach(pinJ4); // Pino do Servo J4
  servoJ5.attach(pinJ5); // Pino do Servo J5
  servoJ6.attach(pinJ6); // Pino do Servo J6

  // Ajusta os motores de passo para a posição zero

  stepperJ1.setCurrentPosition(0);
  stepperJ2.setCurrentPosition(0);
  stepperJ3.setCurrentPosition(0);

  start(); // Realiza os procedimentos de posicionamento inicial do manipulador

}

void loop() {
  Serial.print(int (floor((360*stepperJ1.currentPosition())/6400)/red1));
  Serial.print(" ");
  Serial.print(int (floor((360*stepperJ2.currentPosition())/6400)/red2));
  Serial.print(" ");
  Serial.print(int (floor((360*stepperJ3.currentPosition())/6400)/red2));
  Serial.print(" ");
  Serial.print(servoJ4.read());
  Serial.print(" ");
  Serial.print(servoJ5.read());
  Serial.print(" ");
  Serial.println(servoJ6.read());

  if (Serial.available() > 0){
  
    valorJ1 = Serial.parseInt(); // Faz a leitura do primeiro valor relativo ao J1

    valorJ2 = Serial.parseInt(); // Faz a leitura do segundo valor relativo ao J2

    valorJ3 = Serial.parseInt(); // Faz a leitura do terceiro valor relativo ao J3

    valorJ4 = Serial.parseInt(); // Faz a leitura do quarto valor relativo ao J4

    valorJ5 = Serial.parseInt(); // Faz a leitura do quinto valor relativo ao J5

    valorJ6 = Serial.parseInt(); // Faz a leitura do sexto valor relativo ao J6

    valorM = Serial.parseInt(); // Faz a leitura do modo de uso

    if (valorM == 0){

      J1(valorJ1);
      J2(valorJ2);
      J3(valorJ3);
      J4(valorJ4);
      J5(valorJ5);
      J6(valorJ6);

    }

    else if (valorM == 1){

      end();

    }



    Serial.flush();

  }

  delay(3000);

}

void J1(float N) { //Função de movimentação da J1

  N = floor(((N/360)*6400)*red1); // Converte graus para pulsos

  if ((N - stepperJ1.currentPosition()) > 0){

    Serial.print("J1: ");
    Serial.println(N);  

    while(stepperJ1.currentPosition() != N){
      stepperJ1.setSpeed(velJ1);
      stepperJ1.runSpeed();
    }


  } 

  else if ((N - stepperJ1.currentPosition()) < 0){

    Serial.print("J1: ");
    Serial.println(N);  

    while(stepperJ1.currentPosition() != N){
      stepperJ1.setSpeed(-velJ1);
      stepperJ1.runSpeed();
    }


  }    

  else if ((N - stepperJ1.currentPosition()) == 0 ){
  

  } 

  }

void J2(float N) { //Funcao para movimentar J2

  N = floor(((N/360)*6400)*red2); // Converte graus para pulsos

  if ((N - stepperJ2.currentPosition()) > 0){

    Serial.print("J2: ");
    Serial.println(N);  

    while(stepperJ2.currentPosition() != N){
      stepperJ2.setSpeed(velJ2);
      stepperJ2.runSpeed();
    }


  } 

  else if ((N - stepperJ2.currentPosition()) < 0){

    Serial.print("J2: ");
    Serial.println(N);  

    while(stepperJ2.currentPosition() != N){
      stepperJ2.setSpeed(-velJ2);
      stepperJ2.runSpeed();
    }


  }    

  else if ((N - stepperJ2.currentPosition()) == 0 ){
  

  } 

}

void J3(float N) { //Funcao para movimentar J3

  N = floor(((N/360)*6400)*red3); // Converte graus para pulsos

  if ((N - stepperJ3.currentPosition()) > 0){

    Serial.print("J3: ");
    Serial.println(N);  

    while(stepperJ3.currentPosition() != N){
      stepperJ3.setSpeed(velJ3);
      stepperJ3.runSpeed();
    }


  } 

  else if ((N - stepperJ3.currentPosition()) < 0){

    Serial.print("J3: ");
    Serial.println(N);  

    while(stepperJ3.currentPosition() != N){
      stepperJ3.setSpeed(-velJ3);
      stepperJ3.runSpeed();
    }


  }    

  else if ((N - stepperJ3.currentPosition()) == 0 ){
  

  } 

}

void J4(float N) {

  if (N > 0) {
    
    Serial.print("J4: ");
    Serial.println(N);

    if ((N - servoJ4.read() > 0)) {
      
      for (int pos = servoJ4.read(); pos < N; pos += 1){

        delay(delayServo);
        servoJ4.write(pos);

      }

    }

    else if ((N - servoJ4.read() < 0)) {

        for (int pos = servoJ4.read(); pos > N; pos -= 1){
          delay(delayServo);
          servoJ4.write(pos);
        }
      
    }

    delay(1500);

  }

}

void J5(float N) {

  if (N > 0) {
    
    Serial.print("J5: ");
    Serial.println(N);

    if ((N - servoJ5.read() > 0)) {
      
      for (int pos = servoJ5.read(); pos < N; pos += 1){

        delay(delayServo);
        servoJ5.write(pos);

      }

    }

    else if ((N - servoJ5.read() < 0)) {

        for (int pos = servoJ5.read(); pos > N; pos -= 1){
          delay(delayServo);
          servoJ5.write(pos);
        }
      
    }

    delay(1500);

  }

}

void J6(float N) {

  if (N > 0) {
    
    Serial.print("J6: ");
    Serial.println(N);

    if ((N - servoJ6.read() > 0)) {
      
      for (int pos = servoJ6.read(); pos < N; pos += 1){

        delay(delayServo);
        servoJ6.write(pos);

      }

    }

    else if ((N - servoJ6.read() < 0)) {

        for (int pos = servoJ6.read(); pos > N; pos -= 1){
          delay(delayServo);
          servoJ6.write(pos);
        }
      
    }
    delay(1500);

  }

}

void start() {

  Serial.println("Iniciando o manipulador");
  Serial.println("Por favor, verifique o posicionamento inicial...");
  
  while (!Serial.available()) {
    // Aguarda alguma entrada
  }

  Serial.read();
  Serial.println("Posicionamento inicial em andamento, aguarde...");
 
  J3(45);
  J2(45);
  J3(60);
  J2(60);
  J3(100);
  J2(110);

  stepperJ1.setCurrentPosition(0);
  stepperJ2.setCurrentPosition(0);
  stepperJ3.setCurrentPosition(0);

  J2(-45);
  J3(-45);

  Serial.println("Posicionamento finalizado");
  Serial.flush();

}

void end(){

  Serial.println("Encerrando movimentação");

  J1(0);
  J2(0);
  J3(0);
  J4(90);
  J5(90);
  J6(90);

  J2(-45);
  J3(-45);
  J2(-80);
  J3(-80);
  J2(-110);
  J3(-100);

  Serial.println("Desligando a comunicação com o manipulador");
  Serial.end();

}



