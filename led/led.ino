
int pin2=2;
int pin3=3;
int pin4=4;


char state;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(pin2,OUTPUT);
  pinMode(pin3,OUTPUT);
  pinMode(pin4,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()>0){
  state=Serial.read();
  Serial.println(state);
  }
  if(state=='2'){
    digitalWrite(pin2, HIGH);
    delay(3000);
  }
  else if(state=='3'){
    digitalWrite(pin3, HIGH);
    delay(3000);
  }
  else if(state=='4'){
    digitalWrite(pin4, HIGH);
    delay(3000); 
  }
  else if(state=='a'){
    digitalWrite(pin2, LOW);
  }
  else if(state=='b'){
    digitalWrite(pin3, LOW);
  }
  else if(state=='c'){
    digitalWrite(pin4, LOW);
  }
  /*
  if(val==2){
    digitalWrite(pin1, HIGH);
    digitalWrite(pin1, HIGH);
  digitalWrite(pin2, HIGH);
  digitalWrite(pin3, HIGH);
  digitalWrite(pin4, HIGH);
  }

  else{
    
    digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
  digitalWrite(pin3, LOW);
  digitalWrite(pin4, LOW);// turn the LED off by making the voltage LOW

  }// wait for a second*/
}
