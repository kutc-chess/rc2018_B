void setup() {
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, INPUT);
  pinMode(13, OUTPUT);
  digitalWrite(5, 0);
  digitalWrite(6, 0);
  digitalWrite(13, 1);
}

int wait = 310;
unsigned long prev = 0;
boolean flag = 0;

void loop() {
  if(digitalRead(7) == 1){
    if(flag == 0){
      // wait = ;
      prev = millis();
      flag = 1;
    }
    digitalWrite(5, 1);
  }
  else{
    digitalWrite(5, 0);
    flag = 0;
  }

  if((millis() - prev) > wait && flag){
    digitalWrite(6, 1);
  }
  else{
    digitalWrite(6, 0);
  }
}
