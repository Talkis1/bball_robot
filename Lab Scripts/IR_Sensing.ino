#define IR 10

int r = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(IR, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly: 
  r = digitalRead(IR);
  Serial.println(r);
}
