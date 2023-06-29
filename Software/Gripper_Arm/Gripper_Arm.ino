
int RED_LED = 21;
int GREEN_LED = 22;
int BLUE_LED = 23;

int MTR_CTRL_1 = 17;
int MTR_CTRL_2 = 18;

void setup() {
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(MTR_CTRL_1, OUTPUT);
  pinMode(MTR_CTRL_2, OUTPUT);

  digitalWrite(MTR_CTRL_1, HIGH);
  digitalWrite(MTR_CTRL_2, LOW);
}


void loop() {
  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, HIGH);
  delay(1000);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);
  delay(1000);
}