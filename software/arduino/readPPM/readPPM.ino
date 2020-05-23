#define CHANNEL_AMOUNT 6
#define INT_PIN 2
#define BLANK_TIME 2100

volatile byte pulseCounter = 0;
volatile unsigned long rawValues[CHANNEL_AMOUNT] = {0, 0, 0, 0, 0, 0};
volatile unsigned long microsAtLastPulse = 0;
volatile bool dataAvailable = false;

void setup() {
  Serial.begin(115200);

  pinMode(INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), interruptCallback, RISING);
}

void loop() {
  if (dataAvailable) {
    dataAvailable = false;
    Serial.print(rawValues[0]);
    Serial.print(',');
    Serial.print(rawValues[1]);
    Serial.print(',');
    Serial.print(rawValues[2]);
    Serial.print(',');
    Serial.print(rawValues[3]);
    Serial.print(',');
    Serial.print(rawValues[4]);
    Serial.print(',');
    Serial.println(rawValues[5]);
  }
}

void interruptCallback() {
  unsigned long previousMicros = microsAtLastPulse;
  microsAtLastPulse = micros();
  unsigned long time = microsAtLastPulse - previousMicros;

  if (time > BLANK_TIME) {
    pulseCounter = 0;
    dataAvailable = true;
  }
  else {
    if (pulseCounter < CHANNEL_AMOUNT) {
      rawValues[pulseCounter] = time;
    }
    ++pulseCounter;
  }
}
