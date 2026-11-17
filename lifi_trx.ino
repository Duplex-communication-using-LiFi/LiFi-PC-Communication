// Li-Fi transceiver (full duplex): LED transmitter on D9, photodiode receiver on D2
//
// TX wiring:  D9 -> 10k -> NPN base; NPN collector -> LED cathode;
//             LED anode -> 220R -> 5V; NPN emitter -> GND.   (D9 HIGH = LED ON)
// RX wiring:  Photodiode -> OPA320 transimpedance amp (1M feedback) -> LM393
//             comparator -> D2 (10k pull-up to 5V).  Light ON  => D2 HIGH.
//
// Frame per byte (idle = LED OFF / D2 LOW), same on TX and RX:
//   start bit (ON) | 8 data bits, LSB first | stop bit (OFF)
//
// Both Arduinos MUST use the same bit rate (set from the PC app).
//
// PC -> Arduino (115200 baud):
//   'P'              ping                         -> replies 'K'
//   'B' hi lo        set bit rate in bits/s       -> replies 'K'
//   'D' len data...  queue bytes for the LED
// Arduino -> PC:
//   'R' byte         one byte received from the photodiode
//
// TX and RX are non-blocking state machines, so both run at the same time.

const int LED_PIN = 9;
const int RX_PIN = 2;

unsigned long bitUs = 10000;  // default 100 bit/s

// ---------------- transmit ----------------
byte txq[256];
byte txHead = 0, txTail = 0;  // byte indices wrap at 256
byte txCur = 0;
byte txBit = 0;
bool txBusy = false;
unsigned long txNext = 0;

void txStep() {
  unsigned long now = micros();
  if (txBusy) {
    if ((long)(now - txNext) < 0) return;
    txBit++;
    if (txBit <= 8) {                       // data bits 0..7
      digitalWrite(LED_PIN, (txCur >> (txBit - 1)) & 1);
      txNext += bitUs;
      return;
    }
    if (txBit == 9) {                       // stop bit
      digitalWrite(LED_PIN, LOW);
      txNext += bitUs;
      return;
    }
    txBusy = false;                         // frame finished
  }
  if (txHead != txTail) {                   // start next frame
    txCur = txq[txTail++];
    txBit = 0;
    txBusy = true;
    digitalWrite(LED_PIN, HIGH);            // start bit
    txNext = now + bitUs;
  }
}

// ---------------- receive ----------------
byte rxState = 0;  // 0 idle, 1 check start, 2 data bits, 3 stop bit
byte rxBit = 0;
byte rxByte = 0;
unsigned long rxNext = 0;

void rxStep() {
  unsigned long now = micros();
  if (rxState == 0) {
    if (digitalRead(RX_PIN)) {              // rising edge = start bit
      rxState = 1;
      rxNext = now + bitUs / 2;             // sample middle of start bit
    }
    return;
  }
  if ((long)(now - rxNext) < 0) return;
  if (rxState == 1) {
    if (!digitalRead(RX_PIN)) { rxState = 0; return; }  // glitch
    rxState = 2;
    rxBit = 0;
    rxByte = 0;
    rxNext += bitUs;
    return;
  }
  if (rxState == 2) {
    if (digitalRead(RX_PIN)) rxByte |= (1 << rxBit);
    rxBit++;
    rxNext += bitUs;
    if (rxBit == 8) rxState = 3;
    return;
  }
  // stop bit: must be LOW
  if (!digitalRead(RX_PIN)) {
    Serial.write('R');
    Serial.write(rxByte);
  }
  rxState = 0;
}

// ---------------- commands from PC ----------------
byte cmdState = 0;
byte cmdHi = 0;
byte cmdLen = 0;

void serialStep() {
  if (!Serial.available()) return;
  byte c = Serial.read();
  switch (cmdState) {
    case 0:
      if (c == 'P') Serial.write('K');
      else if (c == 'B') cmdState = 1;
      else if (c == 'D') cmdState = 3;
      break;
    case 1:
      cmdHi = c;
      cmdState = 2;
      break;
    case 2: {
      unsigned int bps = ((unsigned int)cmdHi << 8) | c;
      if (bps > 0) bitUs = 1000000UL / bps;
      Serial.write('K');
      cmdState = 0;
      break;
    }
    case 3:
      cmdLen = c;
      cmdState = cmdLen ? 4 : 0;
      break;
    case 4:
      if ((byte)(txHead + 1) != txTail) txq[txHead++] = c;  // drop if full
      if (--cmdLen == 0) cmdState = 0;
      break;
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(RX_PIN, INPUT);
  Serial.begin(115200);
}

void loop() {
  txStep();
  rxStep();
  serialStep();
}
