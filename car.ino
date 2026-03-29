#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

#define TRIG_PIN 2
#define ECHO_PIN 3

#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11
#define ENA 5
#define ENB 6

#define MOTOR_HIZ_SOL 200
#define MOTOR_HIZ_SAG 180
#define ENGEL_MESAFE 20
#define DONUS_SURESI 500
#define DUZELT_SURESI 500

unsigned long sonLcdGuncelleme = 0;
#define LCD_GUNCELLEME_ARASI 2000

void ileriGit() {
  analogWrite(ENA, MOTOR_HIZ_SOL);
  analogWrite(ENB, MOTOR_HIZ_SAG);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void geriGit() {
  analogWrite(ENA, MOTOR_HIZ_SOL);
  analogWrite(ENB, MOTOR_HIZ_SAG);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void solaDon() {
  analogWrite(ENA, MOTOR_HIZ_SOL);
  analogWrite(ENB, MOTOR_HIZ_SAG);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void sagaDon() {
  analogWrite(ENA, MOTOR_HIZ_SOL);
  analogWrite(ENB, MOTOR_HIZ_SAG);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void dur() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

long mesafeOlc() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long sure = pulseIn(ECHO_PIN, HIGH, 30000);
  long cm = sure / 58;
  if (cm == 0)
    cm = 999;
  return cm;
}

void engeldenKac() {

  dur();
  delay(200);

  geriGit();
  delay(300);
  dur();
  delay(100);

  sagaDon();
  delay(DONUS_SURESI);
  dur();
  delay(100);

  ileriGit();
  delay(600);
  dur();
  delay(100);

  solaDon();
  delay(DUZELT_SURESI);
  dur();
  delay(100);

  ileriGit();
}

void lcdGuncelle() {
  float nem = dht.readHumidity();
  float sicaklik = dht.readTemperature();

  if (!isnan(sicaklik)) {
    Serial.print("TEMP:");
    Serial.println(sicaklik, 1);
  }
  if (!isnan(nem)) {
    Serial.print("HUM:");
    Serial.println(nem, 1);
  }

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("   Ahien-14   ");

  lcd.setCursor(0, 1);
  if (!isnan(sicaklik) && !isnan(nem)) {
    lcd.print("T:");
    lcd.print((int)sicaklik);
    lcd.print("C N:");
    lcd.print((int)nem);
    lcd.print("%");
  } else {
    lcd.print("Sensor Hatasi");
  }
}

void setup() {

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("   Ahien-14   ");
  lcd.setCursor(0, 1);
  lcd.print("  Baslatiliyor");
  delay(2000);

  dht.begin();

  Serial.begin(9600);

  lcdGuncelle();
}

void loop() {
  long mesafe = mesafeOlc();
  Serial.print("DIST:");
  Serial.println(mesafe);

  unsigned long simdi = millis();
  if (simdi - sonLcdGuncelleme >= LCD_GUNCELLEME_ARASI) {
    lcdGuncelle();
    sonLcdGuncelleme = simdi;
  }

  if (mesafe > 0 && mesafe < ENGEL_MESAFE) {
    dur();
  }

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() >= 3) {
      char direction = command.charAt(0);
      int duration = command.substring(2).toInt();

      if (direction == 'F') {
        if (mesafe >= ENGEL_MESAFE || mesafe == 0) {
          ileriGit();
          delay(duration);
        } else {
          Serial.println("Engel var, ileri gidemem!");
        }
      } else if (direction == 'B') {
        geriGit();
        delay(duration);
      } else if (direction == 'L') {
        solaDon();
        delay(duration);
      } else if (direction == 'R') {
        sagaDon();
        delay(duration);
      }

      dur();
    }
  }
  delay(100);
}
