#include "ServoEasing.h"
#include "LightweightServo.h"
#include <avr/wdt.h>

#define WS2811_PORT      PORTD
#define LED_PIN 0        //PD0
#define VESC_RELAY_PIN 8 //PB0 //also switches FAN
#define FAN_PWR_PIN 9    //PB1
#define PPM_PIN 10       //PB2
//#define FAN_PWM_PIN 11    //PB3

#define WDI_PIN A0                //PC0
#define PWR_SENS_POWER_PIN A1     //PC1
#define TRG_SENS_POWER_PIN A2     //PC2
#define PWR_SENS_INPUT_PIN A3     //PC3
#define TRG_SENS_INPUT_PIN A4     //PC4

#define STOP 90
#define START_DEGREE_VALUE  90
#define MODE_STOP 1

#include "ws2811.h"

int mappings[] = {67, STOP, 113, 134, 156, 180}; //-25%, 0, 25%, 50%, 75%, 100%


using namespace ws2811;
namespace {
  /// transmit on bit 0
  const uint8_t channel = 0;
  const uint8_t led_count = 23;
  ws2811::rgb leds[led_count];
}

ServoEasing vesc;
uint8_t is_on = 0;
uint8_t wdt_triggered = 0;
uint32_t wdi_toggled_at = 0;
uint32_t led_last_updated_at = 0;
uint32_t vesc_last_updated_at = 0;
uint32_t mode_last_switched_at = 0;


int mode=0;
int last_mode=0;
int previous_mode=0;
int stored_mode=0;
uint32_t last_switch_touched_at;
int previous_switch_position = 0;
int switch_position = 0;
int moves_up_count = 0;
int moves_down_count = 0;

uint8_t should_indicate_max_speed_reached = 0;
uint8_t should_indicate_min_speed_reached = 0;

uint8_t party_index=0;

void setup_wdt(){
  cli();
  WDTCSR |= (_BV(WDCE) | _BV(WDE));   // Enable the WD Change Bit
  WDTCSR =   _BV(WDIE) |              // Enable WDT Interrupt
         _BV(WDP1) | _BV(WDP2); //1 seconds
  sei();
};

ISR(WDT_vect){
  wdt_triggered = 1;
}

void setup() {  
  MCUSR &= ~_BV(WDRF);                 // Clear the WDT reset flag
  WDTCSR |= (_BV(WDCE) | _BV(WDE));   // Enable the WD Change Bit
  WDTCSR = 0x00;
  setup_wdt();
  pinMode(WDI_PIN, OUTPUT);
  pinMode(PWR_SENS_POWER_PIN, OUTPUT);
  pinMode(TRG_SENS_POWER_PIN, OUTPUT);
  pinMode(PWR_SENS_INPUT_PIN, INPUT_PULLUP);
  pinMode(TRG_SENS_INPUT_PIN, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT);
  pinMode(VESC_RELAY_PIN, OUTPUT);
  pinMode(FAN_PWR_PIN, OUTPUT);
//  pinMode(FAN_PWM_PIN, OUTPUT);
  pinMode(PPM_PIN, OUTPUT);

  digitalWrite(FAN_PWR_PIN, LOW);
//  analogWrite(FAN_PWM_PIN, 0x00);

  digitalWrite(PWR_SENS_POWER_PIN, HIGH); //turn on main sensor
  for (uint8_t idx = 0; idx < led_count ; idx++){
      leds[idx].red = 0;
      leds[idx].green = 0;
      leds[idx].blue = 0;
  }
  cli();
  send( leds, channel);
  sei();

  vesc.attach(PPM_PIN);
  vesc.write(START_DEGREE_VALUE);
  vesc.setSpeed(20);//"degrees" per second. 0 to 100% is 90 "degrees". full speed = 4.5 seconds with 20 degrees speed
//  vesc.setEasingType(EASE_LINEAR); //already set "PROVIDE_ONLY_LINEAR_MOVEMENT" inside ServoEasing.h + additionally set USE_LEIGHTWEIGHT_SERVO_LIB to save space and reduce jitter +added atmega168 to lightweight's lib header and source

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
}

void toggleWDI() {
  digitalWrite(WDI_PIN, !digitalRead(WDI_PIN));
  wdi_toggled_at = millis();
}

void switch_on() {
  digitalWrite(VESC_RELAY_PIN, HIGH); //turn on VESC
  digitalWrite(TRG_SENS_POWER_PIN, HIGH);
  digitalWrite(FAN_PWR_PIN, HIGH);
}

void switch_off() {
  digitalWrite(VESC_RELAY_PIN, LOW);
  digitalWrite(TRG_SENS_POWER_PIN, LOW);
  digitalWrite(FAN_PWR_PIN, LOW);
  is_on = 0;
}

bool trigger_is_up(){
  return switch_position==1;
}

bool has_input_to_process(){
  return moves_down_count > 0 || moves_up_count > 0;
}

void clear_inputs(){
  moves_down_count = 0;
  moves_up_count = 0;
}

void pick_working_mode() {
  if(previous_switch_position != switch_position && (millis() - last_switch_touched_at ) > 50){ //switch triggeres with 50ms debounce
    if(trigger_is_up()){
//      Serial.println("+");
      moves_up_count++;
    }else{
//      Serial.println("-");
      moves_down_count++;
    }
     
    previous_switch_position = switch_position;
    last_switch_touched_at = millis();
  }

  if(trigger_is_up() && has_input_to_process() && (millis() - last_switch_touched_at) > 500){ // fix speed
//    Serial.print("Moves up: ");
//    Serial.println(moves_up_count);
//    Serial.print("Moves down: ");
//    Serial.println(moves_down_count);

    if(moves_up_count == 1 && moves_down_count == 0){ //one time pulled trigger - restoring previous speed, or starting from the lowert
      if(stored_mode <= MODE_STOP){
        mode = MODE_STOP + 1;
      }else{
        mode = stored_mode;
      }
    }

    if(moves_down_count == 1){ //one time pulled trigger fast speed increase
      if(mode < (sizeof(mappings)/sizeof(mappings[0]) - 1) ){
        mode++;
      }else{
        should_indicate_max_speed_reached = 1;
//        Serial.print("TOP SPEED: ");
//        Serial.println(mode);
      }
    }

    if(moves_down_count == 2){ //two or more times pulled trigger fast - desrease speed
      if(mode > MODE_STOP + 1){
        mode--;
      }else{
        should_indicate_min_speed_reached = 1;
        mode = MODE_STOP + 1;
//        Serial.print("MIN SPEED: ");
//        Serial.println(mode);
      }
    }

    if(moves_down_count > 2){ //plenty moves - reverse
      if(mode > MODE_STOP){
        stored_mode = mode;
      }
      mode=0;
    }
    clear_inputs();
  }

  if(!trigger_is_up() && (millis() - last_switch_touched_at) > 300){ // shut down
    mode = MODE_STOP;
    clear_inputs();
  }

  if(last_mode != mode) {
    previous_mode = last_mode;
    if(mode < last_mode && mode > 0){
      vesc.stop();
      vesc.write(mappings[mode]);
    }
    if(mode == 1){
      vesc.stop();
      vesc.write(mappings[mode]);
    }
    if( (mode > 1 && mode > last_mode) || mode == 0){
      vesc.startEaseTo(mappings[mode]);
    }
    last_mode = mode;
    mode_last_switched_at = millis();
    
    if(mode > 1){
      stored_mode = mode;
    }
  }
}

void process_led() {
  if(mode == MODE_STOP){
    for (uint8_t idx = 0; idx < led_count ; idx++){
      leds[idx].red = 0;
      leds[idx].green = 0;
      leds[idx].blue = 128;
    }
  }

  if(mode < MODE_STOP){
    for (uint8_t idx = 0; idx < led_count ; idx++){
      leds[idx].red = 128;
      leds[idx].green = 0;
      leds[idx].blue = 0;
    }
  }

  if(mode > MODE_STOP){
    for (uint8_t idx = 0; idx < led_count ; idx++){
      leds[idx].red = 0;
      leds[idx].green = 64*(mode-1) - 1;
      leds[idx].blue = 0;
    }
  }

  if(mode == 5){ //top speed, party time
    
    for (uint8_t idx = 0; idx < led_count ; idx++){
      leds[idx].blue = 0;
      if(idx==party_index){
        leds[idx].red = 0xFF;
        leds[idx].green = 0;
      }else{
        leds[idx].red = 0;
        leds[idx].green = 192;
      }
    }
    party_index++;
    if(party_index==23){
      party_index = 0;
    }
  }

  if(should_indicate_max_speed_reached || should_indicate_min_speed_reached){
    for(uint8_t i=0;i<5; i++){
      for (uint8_t idx = 0; idx < led_count ; idx++){
        leds[idx].red = 127;
        leds[idx].green = 127;
        leds[idx].blue = 127;
      }
      cli();
      send( leds, channel);
      sei();
      delay(100);
      for (uint8_t idx = 0; idx < led_count ; idx++){
        leds[idx].red = 0;
        leds[idx].green = 0;
        leds[idx].blue = 0;
      }
      cli();
      send( leds, channel);
      sei();
      delay(100);
    }
    
    should_indicate_max_speed_reached = 0;
    should_indicate_min_speed_reached = 0;
  }
  cli();
  send( leds, channel);
  sei();
  led_last_updated_at = millis();
}

void process_fan(){
  if(mode < 3){
    digitalWrite(FAN_PWR_PIN, LOW);
  }

  if(mode >= 3){
    digitalWrite(FAN_PWR_PIN, HIGH);
  }

  if(mode < 3 && previous_mode >= 3){
    if( (millis() - mode_last_switched_at) < 30000){ //lets blow for another 30 seconds after high modes involved
      digitalWrite(FAN_PWR_PIN, HIGH);
    }
  }
}

void loop() {
  wdt_reset();

  if(millis() > wdi_toggled_at + 1000){
    toggleWDI();
  }

  switch_position = digitalRead(TRG_SENS_INPUT_PIN);

  if(wdt_triggered){
    toggleWDI();
    digitalWrite(PWR_SENS_POWER_PIN, HIGH); //turn on POWER sensor
    _delay_ms(25); //wait to check power sensor properly.
    wdt_triggered = 0;
    clear_inputs();
    mode = MODE_STOP;
  }
  
  if(digitalRead(PWR_SENS_INPUT_PIN)){ //should be on?
    if(!is_on){
      switch_on();
      _delay_ms(50); //just for debounce
      is_on = 1;
    }
  }else{
    digitalWrite(PWR_SENS_POWER_PIN, LOW);
    switch_off();
    sleep_mode();
  }

  pick_working_mode();

  if(millis() > led_last_updated_at + 50){
    process_led();
  }
  process_fan();

}
