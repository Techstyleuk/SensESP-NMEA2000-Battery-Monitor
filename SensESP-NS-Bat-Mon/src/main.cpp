// Complete Nautical Sun Code
// 2 Battery Version for Nautical Sun - 2 battery, 3 Temperature and a CAN connection for NMEA2000
// Possible Updates
//      - Move RShuntX to the App

#include <Arduino.h>

#define CAN_RX_PIN GPIO_NUM_17
#define CAN_TX_PIN GPIO_NUM_16

#include <Wire.h>
#include <Adafruit_INA219.h>
#include "sensesp_onewire/onewire_temperature.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/analogvoltage.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/voltagedivider.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/transforms/frequency.h"
#include <ActisenseReader.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <N2kMsg.h>
#include <ReactESP.h>
#include <Wire.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>

using namespace sensesp;

  // State of Charge lookup
class SoCAInterpreter : public CurveInterpolator {
 public:
  SoCAInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the Battery A Voltage values returned by
    // our INA219_A to % (Ratio)
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownOhmValue, knownPascal));
    add_sample(CurveInterpolator::Sample(2.5, 0));
    add_sample(CurveInterpolator::Sample(10.5, 0));
    add_sample(CurveInterpolator::Sample(11.31, .1));
    add_sample(CurveInterpolator::Sample(11.58, .2));
    add_sample(CurveInterpolator::Sample(11.75, .3));
    add_sample(CurveInterpolator::Sample(11.9, .4));
    add_sample(CurveInterpolator::Sample(12.06, .5));
    add_sample(CurveInterpolator::Sample(12.2, .6));
    add_sample(CurveInterpolator::Sample(12.32, .7));
    add_sample(CurveInterpolator::Sample(12.42, .8)); 
    add_sample(CurveInterpolator::Sample(12.5, .9));
    add_sample(CurveInterpolator::Sample(12.6, 1)); 
    add_sample(CurveInterpolator::Sample(20.0, 1)); 
  }
};
class SoCBInterpreter : public CurveInterpolator {
 public:
  SoCBInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the Battery B Voltage values returned by
    // our INA219_B to % (Ratio)
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownOhmValue, knownPascal));
    add_sample(CurveInterpolator::Sample(2.5, 0));
    add_sample(CurveInterpolator::Sample(10.5, 0));
    add_sample(CurveInterpolator::Sample(11.31, .1));
    add_sample(CurveInterpolator::Sample(11.58, .2));
    add_sample(CurveInterpolator::Sample(11.75, .3));
    add_sample(CurveInterpolator::Sample(11.9, .4));
    add_sample(CurveInterpolator::Sample(12.06, .5));
    add_sample(CurveInterpolator::Sample(12.2, .6));
    add_sample(CurveInterpolator::Sample(12.32, .7));
    add_sample(CurveInterpolator::Sample(12.42, .8)); 
    add_sample(CurveInterpolator::Sample(12.5, .9));
    add_sample(CurveInterpolator::Sample(12.6, 1)); 
    add_sample(CurveInterpolator::Sample(20.0, 1)); 
  }
};

reactesp::ReactESP app;

////////////////////INA219
  Adafruit_INA219 ina219_A;
  Adafruit_INA219 ina219_B(0x41);
  
  const float RshuntA = 0.00025;
  const float RshuntB = 0.00025;

  float read_A_current_callback() { return ((ina219_A.getShuntVoltage_mV() / 1000) / RshuntA);}
  float read_A_shuntvoltage_callback() { return (ina219_A.getShuntVoltage_mV() / 1000);}
  float read_A_busvoltage_callback() { return (ina219_A.getBusVoltage_V());}
  float read_A_loadvoltage_callback() { return (ina219_A.getBusVoltage_V() + (ina219_A.getShuntVoltage_mV() / 1000));}
  float read_A_power_callback() { return ((ina219_A.getBusVoltage_V() + (ina219_A.getShuntVoltage_mV() / 1000)) * (ina219_A.getCurrent_mA() / 1000));}
  float A_capacity_callback() { return (RshuntA / RshuntA);}

  float read_B_current_callback() { return ((ina219_B.getShuntVoltage_mV() / 1000) / RshuntB);}
  float read_B_shuntvoltage_callback() { return (ina219_B.getShuntVoltage_mV() / 1000);}
  float read_B_busvoltage_callback() { return (ina219_B.getBusVoltage_V());}
  float read_B_loadvoltage_callback() { return (ina219_B.getBusVoltage_V() + (ina219_B.getShuntVoltage_mV() / 1000));}
  float read_B_power_callback() { return ((ina219_B.getBusVoltage_V() + (ina219_B.getShuntVoltage_mV() / 1000)) * (ina219_B.getCurrent_mA() / 1000));}
  float B_capacity_callback() { return (RshuntB / RshuntB);}
 ////////////////////INA219 end of Add
 
double BatAVoltage = 0;
double BatACurrent = 0;
double BatBVoltage = 0;
double BatBCurrent = 0;
double BatATemp = 0;
double BatBTemp = 0;
double BatASOC = 0;
double BatBSOC = 0;
double BatACapacity = 0;
double BatBCapacity = 0;
double TempOutside = 0;
int BilgeLevel = 0;

Stream *read_stream = &Serial;
Stream *forward_stream = &Serial;

tActisenseReader actisense_reader;

tNMEA2000 *nmea2000;

void ToggleLed() {
  static bool led_state = false;
  digitalWrite(LED_BUILTIN, led_state);
  led_state = !led_state;
}

int num_n2k_messages = 0;
void HandleStreamN2kMsg(const tN2kMsg &message) {
  //N2kMsg.Print(&Serial);
  num_n2k_messages++;
  ToggleLed();
}

int num_actisense_messages = 0;
void HandleStreamActisenseMsg(const tN2kMsg &message) {
  //N2kMsg.Print(&Serial);
  num_actisense_messages++;
  ToggleLed();
  nmea2000->SendMsg(message);
}

// each message so they will not be sent at same time.
tN2kSyncScheduler DCBatStatusSchedulerA(false,1500,500);
tN2kSyncScheduler DCStatusSchedulerA(false,1500,510);
tN2kSyncScheduler BatConfSchedulerA(false,5000,520); // Non periodic
tN2kSyncScheduler DCBatStatusSchedulerB(false,1500,1000);
tN2kSyncScheduler DCStatusSchedulerB(false,1500,1010);
tN2kSyncScheduler BatConfSchedulerB(false,5000,1020); // Non periodic
tN2kSyncScheduler Temperature(false, 1500, 1030);
tN2kSyncScheduler Bilge(false, 1500, 1040);

// *****************************************************************************
// Call back for NMEA2000 open. This will be called, when library starts bus communication.
// See NMEA2000.SetOnOpen(OnN2kOpen); on setup()
void OnN2kOpen() {
  // Start schedulers now.
  DCBatStatusSchedulerA.UpdateNextTime();
  DCStatusSchedulerA.UpdateNextTime();
  BatConfSchedulerA.UpdateNextTime();
  DCBatStatusSchedulerB.UpdateNextTime();
  DCStatusSchedulerB.UpdateNextTime();
  BatConfSchedulerB.UpdateNextTime();
  Temperature.UpdateNextTime();
  Bilge.UpdateNextTime();
}

String can_state;

void PollCANStatus() {
  // CAN controller registers are SJA1000 compatible.
  // Bus status value 0 indicates bus-on; value 1 indicates bus-off.
  unsigned int bus_status = MODULE_CAN->SR.B.BS;

  switch (bus_status) {
    case 0:
      can_state = "RUNNING";
      break;
    case 1:
      can_state = "BUS-OFF";
      Serial.printf("CAN Status=%d\n", can_state);
      // try to automatically recover by rebooting
      app.onDelay(2000, []() {
        esp_task_wdt_init(1, true);
        esp_task_wdt_add(NULL);
        while (true)
          ;
      });
      break;
  }
}
void SendN2kBatteryA() {
  tN2kMsg N2kMsg;

  double BatAVoltage = read_A_loadvoltage_callback();
  double BatACurrent = read_A_current_callback();

  if ( DCBatStatusSchedulerA.IsTime() ) {
    DCBatStatusSchedulerA.UpdateNextTime();
    SetN2kDCBatStatus(N2kMsg,0,BatAVoltage,BatACurrent,BatATemp,255);
    nmea2000->SendMsg(N2kMsg);
    }

  if ( DCStatusSchedulerA.IsTime() ) {
    DCStatusSchedulerA.UpdateNextTime();
    SetN2kDCStatus(N2kMsg,255,0,N2kDCt_Battery,(BatASOC * 100),1,(BatASOC * (BatACapacity/BatACurrent) * 3600),0,AhToCoulomb(BatACapacity));
    nmea2000->SendMsg(N2kMsg);
  }

    if ( BatConfSchedulerA.IsTime() ) {
    BatConfSchedulerA.UpdateNextTime();
    SetN2kBatConf(N2kMsg,0,N2kDCbt_Flooded,N2kDCES_Yes,N2kDCbnv_12v,N2kDCbc_LeadAcid,AhToCoulomb(BatACapacity),53,1.251,75);
    nmea2000->SendMsg(N2kMsg);
  }
}
void SendN2kBatteryB() {
  tN2kMsg N2kMsg;

  double BatBVoltage = read_B_loadvoltage_callback();
  double BatBCurrent = read_B_current_callback();
   
    if ( DCBatStatusSchedulerB.IsTime() ) {
    DCBatStatusSchedulerB.UpdateNextTime();
    SetN2kDCBatStatus(N2kMsg,1,BatBVoltage,BatBCurrent,BatBTemp,255);
    nmea2000->SendMsg(N2kMsg);
    }

  if ( DCStatusSchedulerB.IsTime() ) {
    DCStatusSchedulerB.UpdateNextTime();
    SetN2kDCStatus(N2kMsg,255,1,N2kDCt_Battery,(BatBSOC * 100),1,(BatBSOC * (BatBCapacity/BatBCurrent) * 3600),0,AhToCoulomb(BatBCapacity));
    nmea2000->SendMsg(N2kMsg);
  }

    if ( BatConfSchedulerB.IsTime() ) {
    BatConfSchedulerB.UpdateNextTime();
    SetN2kBatConf(N2kMsg,1,N2kDCbt_Flooded,N2kDCES_Yes,N2kDCbnv_12v,N2kDCbc_LeadAcid,AhToCoulomb(BatBCapacity),53,1.251,75);
    nmea2000->SendMsg(N2kMsg);
  }
}
void OutsideTempFunct() {
  tN2kMsg N2kMsg;
   
    if ( Temperature.IsTime() ) {
    Temperature.UpdateNextTime();
    SetN2kTemperature(N2kMsg,255,1,N2kts_OutsideTemperature,TempOutside,N2kDoubleNA);
    nmea2000->SendMsg(N2kMsg);
    }
}

void BilgeLevelFunct() {
  tN2kMsg N2kMsg;
   
    if ( Bilge.IsTime() ) {
    Bilge.UpdateNextTime();
    SetN2kFluidLevel(N2kMsg, 10, N2kft_GrayWater, BilgeLevel, 10); 
    nmea2000->SendMsg(N2kMsg);
    }
}

// The setup function performs one-time application initialization.
void setup() {
//#ifndef SERIAL_DEBUG_DISABLED
  //SetupSerialDebug(115200);
//#endif
  Serial.begin(115200);
  Wire.begin(21,22);                // join i2c bus (address optional for master)

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("Nautical-Sun-Battery-Monitor")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();

/// 1-Wire Temp Sensors 

  DallasTemperatureSensors* dts = new DallasTemperatureSensors(25); //digital 2

 // A Battery Temperature - electrical.batteries.A.temperature
  auto* A_bat_temp =
      new OneWireTemperature(dts, 1000, "/Battery A Temperature/oneWire");
  A_bat_temp->connect_to(new Linear(1.0, 0.0, "/Battery A Temperature/linear"))
      ->connect_to(
          new SKOutputFloat("electrical.batteries.A.temperature",
                             "/Battery A Temperature/sk_path"));

 // B Battery Temperature - electrical.batteries.B.temperature
  auto* B_bat_temp =
      new OneWireTemperature(dts, 1000, "/Battery B Temperature/oneWire");
  B_bat_temp->connect_to(new Linear(1.0, 0.0, "/Battery B Temperature/linear"))
      ->connect_to(
          new SKOutputFloat("electrical.batteries.B.temperature",
                             "/Battery B Temperature/sk_path"));

 // Outside Air Temperature - environment.outside.temperature
  auto* OutsideTemp =
      new OneWireTemperature(dts, 1000, "/Outside Air Temperature/oneWire");
  OutsideTemp->connect_to(new Linear(1.0, 0.0, "/Outside Air Temperature/linear"))
      ->connect_to(
          new SKOutputFloat("environment.outside.temperature",
                             "/Outside Air Temperature/sk_path"));
      
///////////////////////////INA219 start
  ina219_A.begin();  // Initialize first board (default address 0x40)
  ina219_B.begin();  // Initialize second board with the address 0x41 ----> A0 soldered = 0x41

//ina219_A
  auto* ina219_A_current =
      new RepeatSensor<float>(1000, read_A_current_callback);
  auto* ina219_A_shuntvoltage = 
      new RepeatSensor<float>(1000, read_A_shuntvoltage_callback);
  auto* ina219_A_busvoltage = 
      new RepeatSensor<float>(1000, read_A_busvoltage_callback);   
  auto* ina219_A_loadvoltage = 
      new RepeatSensor<float>(1000, read_A_loadvoltage_callback); 
  auto* ina219_A_power = 
      new RepeatSensor<float>(1000, read_A_power_callback);  
  auto* ina219_A_SOC = 
      new RepeatSensor<float>(1000, read_A_loadvoltage_callback);   
  auto* Bat_A_CAP = 
      new RepeatSensor<float>(1000, A_capacity_callback); 

  // Send the temperature to the Signal K server as a Float
  ina219_A_current->connect_to(new SKOutputFloat("electrical.batteries.A.current"));
  ina219_A_loadvoltage->connect_to(new SKOutputFloat("electrical.batteries.A.voltage"));
  //ina219_A_shuntvoltage->connect_to(new SKOutputFloat("electrical.batteries.A.shuntv"));
  //ina219_A_busvoltage->connect_to(new SKOutputFloat("electrical.batteries.A.vin-v"));
  ina219_A_power->connect_to(new SKOutputFloat("electrical.batteries.A.power"));
  ina219_A_SOC->connect_to(new SoCAInterpreter("/Battery A/Voltage curve"))
      ->connect_to(new SKOutputFloat("electrical.batteries.A.capacity.stateOfCharge", "/Battery A State of Charge/sk_path"));
 
//ina219_B
  auto* ina219_B_current =
      new RepeatSensor<float>(1000, read_B_current_callback);
  auto* ina219_B_shuntvoltage = 
      new RepeatSensor<float>(1000, read_B_shuntvoltage_callback);
  auto* ina219_B_busvoltage = 
      new RepeatSensor<float>(1000, read_B_busvoltage_callback);   
  auto* ina219_B_loadvoltage = 
      new RepeatSensor<float>(1000, read_B_loadvoltage_callback); 
  auto* ina219_B_power = 
      new RepeatSensor<float>(1000, read_B_power_callback);    
  auto* ina219_B_SOC = 
      new RepeatSensor<float>(1000, read_B_loadvoltage_callback); 
  auto* Bat_B_CAP = 
      new RepeatSensor<float>(1000, B_capacity_callback); 

  // Send the temperature to the Signal K server as a Float
  ina219_B_current->connect_to(new SKOutputFloat("electrical.batteries.B.current"));
  ina219_B_loadvoltage->connect_to(new SKOutputFloat("electrical.batteries.B.voltage"));
  //ina219_B_shuntvoltage->connect_to(new SKOutputFloat("electrical.batteries.B.shuntv"));
  //ina219_B_busvoltage->connect_to(new SKOutputFloat("electrical.batteries.B.vin-v"));
  ina219_B_power->connect_to(new SKOutputFloat("electrical.batteries.B.power"));
  ina219_B_SOC->connect_to(new SoCBInterpreter("/Battery B/Voltage curve"))
      ->connect_to(new SKOutputFloat("electrical.batteries.B.capacity.stateOfCharge", "/Battery B State of Charge/sk_path"));

///////////////////////end of INA add
//// Bilge Monitor /////

auto* bilge = new DigitalInputState(12, INPUT_PULLUP, 5000); //Digital 13

bilge->connect_to(new SKOutputFloat("propulsion.engine.bilge.raw"));

// toggle the LED pin at rate of 1 Hz
  pinMode(LED_BUILTIN, OUTPUT);
  app.onRepeatMicros(1e6 / 1, []() { ToggleLed(); });

  // instantiate the NMEA2000 object
  nmea2000 = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);

  // input the NMEA 2000 messages

  // Reserve enough buffer for sending all messages. This does not work on small
  // memory devices like Uno or Mega
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

  // Set Product information
  nmea2000->SetProductInformation(
      "20230223",  // Manufacturer's Model serial code (max 32 chars)
      100,         // Manufacturer's product code
      "JG Battery Monitor",  // Manufacturer's Model ID (max 33 chars)
      "0.1.0.0 (2023-02-23)",  // Manufacturer's Software version code (max 40
                               // chars)
      "0.1.0.0 (2023-02-23)"   // Manufacturer's Model version (max 24 chars)
  );
  // Set device information
  nmea2000->SetDeviceInformation(
      1,    // Unique number. Use e.g. Serial number.
      170,  // Device function=Analog to NMEA 2000 Gateway. See codes on
            // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      35,   // Device class=Inter/Intranetwork Device. See codes on
           // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      122  // Just choosen free from code list on
            // http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );

  nmea2000->SetForwardStream(forward_stream);
  nmea2000->SetMode(tNMEA2000::N2km_ListenAndNode);
  nmea2000->SetForwardType(tNMEA2000::fwdt_Text); // Show bus data in clear
  // text
  nmea2000->SetForwardOwnMessages(false);  // do not echo own messages.
  nmea2000->SetMsgHandler(HandleStreamN2kMsg);
  nmea2000->SetOnOpen(OnN2kOpen);
  nmea2000->Open();

  actisense_reader.SetReadStream(read_stream);
  actisense_reader.SetDefaultSource(75);
  actisense_reader.SetMsgHandler(HandleStreamActisenseMsg);

  // No need to parse the messages at every single loop iteration; 1 ms will do
  app.onRepeat(1, []() {
    nmea2000->ParseMessages();
    actisense_reader.ParseMessages();
  });

  // enable CAN status polling
  app.onRepeat(100, []() { PollCANStatus(); });

    A_bat_temp->connect_to(
      new LambdaConsumer<float>([](float temperatureA) {
        BatATemp = temperatureA;
        SendN2kBatteryA();
      }));
    ina219_A_SOC->connect_to(new SoCAInterpreter("/Battery A/Voltage/curve"))
      ->connect_to(new LambdaConsumer<float>([](float SOCA) {
        BatASOC = SOCA;
        SendN2kBatteryA();
      }));
    Bat_A_CAP->connect_to(new Linear(1.0, 0.0, "/Battery A/Capacity")) 
          ->connect_to(new LambdaConsumer<float>([](float CapA) {
        BatACapacity = CapA;
        SendN2kBatteryA();
      })); 

    B_bat_temp->connect_to(
      new LambdaConsumer<float>([](float temperatureB) {
        BatBTemp = temperatureB;
        SendN2kBatteryB();
      }));
    ina219_B_SOC->connect_to(new SoCBInterpreter("/Battery_B/Voltage/curve"))
      ->connect_to(new LambdaConsumer<float>([](float SOCB) {
        BatBSOC = SOCB;
        SendN2kBatteryB();
      }));
    Bat_B_CAP->connect_to(new Linear(1.0, 0.0, "/Battery B/Capacity")) 
          ->connect_to(new LambdaConsumer<float>([](float CapB) {
        BatBCapacity = CapB;
        SendN2kBatteryB();
      })); 
   
    OutsideTemp->connect_to(
      new LambdaConsumer<float>([](float temperatureOut) {
        TempOutside = temperatureOut;
        OutsideTempFunct();
      }));

    bilge->connect_to(
      new LambdaConsumer<int>([](int bilgeOut) {
        BilgeLevel = bilgeOut;
        BilgeLevelFunct();
      }));

  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();
}



void loop() { 
  app.tick(); 
  }
