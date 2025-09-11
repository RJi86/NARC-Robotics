#include <Arduino.h>
#include "IRcluster/IRRing.h"
#include "Ircluster/LightCLuster.h"
#include "controllers/IRSensorController.h"
#include "controllers/LightSensorController.h"
#include "controllers/VisionFusion.h"

IRRing hwIR;
LightCluster hwLT;
IRSensorController irCtrl(hwIR, 0.6f);
LightSensorController ltCtrl(hwLT, 0.6f);
VisionFusion fusion(irCtrl, ltCtrl);

void setup(){
Serial.begin(115200); while(!Serial){}
Serial.println(F("Vision controllers online"));
// Optional ambient calib at startup (robot should face neutral scene)
irCtrl.calibrateAmbient(32);
ltCtrl.calibrateAmbient(32);
}

void loop(){
fusion.update();
auto ir = irCtrl.bearing();
auto lt = ltCtrl.bearing();
auto fx = fusion.fused();

Serial.print(F("IR deg= ")); Serial.print(ir.bearing_rad * 180.0f / clustercfg::kPI, 1);
Serial.print(F(" c=")); Serial.print(ir.confidence,2);
Serial.print(F(" LT deg= ")); Serial.print(lt.bearing_rad * 180.0f / clustercfg::kPI, 1);
Serial.print(F(" c=")); Serial.print(lt.confidence,2);
Serial.print(F(" FUSED deg= ")); Serial.print(fx.bearing_rad * 180.0f / clustercfg::kPI, 1);
Serial.print(F(" c=")); Serial.println(fx.confidence,2);

delay(10);
}