#ifndef __HID_PAGE_SENSOR_HPP_
#define __HID_PAGE_SENSOR_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class sensor : std::uint16_t;
template <> constexpr inline auto get_info<sensor>() {
  return info(
      0x0020, 0xf9c3, "Sensors",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "Sensor";
        case 0x0010:
          return "Biometric";
        case 0x0011:
          return "Biometric: Human Presence";
        case 0x0012:
          return "Biometric: Human Proximity";
        case 0x0013:
          return "Biometric: Human Touch";
        case 0x0014:
          return "Biometric: Blood Pressure";
        case 0x0015:
          return "Biometric: Body Temperature";
        case 0x0016:
          return "Biometric: Heart Rate";
        case 0x0017:
          return "Biometric: Heart Rate Variability";
        case 0x0018:
          return "Biometric: Peripheral Oxygen Saturation";
        case 0x0019:
          return "Biometric: Respiratory Rate";
        case 0x0020:
          return "Electrical";
        case 0x0021:
          return "Electrical: Capacitance";
        case 0x0022:
          return "Electrical: Current";
        case 0x0023:
          return "Electrical: Power";
        case 0x0024:
          return "Electrical: Inductance";
        case 0x0025:
          return "Electrical: Resistance";
        case 0x0026:
          return "Electrical: Voltage";
        case 0x0027:
          return "Electrical: Potentiometer";
        case 0x0028:
          return "Electrical: Frequency";
        case 0x0029:
          return "Electrical: Period";
        case 0x0030:
          return "Environmental";
        case 0x0031:
          return "Environmental: Atmospheric Pressure";
        case 0x0032:
          return "Environmental: Humidity";
        case 0x0033:
          return "Environmental: Temperature";
        case 0x0034:
          return "Environmental: Wind Direction";
        case 0x0035:
          return "Environmental: Wind Speed";
        case 0x0036:
          return "Environmental: Air Quality";
        case 0x0037:
          return "Environmental: Heat Index";
        case 0x0038:
          return "Environmental: Surface Temperature";
        case 0x0039:
          return "Environmental: Volatile Organic Compounds";
        case 0x003a:
          return "Environmental: Object Presence";
        case 0x003b:
          return "Environmental: Object Proximity";
        case 0x0040:
          return "Light";
        case 0x0041:
          return "Light: Ambient Light";
        case 0x0042:
          return "Light: Consumer Infrared";
        case 0x0043:
          return "Light: Infrared Light";
        case 0x0044:
          return "Light: Visible Light";
        case 0x0045:
          return "Light: Ultraviolet Light";
        case 0x0050:
          return "Location";
        case 0x0051:
          return "Location: Broadcast";
        case 0x0052:
          return "Location: Dead Reckoning";
        case 0x0053:
          return "Location: GPS";
        case 0x0054:
          return "Location: Lookup";
        case 0x0055:
          return "Location: Other";
        case 0x0056:
          return "Location: Static";
        case 0x0057:
          return "Location: Triangulation";
        case 0x0060:
          return "Mechanical";
        case 0x0061:
          return "Mechanical: Boolean Switch";
        case 0x0062:
          return "Mechanical: Boolean Switch Array";
        case 0x0063:
          return "Mechanical: Multivalue Switch";
        case 0x0064:
          return "Mechanical: Force";
        case 0x0065:
          return "Mechanical: Pressure";
        case 0x0066:
          return "Mechanical: Strain";
        case 0x0067:
          return "Mechanical: Weight";
        case 0x0068:
          return "Mechanical: Haptic Vibrator";
        case 0x0069:
          return "Mechanical: Hall Effect Switch";
        case 0x0070:
          return "Motion";
        case 0x0071:
          return "Motion: Accelerometer 1D";
        case 0x0072:
          return "Motion: Accelerometer 2D";
        case 0x0073:
          return "Motion: Accelerometer 3D";
        case 0x0074:
          return "Motion: Gyrometer 1D";
        case 0x0075:
          return "Motion: Gyrometer 2D";
        case 0x0076:
          return "Motion: Gyrometer 3D";
        case 0x0077:
          return "Motion: Motion Detector";
        case 0x0078:
          return "Motion: Speedometer";
        case 0x0079:
          return "Motion: Accelerometer";
        case 0x007a:
          return "Motion: Gyrometer";
        case 0x007b:
          return "Motion: Gravity Vector";
        case 0x007c:
          return "Motion: Linear Accelerometer";
        case 0x0080:
          return "Orientation";
        case 0x0081:
          return "Orientation: Compass 1D";
        case 0x0082:
          return "Orientation: Compass 2D";
        case 0x0083:
          return "Orientation: Compass 3D";
        case 0x0084:
          return "Orientation: Inclinometer 1D";
        case 0x0085:
          return "Orientation: Inclinometer 2D";
        case 0x0086:
          return "Orientation: Inclinometer 3D";
        case 0x0087:
          return "Orientation: Distance 1D";
        case 0x0088:
          return "Orientation: Distance 2D";
        case 0x0089:
          return "Orientation: Distance 3D";
        case 0x008a:
          return "Orientation: Device Orientation";
        case 0x008b:
          return "Orientation: Compass";
        case 0x008c:
          return "Orientation: Inclinometer";
        case 0x008d:
          return "Orientation: Distance";
        case 0x008e:
          return "Orientation: Relative Orientation";
        case 0x008f:
          return "Orientation: Simple Orientation";
        case 0x0090:
          return "Scanner";
        case 0x0091:
          return "Scanner: Barcode";
        case 0x0092:
          return "Scanner: RFID";
        case 0x0093:
          return "Scanner: NFC";
        case 0x00a0:
          return "Time";
        case 0x00a1:
          return "Time: Alarm Timer";
        case 0x00a2:
          return "Time: Real Time Clock";
        case 0x00b0:
          return "Personal Activity";
        case 0x00b1:
          return "Personal Activity: Activity Detection";
        case 0x00b2:
          return "Personal Activity: Device Position";
        case 0x00b3:
          return "Personal Activity: Floor Tracker";
        case 0x00b4:
          return "Personal Activity: Pedometer";
        case 0x00b5:
          return "Personal Activity: Step Detection";
        case 0x00c0:
          return "Orientation Extended";
        case 0x00c1:
          return "Orientation Extended: Geomagnetic Orientation";
        case 0x00c2:
          return "Orientation Extended: Magnetometer";
        case 0x00d0:
          return "Gesture";
        case 0x00d1:
          return "Gesture: Chassis Flip Gesture";
        case 0x00d2:
          return "Gesture: Hinge Fold Gesture";
        case 0x00e0:
          return "Other";
        case 0x00e1:
          return "Other: Custom";
        case 0x00e2:
          return "Other: Generic";
        case 0x00e3:
          return "Other: Generic Enumerator";
        case 0x00e4:
          return "Other: Hinge Angle";
        case 0x00f0:
          return "Vendor Reserved 1";
        case 0x00f1:
          return "Vendor Reserved 2";
        case 0x00f2:
          return "Vendor Reserved 3";
        case 0x00f3:
          return "Vendor Reserved 4";
        case 0x00f4:
          return "Vendor Reserved 5";
        case 0x00f5:
          return "Vendor Reserved 6";
        case 0x00f6:
          return "Vendor Reserved 7";
        case 0x00f7:
          return "Vendor Reserved 8";
        case 0x00f8:
          return "Vendor Reserved 9";
        case 0x00f9:
          return "Vendor Reserved 10";
        case 0x00fa:
          return "Vendor Reserved 11";
        case 0x00fb:
          return "Vendor Reserved 12";
        case 0x00fc:
          return "Vendor Reserved 13";
        case 0x00fd:
          return "Vendor Reserved 14";
        case 0x00fe:
          return "Vendor Reserved 15";
        case 0x00ff:
          return "Vendor Reserved 16";
        case 0x0200:
          return "Event";
        case 0x0201:
          return "Event: Sensor State";
        case 0x0202:
          return "Event: Sensor Event";
        case 0x0300:
          return "Property";
        case 0x0301:
          return "Property: Friendly Name";
        case 0x0302:
          return "Property: Persistent Unique ID";
        case 0x0303:
          return "Property: Sensor Status";
        case 0x0304:
          return "Property: Minimum Report Interval";
        case 0x0305:
          return "Property: Sensor Manufacturer";
        case 0x0306:
          return "Property: Sensor Model";
        case 0x0307:
          return "Property: Sensor Serial Number";
        case 0x0308:
          return "Property: Sensor Description";
        case 0x0309:
          return "Property: Sensor Connection Type";
        case 0x030a:
          return "Property: Sensor Device Path";
        case 0x030b:
          return "Property: Hardware Revision";
        case 0x030c:
          return "Property: Firmware Version";
        case 0x030d:
          return "Property: Release Date";
        case 0x030e:
          return "Property: Report Interval";
        case 0x030f:
          return "Property: Change Sensitivity Absolute";
        case 0x0310:
          return "Property: Change Sensitivity Percent of Range";
        case 0x0311:
          return "Property: Change Sensitivity Percent Relative";
        case 0x0312:
          return "Property: Accuracy";
        case 0x0313:
          return "Property: Resolution";
        case 0x0314:
          return "Property: Maximum";
        case 0x0315:
          return "Property: Minimum";
        case 0x0316:
          return "Property: Reporting State";
        case 0x0317:
          return "Property: Sampling Rate";
        case 0x0318:
          return "Property: Response Curve";
        case 0x0319:
          return "Property: Power State";
        case 0x031a:
          return "Property: Maximum FIFO Events";
        case 0x031b:
          return "Property: Report Latency";
        case 0x031c:
          return "Property: Flush FIFO Events";
        case 0x031d:
          return "Property: Maximum Power Consumption";
        case 0x031e:
          return "Property: Is Primary";
        case 0x031f:
          return "Property: Human Presence Detection Type";
        case 0x0400:
          return "Data Field: Location";
        case 0x0402:
          return "Data Field: Altitude Antenna Sea Level";
        case 0x0403:
          return "Data Field: Differential Reference Station ID";
        case 0x0404:
          return "Data Field: Altitude Ellipsoid Error";
        case 0x0405:
          return "Data Field: Altitude Ellipsoid";
        case 0x0406:
          return "Data Field: Altitude Sea Level Error";
        case 0x0407:
          return "Data Field: Altitude Sea Level";
        case 0x0408:
          return "Data Field: Differential GPS Data Age";
        case 0x0409:
          return "Data Field: Error Radius";
        case 0x040a:
          return "Data Field: Fix Quality";
        case 0x040b:
          return "Data Field: Fix Type";
        case 0x040c:
          return "Data Field: Geoidal Separation";
        case 0x040d:
          return "Data Field: GPS Operation Mode";
        case 0x040e:
          return "Data Field: GPS Selection Mode";
        case 0x040f:
          return "Data Field: GPS Status";
        case 0x0410:
          return "Data Field: Position Dilution of Precision";
        case 0x0411:
          return "Data Field: Horizontal Dilution of Precision";
        case 0x0412:
          return "Data Field: Vertical Dilution of Precision";
        case 0x0413:
          return "Data Field: Latitude";
        case 0x0414:
          return "Data Field: Longitude";
        case 0x0415:
          return "Data Field: True Heading";
        case 0x0416:
          return "Data Field: Magnetic Heading";
        case 0x0417:
          return "Data Field: Magnetic Variation";
        case 0x0418:
          return "Data Field: Speed";
        case 0x0419:
          return "Data Field: Satellites in View";
        case 0x041a:
          return "Data Field: Satellites in View Azimuth";
        case 0x041b:
          return "Data Field: Satellites in View Elevation";
        case 0x041c:
          return "Data Field: Satellites in View IDs";
        case 0x041d:
          return "Data Field: Satellites in View PRNs";
        case 0x041e:
          return "Data Field: Satellites in View S/N Ratios";
        case 0x041f:
          return "Data Field: Satellites Used Count";
        case 0x0420:
          return "Data Field: Satellites Used PRNs";
        case 0x0421:
          return "Data Field: NMEA Sentence";
        case 0x0422:
          return "Data Field: Address Line 1";
        case 0x0423:
          return "Data Field: Address Line 2";
        case 0x0424:
          return "Data Field: City";
        case 0x0425:
          return "Data Field: State or Province";
        case 0x0426:
          return "Data Field: Country or Region";
        case 0x0427:
          return "Data Field: Postal Code";
        case 0x042a:
          return "Property: Location";
        case 0x042b:
          return "Property: Location Desired Accuracy";
        case 0x0430:
          return "Data Field: Environmental";
        case 0x0431:
          return "Data Field: Atmospheric Pressure";
        case 0x0433:
          return "Data Field: Relative Humidity";
        case 0x0434:
          return "Data Field: Temperature";
        case 0x0435:
          return "Data Field: Wind Direction";
        case 0x0436:
          return "Data Field: Wind Speed";
        case 0x0437:
          return "Data Field: Air Quality Index";
        case 0x0438:
          return "Data Field: Equivalent CO2";
        case 0x0439:
          return "Data Field: Volatile Organic Compound Concentration";
        case 0x043a:
          return "Data Field: Object Presence";
        case 0x043b:
          return "Data Field: Object Proximity Range";
        case 0x043c:
          return "Data Field: Object Proximity Out of Range";
        case 0x0440:
          return "Property: Environmental";
        case 0x0441:
          return "Property: Reference Pressure";
        case 0x0450:
          return "Data Field: Motion";
        case 0x0451:
          return "Data Field: Motion State";
        case 0x0452:
          return "Data Field: Acceleration";
        case 0x0453:
          return "Data Field: Acceleration Axis X";
        case 0x0454:
          return "Data Field: Acceleration Axis Y";
        case 0x0455:
          return "Data Field: Acceleration Axis Z";
        case 0x0456:
          return "Data Field: Angular Velocity";
        case 0x0457:
          return "Data Field: Angular Velocity about X Axis";
        case 0x0458:
          return "Data Field: Angular Velocity about Y Axis";
        case 0x0459:
          return "Data Field: Angular Velocity about Z Axis";
        case 0x045a:
          return "Data Field: Angular Position";
        case 0x045b:
          return "Data Field: Angular Position about X Axis";
        case 0x045c:
          return "Data Field: Angular Position about Y Axis";
        case 0x045d:
          return "Data Field: Angular Position about Z Axis";
        case 0x045e:
          return "Data Field: Motion Speed";
        case 0x045f:
          return "Data Field: Motion Intensity";
        case 0x0470:
          return "Data Field: Orientation";
        case 0x0471:
          return "Data Field: Heading";
        case 0x0472:
          return "Data Field: Heading X Axis";
        case 0x0473:
          return "Data Field: Heading Y Axis";
        case 0x0474:
          return "Data Field: Heading Z Axis";
        case 0x0475:
          return "Data Field: Heading Compensated Magnetic North";
        case 0x0476:
          return "Data Field: Heading Compensated True North";
        case 0x0477:
          return "Data Field: Heading Magnetic North";
        case 0x0478:
          return "Data Field: Heading True North";
        case 0x0479:
          return "Data Field: Distance";
        case 0x047a:
          return "Data Field: Distance X Axis";
        case 0x047b:
          return "Data Field: Distance Y Axis";
        case 0x047c:
          return "Data Field: Distance Z Axis";
        case 0x047d:
          return "Data Field: Distance Out-of-Range";
        case 0x047e:
          return "Data Field: Tilt";
        case 0x047f:
          return "Data Field: Tilt X Axis";
        case 0x0480:
          return "Data Field: Tilt Y Axis";
        case 0x0481:
          return "Data Field: Tilt Z Axis";
        case 0x0482:
          return "Data Field: Rotation Matrix";
        case 0x0483:
          return "Data Field: Quaternion";
        case 0x0484:
          return "Data Field: Magnetic Flux";
        case 0x0485:
          return "Data Field: Magnetic Flux X Axis";
        case 0x0486:
          return "Data Field: Magnetic Flux Y Axis";
        case 0x0487:
          return "Data Field: Magnetic Flux Z Axis";
        case 0x0488:
          return "Data Field: Magnetometer Accuracy";
        case 0x0489:
          return "Data Field: Simple Orientation Direction";
        case 0x0490:
          return "Data Field: Mechanical";
        case 0x0491:
          return "Data Field: Boolean Switch State";
        case 0x0492:
          return "Data Field: Boolean Switch Array States";
        case 0x0493:
          return "Data Field: Multivalue Switch Value";
        case 0x0494:
          return "Data Field: Force";
        case 0x0495:
          return "Data Field: Absolute Pressure";
        case 0x0496:
          return "Data Field: Gauge Pressure";
        case 0x0497:
          return "Data Field: Strain";
        case 0x0498:
          return "Data Field: Weight";
        case 0x04a0:
          return "Property: Mechanical";
        case 0x04a1:
          return "Property: Vibration State";
        case 0x04a2:
          return "Property: Forward Vibration Speed";
        case 0x04a3:
          return "Property: Backward Vibration Speed";
        case 0x04b0:
          return "Data Field: Biometric";
        case 0x04b1:
          return "Data Field: Human Presence";
        case 0x04b2:
          return "Data Field: Human Proximity Range";
        case 0x04b3:
          return "Data Field: Human Proximity Out of Range";
        case 0x04b4:
          return "Data Field: Human Touch State";
        case 0x04b5:
          return "Data Field: Blood Pressure";
        case 0x04b6:
          return "Data Field: Blood Pressure Diastolic";
        case 0x04b7:
          return "Data Field: Blood Pressure Systolic";
        case 0x04b8:
          return "Data Field: Heart Rate";
        case 0x04b9:
          return "Data Field: Resting Heart Rate";
        case 0x04ba:
          return "Data Field: Heartbeat Interval";
        case 0x04bb:
          return "Data Field: Respiratory Rate";
        case 0x04bc:
          return "Data Field: SpO2";
        case 0x04bd:
          return "Data Field: Human Attention Detected";
        case 0x04be:
          return "Data Field: Human Head Azimuth";
        case 0x04bf:
          return "Data Field: Human Head Altitude";
        case 0x04c0:
          return "Data Field: Human Head Roll";
        case 0x04c1:
          return "Data Field: Human Head Pitch";
        case 0x04c2:
          return "Data Field: Human Head Yaw";
        case 0x04c3:
          return "Data Field: Human Correlation Id";
        case 0x04d0:
          return "Data Field: Light";
        case 0x04d1:
          return "Data Field: Illuminance";
        case 0x04d2:
          return "Data Field: Color Temperature";
        case 0x04d3:
          return "Data Field: Chromaticity";
        case 0x04d4:
          return "Data Field: Chromaticity X";
        case 0x04d5:
          return "Data Field: Chromaticity Y";
        case 0x04d6:
          return "Data Field: Consumer IR Sentence Receive";
        case 0x04d7:
          return "Data Field: Infrared Light";
        case 0x04d8:
          return "Data Field: Red Light";
        case 0x04d9:
          return "Data Field: Green Light";
        case 0x04da:
          return "Data Field: Blue Light";
        case 0x04db:
          return "Data Field: Ultraviolet A Light";
        case 0x04dc:
          return "Data Field: Ultraviolet B Light";
        case 0x04dd:
          return "Data Field: Ultraviolet Index";
        case 0x04de:
          return "Data Field: Near Infrared Light";
        case 0x04df:
          return "Property: Light";
        case 0x04e0:
          return "Property: Consumer IR Sentence Send";
        case 0x04e2:
          return "Property: Auto Brightness Preferred";
        case 0x04e3:
          return "Property: Auto Color Preferred";
        case 0x04f0:
          return "Data Field: Scanner";
        case 0x04f1:
          return "Data Field: RFID Tag 40 Bit";
        case 0x04f2:
          return "Data Field: NFC Sentence Receive";
        case 0x04f8:
          return "Property: Scanner";
        case 0x04f9:
          return "Property: NFC Sentence Send";
        case 0x0500:
          return "Data Field: Electrical";
        case 0x0501:
          return "Data Field: Capacitance";
        case 0x0502:
          return "Data Field: Current";
        case 0x0503:
          return "Data Field: Electrical Power";
        case 0x0504:
          return "Data Field: Inductance";
        case 0x0505:
          return "Data Field: Resistance";
        case 0x0506:
          return "Data Field: Voltage";
        case 0x0507:
          return "Data Field: Frequency";
        case 0x0508:
          return "Data Field: Period";
        case 0x0509:
          return "Data Field: Percent of Range";
        case 0x0520:
          return "Data Field: Time";
        case 0x0521:
          return "Data Field: Year";
        case 0x0522:
          return "Data Field: Month";
        case 0x0523:
          return "Data Field: Day";
        case 0x0524:
          return "Data Field: Day of Week";
        case 0x0525:
          return "Data Field: Hour";
        case 0x0526:
          return "Data Field: Minute";
        case 0x0527:
          return "Data Field: Second";
        case 0x0528:
          return "Data Field: Millisecond";
        case 0x0529:
          return "Data Field: Timestamp";
        case 0x052a:
          return "Data Field: Julian Day of Year";
        case 0x052b:
          return "Data Field: Time Since System Boot";
        case 0x0530:
          return "Property: Time";
        case 0x0531:
          return "Property: Time Zone Offset from UTC";
        case 0x0532:
          return "Property: Time Zone Name";
        case 0x0533:
          return "Property: Daylight Savings Time Observed";
        case 0x0534:
          return "Property: Time Trim Adjustment";
        case 0x0535:
          return "Property: Arm Alarm";
        case 0x0540:
          return "Data Field: Custom";
        case 0x0541:
          return "Data Field: Custom Usage";
        case 0x0542:
          return "Data Field: Custom Boolean Array";
        case 0x0543:
          return "Data Field: Custom Value";
        case 0x0544:
          return "Data Field: Custom Value 1";
        case 0x0545:
          return "Data Field: Custom Value 2";
        case 0x0546:
          return "Data Field: Custom Value 3";
        case 0x0547:
          return "Data Field: Custom Value 4";
        case 0x0548:
          return "Data Field: Custom Value 5";
        case 0x0549:
          return "Data Field: Custom Value 6";
        case 0x054a:
          return "Data Field: Custom Value 7";
        case 0x054b:
          return "Data Field: Custom Value 8";
        case 0x054c:
          return "Data Field: Custom Value 9";
        case 0x054d:
          return "Data Field: Custom Value 10";
        case 0x054e:
          return "Data Field: Custom Value 11";
        case 0x054f:
          return "Data Field: Custom Value 12";
        case 0x0550:
          return "Data Field: Custom Value 13";
        case 0x0551:
          return "Data Field: Custom Value 14";
        case 0x0552:
          return "Data Field: Custom Value 15";
        case 0x0553:
          return "Data Field: Custom Value 16";
        case 0x0554:
          return "Data Field: Custom Value 17";
        case 0x0555:
          return "Data Field: Custom Value 18";
        case 0x0556:
          return "Data Field: Custom Value 19";
        case 0x0557:
          return "Data Field: Custom Value 20";
        case 0x0558:
          return "Data Field: Custom Value 21";
        case 0x0559:
          return "Data Field: Custom Value 22";
        case 0x055a:
          return "Data Field: Custom Value 23";
        case 0x055b:
          return "Data Field: Custom Value 24";
        case 0x055c:
          return "Data Field: Custom Value 25";
        case 0x055d:
          return "Data Field: Custom Value 26";
        case 0x055e:
          return "Data Field: Custom Value 27";
        case 0x055f:
          return "Data Field: Custom Value 28";
        case 0x0560:
          return "Data Field: Generic";
        case 0x0561:
          return "Data Field: Generic GUID or PROPERTYKEY";
        case 0x0562:
          return "Data Field: Generic Category GUID";
        case 0x0563:
          return "Data Field: Generic Type GUID";
        case 0x0564:
          return "Data Field: Generic Event PROPERTYKEY";
        case 0x0565:
          return "Data Field: Generic Property PROPERTYKEY";
        case 0x0566:
          return "Data Field: Generic Data Field PROPERTYKEY";
        case 0x0567:
          return "Data Field: Generic Event";
        case 0x0568:
          return "Data Field: Generic Property";
        case 0x0569:
          return "Data Field: Generic Data Field";
        case 0x056a:
          return "Data Field: Enumerator Table Row Index";
        case 0x056b:
          return "Data Field: Enumerator Table Row Count";
        case 0x056c:
          return "Data Field: Generic GUID or PROPERTYKEY kind";
        case 0x056d:
          return "Data Field: Generic GUID";
        case 0x056e:
          return "Data Field: Generic PROPERTYKEY";
        case 0x056f:
          return "Data Field: Generic Top Level Collection ID";
        case 0x0570:
          return "Data Field: Generic Report ID";
        case 0x0571:
          return "Data Field: Generic Report Item Position Index";
        case 0x0572:
          return "Data Field: Generic Firmware VARTYPE";
        case 0x0573:
          return "Data Field: Generic Unit of Measure";
        case 0x0574:
          return "Data Field: Generic Unit Exponent";
        case 0x0575:
          return "Data Field: Generic Report Size";
        case 0x0576:
          return "Data Field: Generic Report Count";
        case 0x0580:
          return "Property: Generic";
        case 0x0581:
          return "Property: Enumerator Table Row Index";
        case 0x0582:
          return "Property: Enumerator Table Row Count";
        case 0x0590:
          return "Data Field: Personal Activity";
        case 0x0591:
          return "Data Field: Activity Type";
        case 0x0592:
          return "Data Field: Activity State";
        case 0x0593:
          return "Data Field: Device Position";
        case 0x0594:
          return "Data Field: Step Count";
        case 0x0595:
          return "Data Field: Step Count Reset";
        case 0x0596:
          return "Data Field: Step Duration";
        case 0x0597:
          return "Data Field: Step Type";
        case 0x05a0:
          return "Property: Minimum Activity Detection Interval";
        case 0x05a1:
          return "Property: Supported Activity Types";
        case 0x05a2:
          return "Property: Subscribed Activity Types";
        case 0x05a3:
          return "Property: Supported Step Types";
        case 0x05a4:
          return "Property: Subscribed Step Types";
        case 0x05a5:
          return "Property: Floor Height";
        case 0x05b0:
          return "Data Field: Custom Type ID";
        case 0x05c0:
          return "Property: Custom";
        case 0x05c1:
          return "Property: Custom Value 1";
        case 0x05c2:
          return "Property: Custom Value 2";
        case 0x05c3:
          return "Property: Custom Value 3";
        case 0x05c4:
          return "Property: Custom Value 4";
        case 0x05c5:
          return "Property: Custom Value 5";
        case 0x05c6:
          return "Property: Custom Value 6";
        case 0x05c7:
          return "Property: Custom Value 7";
        case 0x05c8:
          return "Property: Custom Value 8";
        case 0x05c9:
          return "Property: Custom Value 9";
        case 0x05ca:
          return "Property: Custom Value 10";
        case 0x05cb:
          return "Property: Custom Value 11";
        case 0x05cc:
          return "Property: Custom Value 12";
        case 0x05cd:
          return "Property: Custom Value 13";
        case 0x05ce:
          return "Property: Custom Value 14";
        case 0x05cf:
          return "Property: Custom Value 15";
        case 0x05d0:
          return "Property: Custom Value 16";
        case 0x05e0:
          return "Data Field: Hinge";
        case 0x05e1:
          return "Data Field: Hinge Angle";
        case 0x05f0:
          return "Data Field: Gesture Sensor";
        case 0x05f1:
          return "Data Field: Gesture State";
        case 0x05f2:
          return "Data Field: Hinge Fold Initial Angle";
        case 0x05f3:
          return "Data Field: Hinge Fold Final Angle";
        case 0x05f4:
          return "Data Field: Hinge Fold Contributing Panel";
        case 0x05f5:
          return "Data Field: Hinge Fold Type";
        case 0x0800:
          return "Sensor State: Undefined";
        case 0x0801:
          return "Sensor State: Ready";
        case 0x0802:
          return "Sensor State: Not Available";
        case 0x0803:
          return "Sensor State: No Data";
        case 0x0804:
          return "Sensor State: Initializing";
        case 0x0805:
          return "Sensor State: Access Denied";
        case 0x0806:
          return "Sensor State: Error";
        case 0x0810:
          return "Sensor Event: Unknown";
        case 0x0811:
          return "Sensor Event: State Changed";
        case 0x0812:
          return "Sensor Event: Property Changed";
        case 0x0813:
          return "Sensor Event: Data Updated";
        case 0x0814:
          return "Sensor Event: Poll Response";
        case 0x0815:
          return "Sensor Event: Change Sensitivity";
        case 0x0816:
          return "Sensor Event: Range Maximum Reached";
        case 0x0817:
          return "Sensor Event: Range Minimum Reached";
        case 0x0818:
          return "Sensor Event: High Threshold Cross Upward";
        case 0x0819:
          return "Sensor Event: High Threshold Cross Downward";
        case 0x081a:
          return "Sensor Event: Low Threshold Cross Upward";
        case 0x081b:
          return "Sensor Event: Low Threshold Cross Downward";
        case 0x081c:
          return "Sensor Event: Zero Threshold Cross Upward";
        case 0x081d:
          return "Sensor Event: Zero Threshold Cross Downward";
        case 0x081e:
          return "Sensor Event: Period Exceeded";
        case 0x081f:
          return "Sensor Event: Frequency Exceeded";
        case 0x0820:
          return "Sensor Event: Complex Trigger";
        case 0x0830:
          return "Connection Type: PC Integrated";
        case 0x0831:
          return "Connection Type: PC Attached";
        case 0x0832:
          return "Connection Type: PC External";
        case 0x0840:
          return "Reporting State: Report No Events";
        case 0x0841:
          return "Reporting State: Report All Events";
        case 0x0842:
          return "Reporting State: Report Threshold Events";
        case 0x0843:
          return "Reporting State: Wake On No Events";
        case 0x0844:
          return "Reporting State: Wake On All Events";
        case 0x0845:
          return "Reporting State: Wake On Threshold Events";
        case 0x0846:
          return "Reporting State: Anytime";
        case 0x0850:
          return "Power State: Undefined";
        case 0x0851:
          return "Power State: D0 Full Power";
        case 0x0852:
          return "Power State: D1 Low Power";
        case 0x0853:
          return "Power State: D2 Standby Power with Wakeup";
        case 0x0854:
          return "Power State: D3 Sleep with Wakeup";
        case 0x0855:
          return "Power State: D4 Power Off";
        case 0x0860:
          return "Accuracy: Default";
        case 0x0861:
          return "Accuracy: High";
        case 0x0862:
          return "Accuracy: Medium";
        case 0x0863:
          return "Accuracy: Low";
        case 0x0870:
          return "Fix Quality: No Fix";
        case 0x0871:
          return "Fix Quality: GPS";
        case 0x0872:
          return "Fix Quality: DGPS";
        case 0x0880:
          return "Fix Type: No Fix";
        case 0x0881:
          return "Fix Type: GPS SPS Mode, Fix Valid";
        case 0x0882:
          return "Fix Type: DGPS SPS Mode, Fix Valid";
        case 0x0883:
          return "Fix Type: GPS PPS Mode, Fix Valid";
        case 0x0884:
          return "Fix Type: Real Time Kinematic";
        case 0x0885:
          return "Fix Type: Float RTK";
        case 0x0886:
          return "Fix Type: Estimated (dead reckoned)";
        case 0x0887:
          return "Fix Type: Manual Input Mode";
        case 0x0888:
          return "Fix Type: Simulator Mode";
        case 0x0890:
          return "GPS Operation Mode: Manual";
        case 0x0891:
          return "GPS Operation Mode: Automatic";
        case 0x08a0:
          return "GPS Selection Mode: Autonomous";
        case 0x08a1:
          return "GPS Selection Mode: DGPS";
        case 0x08a2:
          return "GPS Selection Mode: Estimated (dead reckoned)";
        case 0x08a3:
          return "GPS Selection Mode: Manual Input";
        case 0x08a4:
          return "GPS Selection Mode: Simulator";
        case 0x08a5:
          return "GPS Selection Mode: Data Not Valid";
        case 0x08b0:
          return "GPS Status Data: Valid";
        case 0x08b1:
          return "GPS Status Data: Not Valid";
        case 0x08c0:
          return "Day of Week: Sunday";
        case 0x08c1:
          return "Day of Week: Monday";
        case 0x08c2:
          return "Day of Week: Tuesday";
        case 0x08c3:
          return "Day of Week: Wednesday";
        case 0x08c4:
          return "Day of Week: Thursday";
        case 0x08c5:
          return "Day of Week: Friday";
        case 0x08c6:
          return "Day of Week: Saturday";
        case 0x08d0:
          return "Kind: Category";
        case 0x08d1:
          return "Kind: Type";
        case 0x08d2:
          return "Kind: Event";
        case 0x08d3:
          return "Kind: Property";
        case 0x08d4:
          return "Kind: Data Field";
        case 0x08e0:
          return "Magnetometer Accuracy: Low";
        case 0x08e1:
          return "Magnetometer Accuracy: Medium";
        case 0x08e2:
          return "Magnetometer Accuracy: High";
        case 0x08f0:
          return "Simple Orientation Direction: Not Rotated";
        case 0x08f1:
          return "Simple Orientation Direction: Rotated 90 Degrees CCW";
        case 0x08f2:
          return "Simple Orientation Direction: Rotated 180 Degrees CCW";
        case 0x08f3:
          return "Simple Orientation Direction: Rotated 270 Degrees CCW";
        case 0x08f4:
          return "Simple Orientation Direction: Face Up";
        case 0x08f5:
          return "Simple Orientation Direction: Face Down";
        case 0x0900:
          return "VT_NULL: Empty";
        case 0x0901:
          return "VT_BOOL: Boolean";
        case 0x0902:
          return "VT_UI1: Byte";
        case 0x0903:
          return "VT_I1: Character";
        case 0x0904:
          return "VT_UI2: Unsigned Short";
        case 0x0905:
          return "VT_I2: Short";
        case 0x0906:
          return "VT_UI4: Unsigned Long";
        case 0x0907:
          return "VT_I4: Long";
        case 0x0908:
          return "VT_UI8: Unsigned Long Long";
        case 0x0909:
          return "VT_I8: Long Long";
        case 0x090a:
          return "VT_R4: Float";
        case 0x090b:
          return "VT_R8: Double";
        case 0x090c:
          return "VT_WSTR: Wide String";
        case 0x090d:
          return "VT_STR: Narrow String";
        case 0x090e:
          return "VT_CLSID: Guid";
        case 0x090f:
          return "VT_VECTOR|VT_UI1: Opaque Structure";
        case 0x0910:
          return "VT_F16E0: HID 16-bit Float e0";
        case 0x0911:
          return "VT_F16E1: HID 16-bit Float e1";
        case 0x0912:
          return "VT_F16E2: HID 16-bit Float e2";
        case 0x0913:
          return "VT_F16E3: HID 16-bit Float e3";
        case 0x0914:
          return "VT_F16E4: HID 16-bit Float e4";
        case 0x0915:
          return "VT_F16E5: HID 16-bit Float e5";
        case 0x0916:
          return "VT_F16E6: HID 16-bit Float e6";
        case 0x0917:
          return "VT_F16E7: HID 16-bit Float e7";
        case 0x0918:
          return "VT_F16E8: HID 16-bit Float e-8";
        case 0x0919:
          return "VT_F16E9: HID 16-bit Float e-7";
        case 0x091a:
          return "VT_F16EA: HID 16-bit Float e-6";
        case 0x091b:
          return "VT_F16EB: HID 16-bit Float e-5";
        case 0x091c:
          return "VT_F16EC: HID 16-bit Float e-4";
        case 0x091d:
          return "VT_F16ED: HID 16-bit Float e-3";
        case 0x091e:
          return "VT_F16EE: HID 16-bit Float e-2";
        case 0x091f:
          return "VT_F16EF: HID 16-bit Float e-1";
        case 0x0920:
          return "VT_F32E0: HID 32-bit Float e0";
        case 0x0921:
          return "VT_F32E1: HID 32-bit Float e1";
        case 0x0922:
          return "VT_F32E2: HID 32-bit Float e2";
        case 0x0923:
          return "VT_F32E3: HID 32-bit Float e3";
        case 0x0924:
          return "VT_F32E4: HID 32-bit Float e4";
        case 0x0925:
          return "VT_F32E5: HID 32-bit Float e5";
        case 0x0926:
          return "VT_F32E6: HID 32-bit Float e6";
        case 0x0927:
          return "VT_F32E7: HID 32-bit Float e7";
        case 0x0928:
          return "VT_F32E8: HID 32-bit Float e-8";
        case 0x0929:
          return "VT_F32E9: HID 32-bit Float e-7";
        case 0x092a:
          return "VT_F32EA: HID 32-bit Float e-6";
        case 0x092b:
          return "VT_F32EB: HID 32-bit Float e-5";
        case 0x092c:
          return "VT_F32EC: HID 32-bit Float e-4";
        case 0x092d:
          return "VT_F32ED: HID 32-bit Float e-3";
        case 0x092e:
          return "VT_F32EE: HID 32-bit Float e-2";
        case 0x092f:
          return "VT_F32EF: HID 32-bit Float e-1";
        case 0x0930:
          return "Activity Type: Unknown";
        case 0x0931:
          return "Activity Type: Stationary";
        case 0x0932:
          return "Activity Type: Fidgeting";
        case 0x0933:
          return "Activity Type: Walking";
        case 0x0934:
          return "Activity Type: Running";
        case 0x0935:
          return "Activity Type: In Vehicle";
        case 0x0936:
          return "Activity Type: Biking";
        case 0x0937:
          return "Activity Type: Idle";
        case 0x0940:
          return "Unit: Not Specified";
        case 0x0941:
          return "Unit: Lux";
        case 0x0942:
          return "Unit: Degrees Kelvin";
        case 0x0943:
          return "Unit: Degrees Celsius";
        case 0x0944:
          return "Unit: Pascal";
        case 0x0945:
          return "Unit: Newton";
        case 0x0946:
          return "Unit: Meters/Second";
        case 0x0947:
          return "Unit: Kilogram";
        case 0x0948:
          return "Unit: Meter";
        case 0x0949:
          return "Unit: Meters/Second/Second";
        case 0x094a:
          return "Unit: Farad";
        case 0x094b:
          return "Unit: Ampere";
        case 0x094c:
          return "Unit: Watt";
        case 0x094d:
          return "Unit: Henry";
        case 0x094e:
          return "Unit: Ohm";
        case 0x094f:
          return "Unit: Volt";
        case 0x0950:
          return "Unit: Hertz";
        case 0x0951:
          return "Unit: Bar";
        case 0x0952:
          return "Unit: Degrees Anti-clockwise";
        case 0x0953:
          return "Unit: Degrees Clockwise";
        case 0x0954:
          return "Unit: Degrees";
        case 0x0955:
          return "Unit: Degrees/Second";
        case 0x0956:
          return "Unit: Degrees/Second/Second";
        case 0x0957:
          return "Unit: Knot";
        case 0x0958:
          return "Unit: Percent";
        case 0x0959:
          return "Unit: Second";
        case 0x095a:
          return "Unit: Millisecond";
        case 0x095b:
          return "Unit: G";
        case 0x095c:
          return "Unit: Bytes";
        case 0x095d:
          return "Unit: Milligauss";
        case 0x095e:
          return "Unit: Bits";
        case 0x0960:
          return "Activity State: No State Change";
        case 0x0961:
          return "Activity State: Start Activity";
        case 0x0962:
          return "Activity State: End Activity";
        case 0x0970:
          return "Exponent 0: e0";
        case 0x0971:
          return "Exponent 1: e1";
        case 0x0972:
          return "Exponent 2: e2";
        case 0x0973:
          return "Exponent 3: e3";
        case 0x0974:
          return "Exponent 4: e4";
        case 0x0975:
          return "Exponent 5: e5";
        case 0x0976:
          return "Exponent 6: e6";
        case 0x0977:
          return "Exponent 7: e7";
        case 0x0978:
          return "Exponent 8: e-8";
        case 0x0979:
          return "Exponent 9: e-7";
        case 0x097a:
          return "Exponent A: e-6";
        case 0x097b:
          return "Exponent B: e-5";
        case 0x097c:
          return "Exponent C: e-4";
        case 0x097d:
          return "Exponent D: e-3";
        case 0x097e:
          return "Exponent E: e-2";
        case 0x097f:
          return "Exponent F: e-1";
        case 0x0980:
          return "Device Position: Unknown";
        case 0x0981:
          return "Device Position: Unchanged";
        case 0x0982:
          return "Device Position: On Desk";
        case 0x0983:
          return "Device Position: In Hand";
        case 0x0984:
          return "Device Position: Moving in Bag";
        case 0x0985:
          return "Device Position: Stationary in Bag";
        case 0x0990:
          return "Step Type: Unknown";
        case 0x0991:
          return "Step Type: Walking";
        case 0x0992:
          return "Step Type: Running";
        case 0x09a0:
          return "Gesture State: Unknown";
        case 0x09a1:
          return "Gesture State: Started";
        case 0x09a2:
          return "Gesture State: Completed";
        case 0x09a3:
          return "Gesture State: Cancelled";
        case 0x09b0:
          return "Hinge Fold Contributing Panel: Unknown";
        case 0x09b1:
          return "Hinge Fold Contributing Panel: Panel 1";
        case 0x09b2:
          return "Hinge Fold Contributing Panel: Panel 2";
        case 0x09b3:
          return "Hinge Fold Contributing Panel: Both";
        case 0x09b4:
          return "Hinge Fold Type: Unknown";
        case 0x09b5:
          return "Hinge Fold Type: Increasing";
        case 0x09b6:
          return "Hinge Fold Type: Decreasing";
        case 0x09c0:
          return "Human Presence Detection Type: Vendor-Defined Non-Biometric";
        case 0x09c1:
          return "Human Presence Detection Type: Vendor-Defined Biometric";
        case 0x09c2:
          return "Human Presence Detection Type: Facial Biometric";
        case 0x09c3:
          return "Human Presence Detection Type: Audio Biometric";
        case 0x1000:
          return "Change Sensitivity Absolute";
        case 0x2000:
          return "Maximum";
        case 0x3000:
          return "Minimum";
        case 0x4000:
          return "Accuracy";
        case 0x5000:
          return "Resolution";
        case 0x6000:
          return "Threshold High";
        case 0x7000:
          return "Threshold Low";
        case 0x8000:
          return "Calibration Offset";
        case 0x9000:
          return "Calibration Multiplier";
        case 0xa000:
          return "Report Interval";
        case 0xb000:
          return "Frequency Max";
        case 0xc000:
          return "Period Max";
        case 0xd000:
          return "Change Sensitivity Percent of Range";
        case 0xe000:
          return "Change Sensitivity Percent Relative";
        case 0xf000:
          return "Vendor Reserved";
        default:
          return (const char *)nullptr;
        }
      },
      0xf000);
}
enum class sensor : std::uint16_t {
  SENSOR = 0x0001,
  BIOMETRIC = 0x0010,
  BIOMETRIC_HUMAN_PRESENCE = 0x0011,
  BIOMETRIC_HUMAN_PROXIMITY = 0x0012,
  BIOMETRIC_HUMAN_TOUCH = 0x0013,
  BIOMETRIC_BLOOD_PRESSURE = 0x0014,
  BIOMETRIC_BODY_TEMPERATURE = 0x0015,
  BIOMETRIC_HEART_RATE = 0x0016,
  BIOMETRIC_HEART_RATE_VARIABILITY = 0x0017,
  BIOMETRIC_PERIPHERAL_OXYGEN_SATURATION = 0x0018,
  BIOMETRIC_RESPIRATORY_RATE = 0x0019,
  ELECTRICAL = 0x0020,
  ELECTRICAL_CAPACITANCE = 0x0021,
  ELECTRICAL_CURRENT = 0x0022,
  ELECTRICAL_POWER = 0x0023,
  ELECTRICAL_INDUCTANCE = 0x0024,
  ELECTRICAL_RESISTANCE = 0x0025,
  ELECTRICAL_VOLTAGE = 0x0026,
  ELECTRICAL_POTENTIOMETER = 0x0027,
  ELECTRICAL_FREQUENCY = 0x0028,
  ELECTRICAL_PERIOD = 0x0029,
  ENVIRONMENTAL = 0x0030,
  ENVIRONMENTAL_ATMOSPHERIC_PRESSURE = 0x0031,
  ENVIRONMENTAL_HUMIDITY = 0x0032,
  ENVIRONMENTAL_TEMPERATURE = 0x0033,
  ENVIRONMENTAL_WIND_DIRECTION = 0x0034,
  ENVIRONMENTAL_WIND_SPEED = 0x0035,
  ENVIRONMENTAL_AIR_QUALITY = 0x0036,
  ENVIRONMENTAL_HEAT_INDEX = 0x0037,
  ENVIRONMENTAL_SURFACE_TEMPERATURE = 0x0038,
  ENVIRONMENTAL_VOLATILE_ORGANIC_COMPOUNDS = 0x0039,
  ENVIRONMENTAL_OBJECT_PRESENCE = 0x003a,
  ENVIRONMENTAL_OBJECT_PROXIMITY = 0x003b,
  LIGHT = 0x0040,
  LIGHT_AMBIENT_LIGHT = 0x0041,
  LIGHT_CONSUMER_INFRARED = 0x0042,
  LIGHT_INFRARED_LIGHT = 0x0043,
  LIGHT_VISIBLE_LIGHT = 0x0044,
  LIGHT_ULTRAVIOLET_LIGHT = 0x0045,
  LOCATION = 0x0050,
  LOCATION_BROADCAST = 0x0051,
  LOCATION_DEAD_RECKONING = 0x0052,
  LOCATION_GPS = 0x0053,
  LOCATION_LOOKUP = 0x0054,
  LOCATION_OTHER = 0x0055,
  LOCATION_STATIC = 0x0056,
  LOCATION_TRIANGULATION = 0x0057,
  MECHANICAL = 0x0060,
  MECHANICAL_BOOLEAN_SWITCH = 0x0061,
  MECHANICAL_BOOLEAN_SWITCH_ARRAY = 0x0062,
  MECHANICAL_MULTIVALUE_SWITCH = 0x0063,
  MECHANICAL_FORCE = 0x0064,
  MECHANICAL_PRESSURE = 0x0065,
  MECHANICAL_STRAIN = 0x0066,
  MECHANICAL_WEIGHT = 0x0067,
  MECHANICAL_HAPTIC_VIBRATOR = 0x0068,
  MECHANICAL_HALL_EFFECT_SWITCH = 0x0069,
  MOTION = 0x0070,
  MOTION_ACCELEROMETER_1D = 0x0071,
  MOTION_ACCELEROMETER_2D = 0x0072,
  MOTION_ACCELEROMETER_3D = 0x0073,
  MOTION_GYROMETER_1D = 0x0074,
  MOTION_GYROMETER_2D = 0x0075,
  MOTION_GYROMETER_3D = 0x0076,
  MOTION_MOTION_DETECTOR = 0x0077,
  MOTION_SPEEDOMETER = 0x0078,
  MOTION_ACCELEROMETER = 0x0079,
  MOTION_GYROMETER = 0x007a,
  MOTION_GRAVITY_VECTOR = 0x007b,
  MOTION_LINEAR_ACCELEROMETER = 0x007c,
  ORIENTATION = 0x0080,
  ORIENTATION_COMPASS_1D = 0x0081,
  ORIENTATION_COMPASS_2D = 0x0082,
  ORIENTATION_COMPASS_3D = 0x0083,
  ORIENTATION_INCLINOMETER_1D = 0x0084,
  ORIENTATION_INCLINOMETER_2D = 0x0085,
  ORIENTATION_INCLINOMETER_3D = 0x0086,
  ORIENTATION_DISTANCE_1D = 0x0087,
  ORIENTATION_DISTANCE_2D = 0x0088,
  ORIENTATION_DISTANCE_3D = 0x0089,
  ORIENTATION_DEVICE_ORIENTATION = 0x008a,
  ORIENTATION_COMPASS = 0x008b,
  ORIENTATION_INCLINOMETER = 0x008c,
  ORIENTATION_DISTANCE = 0x008d,
  ORIENTATION_RELATIVE_ORIENTATION = 0x008e,
  ORIENTATION_SIMPLE_ORIENTATION = 0x008f,
  SCANNER = 0x0090,
  SCANNER_BARCODE = 0x0091,
  SCANNER_RFID = 0x0092,
  SCANNER_NFC = 0x0093,
  TIME = 0x00a0,
  TIME_ALARM_TIMER = 0x00a1,
  TIME_REAL_TIME_CLOCK = 0x00a2,
  PERSONAL_ACTIVITY = 0x00b0,
  PERSONAL_ACTIVITY_ACTIVITY_DETECTION = 0x00b1,
  PERSONAL_ACTIVITY_DEVICE_POSITION = 0x00b2,
  PERSONAL_ACTIVITY_FLOOR_TRACKER = 0x00b3,
  PERSONAL_ACTIVITY_PEDOMETER = 0x00b4,
  PERSONAL_ACTIVITY_STEP_DETECTION = 0x00b5,
  ORIENTATION_EXTENDED = 0x00c0,
  ORIENTATION_EXTENDED_GEOMAGNETIC_ORIENTATION = 0x00c1,
  ORIENTATION_EXTENDED_MAGNETOMETER = 0x00c2,
  GESTURE = 0x00d0,
  GESTURE_CHASSIS_FLIP_GESTURE = 0x00d1,
  GESTURE_HINGE_FOLD_GESTURE = 0x00d2,
  OTHER = 0x00e0,
  OTHER_CUSTOM = 0x00e1,
  OTHER_GENERIC = 0x00e2,
  OTHER_GENERIC_ENUMERATOR = 0x00e3,
  OTHER_HINGE_ANGLE = 0x00e4,
  VENDOR_RESERVED_1 = 0x00f0,
  VENDOR_RESERVED_2 = 0x00f1,
  VENDOR_RESERVED_3 = 0x00f2,
  VENDOR_RESERVED_4 = 0x00f3,
  VENDOR_RESERVED_5 = 0x00f4,
  VENDOR_RESERVED_6 = 0x00f5,
  VENDOR_RESERVED_7 = 0x00f6,
  VENDOR_RESERVED_8 = 0x00f7,
  VENDOR_RESERVED_9 = 0x00f8,
  VENDOR_RESERVED_10 = 0x00f9,
  VENDOR_RESERVED_11 = 0x00fa,
  VENDOR_RESERVED_12 = 0x00fb,
  VENDOR_RESERVED_13 = 0x00fc,
  VENDOR_RESERVED_14 = 0x00fd,
  VENDOR_RESERVED_15 = 0x00fe,
  VENDOR_RESERVED_16 = 0x00ff,
  EVENT = 0x0200,
  EVENT_SENSOR_STATE = 0x0201,
  EVENT_SENSOR_EVENT = 0x0202,
  PROPERTY = 0x0300,
  PROPERTY_FRIENDLY_NAME = 0x0301,
  PROPERTY_PERSISTENT_UNIQUE_ID = 0x0302,
  PROPERTY_SENSOR_STATUS = 0x0303,
  PROPERTY_MINIMUM_REPORT_INTERVAL = 0x0304,
  PROPERTY_SENSOR_MANUFACTURER = 0x0305,
  PROPERTY_SENSOR_MODEL = 0x0306,
  PROPERTY_SENSOR_SERIAL_NUMBER = 0x0307,
  PROPERTY_SENSOR_DESCRIPTION = 0x0308,
  PROPERTY_SENSOR_CONNECTION_TYPE = 0x0309,
  PROPERTY_SENSOR_DEVICE_PATH = 0x030a,
  PROPERTY_HARDWARE_REVISION = 0x030b,
  PROPERTY_FIRMWARE_VERSION = 0x030c,
  PROPERTY_RELEASE_DATE = 0x030d,
  PROPERTY_REPORT_INTERVAL = 0x030e,
  PROPERTY_CHANGE_SENSITIVITY_ABSOLUTE = 0x030f,
  PROPERTY_CHANGE_SENSITIVITY_PERCENT_OF_RANGE = 0x0310,
  PROPERTY_CHANGE_SENSITIVITY_PERCENT_RELATIVE = 0x0311,
  PROPERTY_ACCURACY = 0x0312,
  PROPERTY_RESOLUTION = 0x0313,
  PROPERTY_MAXIMUM = 0x0314,
  PROPERTY_MINIMUM = 0x0315,
  PROPERTY_REPORTING_STATE = 0x0316,
  PROPERTY_SAMPLING_RATE = 0x0317,
  PROPERTY_RESPONSE_CURVE = 0x0318,
  PROPERTY_POWER_STATE = 0x0319,
  PROPERTY_MAXIMUM_FIFO_EVENTS = 0x031a,
  PROPERTY_REPORT_LATENCY = 0x031b,
  PROPERTY_FLUSH_FIFO_EVENTS = 0x031c,
  PROPERTY_MAXIMUM_POWER_CONSUMPTION = 0x031d,
  PROPERTY_IS_PRIMARY = 0x031e,
  PROPERTY_HUMAN_PRESENCE_DETECTION_TYPE = 0x031f,
  DATA_FIELD_LOCATION = 0x0400,
  DATA_FIELD_ALTITUDE_ANTENNA_SEA_LEVEL = 0x0402,
  DATA_FIELD_DIFFERENTIAL_REFERENCE_STATION_ID = 0x0403,
  DATA_FIELD_ALTITUDE_ELLIPSOID_ERROR = 0x0404,
  DATA_FIELD_ALTITUDE_ELLIPSOID = 0x0405,
  DATA_FIELD_ALTITUDE_SEA_LEVEL_ERROR = 0x0406,
  DATA_FIELD_ALTITUDE_SEA_LEVEL = 0x0407,
  DATA_FIELD_DIFFERENTIAL_GPS_DATA_AGE = 0x0408,
  DATA_FIELD_ERROR_RADIUS = 0x0409,
  DATA_FIELD_FIX_QUALITY = 0x040a,
  DATA_FIELD_FIX_TYPE = 0x040b,
  DATA_FIELD_GEOIDAL_SEPARATION = 0x040c,
  DATA_FIELD_GPS_OPERATION_MODE = 0x040d,
  DATA_FIELD_GPS_SELECTION_MODE = 0x040e,
  DATA_FIELD_GPS_STATUS = 0x040f,
  DATA_FIELD_POSITION_DILUTION_OF_PRECISION = 0x0410,
  DATA_FIELD_HORIZONTAL_DILUTION_OF_PRECISION = 0x0411,
  DATA_FIELD_VERTICAL_DILUTION_OF_PRECISION = 0x0412,
  DATA_FIELD_LATITUDE = 0x0413,
  DATA_FIELD_LONGITUDE = 0x0414,
  DATA_FIELD_TRUE_HEADING = 0x0415,
  DATA_FIELD_MAGNETIC_HEADING = 0x0416,
  DATA_FIELD_MAGNETIC_VARIATION = 0x0417,
  DATA_FIELD_SPEED = 0x0418,
  DATA_FIELD_SATELLITES_IN_VIEW = 0x0419,
  DATA_FIELD_SATELLITES_IN_VIEW_AZIMUTH = 0x041a,
  DATA_FIELD_SATELLITES_IN_VIEW_ELEVATION = 0x041b,
  DATA_FIELD_SATELLITES_IN_VIEW_IDS = 0x041c,
  DATA_FIELD_SATELLITES_IN_VIEW_PRNS = 0x041d,
  DATA_FIELD_SATELLITES_IN_VIEW_S_N_RATIOS = 0x041e,
  DATA_FIELD_SATELLITES_USED_COUNT = 0x041f,
  DATA_FIELD_SATELLITES_USED_PRNS = 0x0420,
  DATA_FIELD_NMEA_SENTENCE = 0x0421,
  DATA_FIELD_ADDRESS_LINE_1 = 0x0422,
  DATA_FIELD_ADDRESS_LINE_2 = 0x0423,
  DATA_FIELD_CITY = 0x0424,
  DATA_FIELD_STATE_OR_PROVINCE = 0x0425,
  DATA_FIELD_COUNTRY_OR_REGION = 0x0426,
  DATA_FIELD_POSTAL_CODE = 0x0427,
  PROPERTY_LOCATION = 0x042a,
  PROPERTY_LOCATION_DESIRED_ACCURACY = 0x042b,
  DATA_FIELD_ENVIRONMENTAL = 0x0430,
  DATA_FIELD_ATMOSPHERIC_PRESSURE = 0x0431,
  DATA_FIELD_RELATIVE_HUMIDITY = 0x0433,
  DATA_FIELD_TEMPERATURE = 0x0434,
  DATA_FIELD_WIND_DIRECTION = 0x0435,
  DATA_FIELD_WIND_SPEED = 0x0436,
  DATA_FIELD_AIR_QUALITY_INDEX = 0x0437,
  DATA_FIELD_EQUIVALENT_CO2 = 0x0438,
  DATA_FIELD_VOLATILE_ORGANIC_COMPOUND_CONCENTRATION = 0x0439,
  DATA_FIELD_OBJECT_PRESENCE = 0x043a,
  DATA_FIELD_OBJECT_PROXIMITY_RANGE = 0x043b,
  DATA_FIELD_OBJECT_PROXIMITY_OUT_OF_RANGE = 0x043c,
  PROPERTY_ENVIRONMENTAL = 0x0440,
  PROPERTY_REFERENCE_PRESSURE = 0x0441,
  DATA_FIELD_MOTION = 0x0450,
  DATA_FIELD_MOTION_STATE = 0x0451,
  DATA_FIELD_ACCELERATION = 0x0452,
  DATA_FIELD_ACCELERATION_AXIS_X = 0x0453,
  DATA_FIELD_ACCELERATION_AXIS_Y = 0x0454,
  DATA_FIELD_ACCELERATION_AXIS_Z = 0x0455,
  DATA_FIELD_ANGULAR_VELOCITY = 0x0456,
  DATA_FIELD_ANGULAR_VELOCITY_ABOUT_X_AXIS = 0x0457,
  DATA_FIELD_ANGULAR_VELOCITY_ABOUT_Y_AXIS = 0x0458,
  DATA_FIELD_ANGULAR_VELOCITY_ABOUT_Z_AXIS = 0x0459,
  DATA_FIELD_ANGULAR_POSITION = 0x045a,
  DATA_FIELD_ANGULAR_POSITION_ABOUT_X_AXIS = 0x045b,
  DATA_FIELD_ANGULAR_POSITION_ABOUT_Y_AXIS = 0x045c,
  DATA_FIELD_ANGULAR_POSITION_ABOUT_Z_AXIS = 0x045d,
  DATA_FIELD_MOTION_SPEED = 0x045e,
  DATA_FIELD_MOTION_INTENSITY = 0x045f,
  DATA_FIELD_ORIENTATION = 0x0470,
  DATA_FIELD_HEADING = 0x0471,
  DATA_FIELD_HEADING_X_AXIS = 0x0472,
  DATA_FIELD_HEADING_Y_AXIS = 0x0473,
  DATA_FIELD_HEADING_Z_AXIS = 0x0474,
  DATA_FIELD_HEADING_COMPENSATED_MAGNETIC_NORTH = 0x0475,
  DATA_FIELD_HEADING_COMPENSATED_TRUE_NORTH = 0x0476,
  DATA_FIELD_HEADING_MAGNETIC_NORTH = 0x0477,
  DATA_FIELD_HEADING_TRUE_NORTH = 0x0478,
  DATA_FIELD_DISTANCE = 0x0479,
  DATA_FIELD_DISTANCE_X_AXIS = 0x047a,
  DATA_FIELD_DISTANCE_Y_AXIS = 0x047b,
  DATA_FIELD_DISTANCE_Z_AXIS = 0x047c,
  DATA_FIELD_DISTANCE_OUT_OF_RANGE = 0x047d,
  DATA_FIELD_TILT = 0x047e,
  DATA_FIELD_TILT_X_AXIS = 0x047f,
  DATA_FIELD_TILT_Y_AXIS = 0x0480,
  DATA_FIELD_TILT_Z_AXIS = 0x0481,
  DATA_FIELD_ROTATION_MATRIX = 0x0482,
  DATA_FIELD_QUATERNION = 0x0483,
  DATA_FIELD_MAGNETIC_FLUX = 0x0484,
  DATA_FIELD_MAGNETIC_FLUX_X_AXIS = 0x0485,
  DATA_FIELD_MAGNETIC_FLUX_Y_AXIS = 0x0486,
  DATA_FIELD_MAGNETIC_FLUX_Z_AXIS = 0x0487,
  DATA_FIELD_MAGNETOMETER_ACCURACY = 0x0488,
  DATA_FIELD_SIMPLE_ORIENTATION_DIRECTION = 0x0489,
  DATA_FIELD_MECHANICAL = 0x0490,
  DATA_FIELD_BOOLEAN_SWITCH_STATE = 0x0491,
  DATA_FIELD_BOOLEAN_SWITCH_ARRAY_STATES = 0x0492,
  DATA_FIELD_MULTIVALUE_SWITCH_VALUE = 0x0493,
  DATA_FIELD_FORCE = 0x0494,
  DATA_FIELD_ABSOLUTE_PRESSURE = 0x0495,
  DATA_FIELD_GAUGE_PRESSURE = 0x0496,
  DATA_FIELD_STRAIN = 0x0497,
  DATA_FIELD_WEIGHT = 0x0498,
  PROPERTY_MECHANICAL = 0x04a0,
  PROPERTY_VIBRATION_STATE = 0x04a1,
  PROPERTY_FORWARD_VIBRATION_SPEED = 0x04a2,
  PROPERTY_BACKWARD_VIBRATION_SPEED = 0x04a3,
  DATA_FIELD_BIOMETRIC = 0x04b0,
  DATA_FIELD_HUMAN_PRESENCE = 0x04b1,
  DATA_FIELD_HUMAN_PROXIMITY_RANGE = 0x04b2,
  DATA_FIELD_HUMAN_PROXIMITY_OUT_OF_RANGE = 0x04b3,
  DATA_FIELD_HUMAN_TOUCH_STATE = 0x04b4,
  DATA_FIELD_BLOOD_PRESSURE = 0x04b5,
  DATA_FIELD_BLOOD_PRESSURE_DIASTOLIC = 0x04b6,
  DATA_FIELD_BLOOD_PRESSURE_SYSTOLIC = 0x04b7,
  DATA_FIELD_HEART_RATE = 0x04b8,
  DATA_FIELD_RESTING_HEART_RATE = 0x04b9,
  DATA_FIELD_HEARTBEAT_INTERVAL = 0x04ba,
  DATA_FIELD_RESPIRATORY_RATE = 0x04bb,
  DATA_FIELD_SPO2 = 0x04bc,
  DATA_FIELD_HUMAN_ATTENTION_DETECTED = 0x04bd,
  DATA_FIELD_HUMAN_HEAD_AZIMUTH = 0x04be,
  DATA_FIELD_HUMAN_HEAD_ALTITUDE = 0x04bf,
  DATA_FIELD_HUMAN_HEAD_ROLL = 0x04c0,
  DATA_FIELD_HUMAN_HEAD_PITCH = 0x04c1,
  DATA_FIELD_HUMAN_HEAD_YAW = 0x04c2,
  DATA_FIELD_HUMAN_CORRELATION_ID = 0x04c3,
  DATA_FIELD_LIGHT = 0x04d0,
  DATA_FIELD_ILLUMINANCE = 0x04d1,
  DATA_FIELD_COLOR_TEMPERATURE = 0x04d2,
  DATA_FIELD_CHROMATICITY = 0x04d3,
  DATA_FIELD_CHROMATICITY_X = 0x04d4,
  DATA_FIELD_CHROMATICITY_Y = 0x04d5,
  DATA_FIELD_CONSUMER_IR_SENTENCE_RECEIVE = 0x04d6,
  DATA_FIELD_INFRARED_LIGHT = 0x04d7,
  DATA_FIELD_RED_LIGHT = 0x04d8,
  DATA_FIELD_GREEN_LIGHT = 0x04d9,
  DATA_FIELD_BLUE_LIGHT = 0x04da,
  DATA_FIELD_ULTRAVIOLET_A_LIGHT = 0x04db,
  DATA_FIELD_ULTRAVIOLET_B_LIGHT = 0x04dc,
  DATA_FIELD_ULTRAVIOLET_INDEX = 0x04dd,
  DATA_FIELD_NEAR_INFRARED_LIGHT = 0x04de,
  PROPERTY_LIGHT = 0x04df,
  PROPERTY_CONSUMER_IR_SENTENCE_SEND = 0x04e0,
  PROPERTY_AUTO_BRIGHTNESS_PREFERRED = 0x04e2,
  PROPERTY_AUTO_COLOR_PREFERRED = 0x04e3,
  DATA_FIELD_SCANNER = 0x04f0,
  DATA_FIELD_RFID_TAG_40_BIT = 0x04f1,
  DATA_FIELD_NFC_SENTENCE_RECEIVE = 0x04f2,
  PROPERTY_SCANNER = 0x04f8,
  PROPERTY_NFC_SENTENCE_SEND = 0x04f9,
  DATA_FIELD_ELECTRICAL = 0x0500,
  DATA_FIELD_CAPACITANCE = 0x0501,
  DATA_FIELD_CURRENT = 0x0502,
  DATA_FIELD_ELECTRICAL_POWER = 0x0503,
  DATA_FIELD_INDUCTANCE = 0x0504,
  DATA_FIELD_RESISTANCE = 0x0505,
  DATA_FIELD_VOLTAGE = 0x0506,
  DATA_FIELD_FREQUENCY = 0x0507,
  DATA_FIELD_PERIOD = 0x0508,
  DATA_FIELD_PERCENT_OF_RANGE = 0x0509,
  DATA_FIELD_TIME = 0x0520,
  DATA_FIELD_YEAR = 0x0521,
  DATA_FIELD_MONTH = 0x0522,
  DATA_FIELD_DAY = 0x0523,
  DATA_FIELD_DAY_OF_WEEK = 0x0524,
  DATA_FIELD_HOUR = 0x0525,
  DATA_FIELD_MINUTE = 0x0526,
  DATA_FIELD_SECOND = 0x0527,
  DATA_FIELD_MILLISECOND = 0x0528,
  DATA_FIELD_TIMESTAMP = 0x0529,
  DATA_FIELD_JULIAN_DAY_OF_YEAR = 0x052a,
  DATA_FIELD_TIME_SINCE_SYSTEM_BOOT = 0x052b,
  PROPERTY_TIME = 0x0530,
  PROPERTY_TIME_ZONE_OFFSET_FROM_UTC = 0x0531,
  PROPERTY_TIME_ZONE_NAME = 0x0532,
  PROPERTY_DAYLIGHT_SAVINGS_TIME_OBSERVED = 0x0533,
  PROPERTY_TIME_TRIM_ADJUSTMENT = 0x0534,
  PROPERTY_ARM_ALARM = 0x0535,
  DATA_FIELD_CUSTOM = 0x0540,
  DATA_FIELD_CUSTOM_USAGE = 0x0541,
  DATA_FIELD_CUSTOM_BOOLEAN_ARRAY = 0x0542,
  DATA_FIELD_CUSTOM_VALUE = 0x0543,
  DATA_FIELD_CUSTOM_VALUE_1 = 0x0544,
  DATA_FIELD_CUSTOM_VALUE_2 = 0x0545,
  DATA_FIELD_CUSTOM_VALUE_3 = 0x0546,
  DATA_FIELD_CUSTOM_VALUE_4 = 0x0547,
  DATA_FIELD_CUSTOM_VALUE_5 = 0x0548,
  DATA_FIELD_CUSTOM_VALUE_6 = 0x0549,
  DATA_FIELD_CUSTOM_VALUE_7 = 0x054a,
  DATA_FIELD_CUSTOM_VALUE_8 = 0x054b,
  DATA_FIELD_CUSTOM_VALUE_9 = 0x054c,
  DATA_FIELD_CUSTOM_VALUE_10 = 0x054d,
  DATA_FIELD_CUSTOM_VALUE_11 = 0x054e,
  DATA_FIELD_CUSTOM_VALUE_12 = 0x054f,
  DATA_FIELD_CUSTOM_VALUE_13 = 0x0550,
  DATA_FIELD_CUSTOM_VALUE_14 = 0x0551,
  DATA_FIELD_CUSTOM_VALUE_15 = 0x0552,
  DATA_FIELD_CUSTOM_VALUE_16 = 0x0553,
  DATA_FIELD_CUSTOM_VALUE_17 = 0x0554,
  DATA_FIELD_CUSTOM_VALUE_18 = 0x0555,
  DATA_FIELD_CUSTOM_VALUE_19 = 0x0556,
  DATA_FIELD_CUSTOM_VALUE_20 = 0x0557,
  DATA_FIELD_CUSTOM_VALUE_21 = 0x0558,
  DATA_FIELD_CUSTOM_VALUE_22 = 0x0559,
  DATA_FIELD_CUSTOM_VALUE_23 = 0x055a,
  DATA_FIELD_CUSTOM_VALUE_24 = 0x055b,
  DATA_FIELD_CUSTOM_VALUE_25 = 0x055c,
  DATA_FIELD_CUSTOM_VALUE_26 = 0x055d,
  DATA_FIELD_CUSTOM_VALUE_27 = 0x055e,
  DATA_FIELD_CUSTOM_VALUE_28 = 0x055f,
  DATA_FIELD_GENERIC = 0x0560,
  DATA_FIELD_GENERIC_GUID_OR_PROPERTYKEY = 0x0561,
  DATA_FIELD_GENERIC_CATEGORY_GUID = 0x0562,
  DATA_FIELD_GENERIC_TYPE_GUID = 0x0563,
  DATA_FIELD_GENERIC_EVENT_PROPERTYKEY = 0x0564,
  DATA_FIELD_GENERIC_PROPERTY_PROPERTYKEY = 0x0565,
  DATA_FIELD_GENERIC_DATA_FIELD_PROPERTYKEY = 0x0566,
  DATA_FIELD_GENERIC_EVENT = 0x0567,
  DATA_FIELD_GENERIC_PROPERTY = 0x0568,
  DATA_FIELD_GENERIC_DATA_FIELD = 0x0569,
  DATA_FIELD_ENUMERATOR_TABLE_ROW_INDEX = 0x056a,
  DATA_FIELD_ENUMERATOR_TABLE_ROW_COUNT = 0x056b,
  DATA_FIELD_GENERIC_GUID_OR_PROPERTYKEY_KIND = 0x056c,
  DATA_FIELD_GENERIC_GUID = 0x056d,
  DATA_FIELD_GENERIC_PROPERTYKEY = 0x056e,
  DATA_FIELD_GENERIC_TOP_LEVEL_COLLECTION_ID = 0x056f,
  DATA_FIELD_GENERIC_REPORT_ID = 0x0570,
  DATA_FIELD_GENERIC_REPORT_ITEM_POSITION_INDEX = 0x0571,
  DATA_FIELD_GENERIC_FIRMWARE_VARTYPE = 0x0572,
  DATA_FIELD_GENERIC_UNIT_OF_MEASURE = 0x0573,
  DATA_FIELD_GENERIC_UNIT_EXPONENT = 0x0574,
  DATA_FIELD_GENERIC_REPORT_SIZE = 0x0575,
  DATA_FIELD_GENERIC_REPORT_COUNT = 0x0576,
  PROPERTY_GENERIC = 0x0580,
  PROPERTY_ENUMERATOR_TABLE_ROW_INDEX = 0x0581,
  PROPERTY_ENUMERATOR_TABLE_ROW_COUNT = 0x0582,
  DATA_FIELD_PERSONAL_ACTIVITY = 0x0590,
  DATA_FIELD_ACTIVITY_TYPE = 0x0591,
  DATA_FIELD_ACTIVITY_STATE = 0x0592,
  DATA_FIELD_DEVICE_POSITION = 0x0593,
  DATA_FIELD_STEP_COUNT = 0x0594,
  DATA_FIELD_STEP_COUNT_RESET = 0x0595,
  DATA_FIELD_STEP_DURATION = 0x0596,
  DATA_FIELD_STEP_TYPE = 0x0597,
  PROPERTY_MINIMUM_ACTIVITY_DETECTION_INTERVAL = 0x05a0,
  PROPERTY_SUPPORTED_ACTIVITY_TYPES = 0x05a1,
  PROPERTY_SUBSCRIBED_ACTIVITY_TYPES = 0x05a2,
  PROPERTY_SUPPORTED_STEP_TYPES = 0x05a3,
  PROPERTY_SUBSCRIBED_STEP_TYPES = 0x05a4,
  PROPERTY_FLOOR_HEIGHT = 0x05a5,
  DATA_FIELD_CUSTOM_TYPE_ID = 0x05b0,
  PROPERTY_CUSTOM = 0x05c0,
  PROPERTY_CUSTOM_VALUE_1 = 0x05c1,
  PROPERTY_CUSTOM_VALUE_2 = 0x05c2,
  PROPERTY_CUSTOM_VALUE_3 = 0x05c3,
  PROPERTY_CUSTOM_VALUE_4 = 0x05c4,
  PROPERTY_CUSTOM_VALUE_5 = 0x05c5,
  PROPERTY_CUSTOM_VALUE_6 = 0x05c6,
  PROPERTY_CUSTOM_VALUE_7 = 0x05c7,
  PROPERTY_CUSTOM_VALUE_8 = 0x05c8,
  PROPERTY_CUSTOM_VALUE_9 = 0x05c9,
  PROPERTY_CUSTOM_VALUE_10 = 0x05ca,
  PROPERTY_CUSTOM_VALUE_11 = 0x05cb,
  PROPERTY_CUSTOM_VALUE_12 = 0x05cc,
  PROPERTY_CUSTOM_VALUE_13 = 0x05cd,
  PROPERTY_CUSTOM_VALUE_14 = 0x05ce,
  PROPERTY_CUSTOM_VALUE_15 = 0x05cf,
  PROPERTY_CUSTOM_VALUE_16 = 0x05d0,
  DATA_FIELD_HINGE = 0x05e0,
  DATA_FIELD_HINGE_ANGLE = 0x05e1,
  DATA_FIELD_GESTURE_SENSOR = 0x05f0,
  DATA_FIELD_GESTURE_STATE = 0x05f1,
  DATA_FIELD_HINGE_FOLD_INITIAL_ANGLE = 0x05f2,
  DATA_FIELD_HINGE_FOLD_FINAL_ANGLE = 0x05f3,
  DATA_FIELD_HINGE_FOLD_CONTRIBUTING_PANEL = 0x05f4,
  DATA_FIELD_HINGE_FOLD_TYPE = 0x05f5,
  SENSOR_STATE_UNDEFINED = 0x0800,
  SENSOR_STATE_READY = 0x0801,
  SENSOR_STATE_NOT_AVAILABLE = 0x0802,
  SENSOR_STATE_NO_DATA = 0x0803,
  SENSOR_STATE_INITIALIZING = 0x0804,
  SENSOR_STATE_ACCESS_DENIED = 0x0805,
  SENSOR_STATE_ERROR = 0x0806,
  SENSOR_EVENT_UNKNOWN = 0x0810,
  SENSOR_EVENT_STATE_CHANGED = 0x0811,
  SENSOR_EVENT_PROPERTY_CHANGED = 0x0812,
  SENSOR_EVENT_DATA_UPDATED = 0x0813,
  SENSOR_EVENT_POLL_RESPONSE = 0x0814,
  SENSOR_EVENT_CHANGE_SENSITIVITY = 0x0815,
  SENSOR_EVENT_RANGE_MAXIMUM_REACHED = 0x0816,
  SENSOR_EVENT_RANGE_MINIMUM_REACHED = 0x0817,
  SENSOR_EVENT_HIGH_THRESHOLD_CROSS_UPWARD = 0x0818,
  SENSOR_EVENT_HIGH_THRESHOLD_CROSS_DOWNWARD = 0x0819,
  SENSOR_EVENT_LOW_THRESHOLD_CROSS_UPWARD = 0x081a,
  SENSOR_EVENT_LOW_THRESHOLD_CROSS_DOWNWARD = 0x081b,
  SENSOR_EVENT_ZERO_THRESHOLD_CROSS_UPWARD = 0x081c,
  SENSOR_EVENT_ZERO_THRESHOLD_CROSS_DOWNWARD = 0x081d,
  SENSOR_EVENT_PERIOD_EXCEEDED = 0x081e,
  SENSOR_EVENT_FREQUENCY_EXCEEDED = 0x081f,
  SENSOR_EVENT_COMPLEX_TRIGGER = 0x0820,
  CONNECTION_TYPE_PC_INTEGRATED = 0x0830,
  CONNECTION_TYPE_PC_ATTACHED = 0x0831,
  CONNECTION_TYPE_PC_EXTERNAL = 0x0832,
  REPORTING_STATE_REPORT_NO_EVENTS = 0x0840,
  REPORTING_STATE_REPORT_ALL_EVENTS = 0x0841,
  REPORTING_STATE_REPORT_THRESHOLD_EVENTS = 0x0842,
  REPORTING_STATE_WAKE_ON_NO_EVENTS = 0x0843,
  REPORTING_STATE_WAKE_ON_ALL_EVENTS = 0x0844,
  REPORTING_STATE_WAKE_ON_THRESHOLD_EVENTS = 0x0845,
  REPORTING_STATE_ANYTIME = 0x0846,
  POWER_STATE_UNDEFINED = 0x0850,
  POWER_STATE_D0_FULL_POWER = 0x0851,
  POWER_STATE_D1_LOW_POWER = 0x0852,
  POWER_STATE_D2_STANDBY_POWER_WITH_WAKEUP = 0x0853,
  POWER_STATE_D3_SLEEP_WITH_WAKEUP = 0x0854,
  POWER_STATE_D4_POWER_OFF = 0x0855,
  ACCURACY_DEFAULT = 0x0860,
  ACCURACY_HIGH = 0x0861,
  ACCURACY_MEDIUM = 0x0862,
  ACCURACY_LOW = 0x0863,
  FIX_QUALITY_NO_FIX = 0x0870,
  FIX_QUALITY_GPS = 0x0871,
  FIX_QUALITY_DGPS = 0x0872,
  FIX_TYPE_NO_FIX = 0x0880,
  FIX_TYPE_GPS_SPS_MODE_FIX_VALID = 0x0881,
  FIX_TYPE_DGPS_SPS_MODE_FIX_VALID = 0x0882,
  FIX_TYPE_GPS_PPS_MODE_FIX_VALID = 0x0883,
  FIX_TYPE_REAL_TIME_KINEMATIC = 0x0884,
  FIX_TYPE_FLOAT_RTK = 0x0885,
  FIX_TYPE_ESTIMATED_DEAD_RECKONED = 0x0886,
  FIX_TYPE_MANUAL_INPUT_MODE = 0x0887,
  FIX_TYPE_SIMULATOR_MODE = 0x0888,
  GPS_OPERATION_MODE_MANUAL = 0x0890,
  GPS_OPERATION_MODE_AUTOMATIC = 0x0891,
  GPS_SELECTION_MODE_AUTONOMOUS = 0x08a0,
  GPS_SELECTION_MODE_DGPS = 0x08a1,
  GPS_SELECTION_MODE_ESTIMATED_DEAD_RECKONED = 0x08a2,
  GPS_SELECTION_MODE_MANUAL_INPUT = 0x08a3,
  GPS_SELECTION_MODE_SIMULATOR = 0x08a4,
  GPS_SELECTION_MODE_DATA_NOT_VALID = 0x08a5,
  GPS_STATUS_DATA_VALID = 0x08b0,
  GPS_STATUS_DATA_NOT_VALID = 0x08b1,
  DAY_OF_WEEK_SUNDAY = 0x08c0,
  DAY_OF_WEEK_MONDAY = 0x08c1,
  DAY_OF_WEEK_TUESDAY = 0x08c2,
  DAY_OF_WEEK_WEDNESDAY = 0x08c3,
  DAY_OF_WEEK_THURSDAY = 0x08c4,
  DAY_OF_WEEK_FRIDAY = 0x08c5,
  DAY_OF_WEEK_SATURDAY = 0x08c6,
  KIND_CATEGORY = 0x08d0,
  KIND_TYPE = 0x08d1,
  KIND_EVENT = 0x08d2,
  KIND_PROPERTY = 0x08d3,
  KIND_DATA_FIELD = 0x08d4,
  MAGNETOMETER_ACCURACY_LOW = 0x08e0,
  MAGNETOMETER_ACCURACY_MEDIUM = 0x08e1,
  MAGNETOMETER_ACCURACY_HIGH = 0x08e2,
  SIMPLE_ORIENTATION_DIRECTION_NOT_ROTATED = 0x08f0,
  SIMPLE_ORIENTATION_DIRECTION_ROTATED_90_DEGREES_CCW = 0x08f1,
  SIMPLE_ORIENTATION_DIRECTION_ROTATED_180_DEGREES_CCW = 0x08f2,
  SIMPLE_ORIENTATION_DIRECTION_ROTATED_270_DEGREES_CCW = 0x08f3,
  SIMPLE_ORIENTATION_DIRECTION_FACE_UP = 0x08f4,
  SIMPLE_ORIENTATION_DIRECTION_FACE_DOWN = 0x08f5,
  VT_NULL_EMPTY = 0x0900,
  VT_BOOL_BOOLEAN = 0x0901,
  VT_UI1_BYTE = 0x0902,
  VT_I1_CHARACTER = 0x0903,
  VT_UI2_UNSIGNED_SHORT = 0x0904,
  VT_I2_SHORT = 0x0905,
  VT_UI4_UNSIGNED_LONG = 0x0906,
  VT_I4_LONG = 0x0907,
  VT_UI8_UNSIGNED_LONG_LONG = 0x0908,
  VT_I8_LONG_LONG = 0x0909,
  VT_R4_FLOAT = 0x090a,
  VT_R8_DOUBLE = 0x090b,
  VT_WSTR_WIDE_STRING = 0x090c,
  VT_STR_NARROW_STRING = 0x090d,
  VT_CLSID_GUID = 0x090e,
  VT_VECTOR_VT_UI1_OPAQUE_STRUCTURE = 0x090f,
  VT_F16E0_HID_16_BIT_FLOAT_E0 = 0x0910,
  VT_F16E1_HID_16_BIT_FLOAT_E1 = 0x0911,
  VT_F16E2_HID_16_BIT_FLOAT_E2 = 0x0912,
  VT_F16E3_HID_16_BIT_FLOAT_E3 = 0x0913,
  VT_F16E4_HID_16_BIT_FLOAT_E4 = 0x0914,
  VT_F16E5_HID_16_BIT_FLOAT_E5 = 0x0915,
  VT_F16E6_HID_16_BIT_FLOAT_E6 = 0x0916,
  VT_F16E7_HID_16_BIT_FLOAT_E7 = 0x0917,
  VT_F16E8_HID_16_BIT_FLOAT_E_8 = 0x0918,
  VT_F16E9_HID_16_BIT_FLOAT_E_7 = 0x0919,
  VT_F16EA_HID_16_BIT_FLOAT_E_6 = 0x091a,
  VT_F16EB_HID_16_BIT_FLOAT_E_5 = 0x091b,
  VT_F16EC_HID_16_BIT_FLOAT_E_4 = 0x091c,
  VT_F16ED_HID_16_BIT_FLOAT_E_3 = 0x091d,
  VT_F16EE_HID_16_BIT_FLOAT_E_2 = 0x091e,
  VT_F16EF_HID_16_BIT_FLOAT_E_1 = 0x091f,
  VT_F32E0_HID_32_BIT_FLOAT_E0 = 0x0920,
  VT_F32E1_HID_32_BIT_FLOAT_E1 = 0x0921,
  VT_F32E2_HID_32_BIT_FLOAT_E2 = 0x0922,
  VT_F32E3_HID_32_BIT_FLOAT_E3 = 0x0923,
  VT_F32E4_HID_32_BIT_FLOAT_E4 = 0x0924,
  VT_F32E5_HID_32_BIT_FLOAT_E5 = 0x0925,
  VT_F32E6_HID_32_BIT_FLOAT_E6 = 0x0926,
  VT_F32E7_HID_32_BIT_FLOAT_E7 = 0x0927,
  VT_F32E8_HID_32_BIT_FLOAT_E_8 = 0x0928,
  VT_F32E9_HID_32_BIT_FLOAT_E_7 = 0x0929,
  VT_F32EA_HID_32_BIT_FLOAT_E_6 = 0x092a,
  VT_F32EB_HID_32_BIT_FLOAT_E_5 = 0x092b,
  VT_F32EC_HID_32_BIT_FLOAT_E_4 = 0x092c,
  VT_F32ED_HID_32_BIT_FLOAT_E_3 = 0x092d,
  VT_F32EE_HID_32_BIT_FLOAT_E_2 = 0x092e,
  VT_F32EF_HID_32_BIT_FLOAT_E_1 = 0x092f,
  ACTIVITY_TYPE_UNKNOWN = 0x0930,
  ACTIVITY_TYPE_STATIONARY = 0x0931,
  ACTIVITY_TYPE_FIDGETING = 0x0932,
  ACTIVITY_TYPE_WALKING = 0x0933,
  ACTIVITY_TYPE_RUNNING = 0x0934,
  ACTIVITY_TYPE_IN_VEHICLE = 0x0935,
  ACTIVITY_TYPE_BIKING = 0x0936,
  ACTIVITY_TYPE_IDLE = 0x0937,
  UNIT_NOT_SPECIFIED = 0x0940,
  UNIT_LUX = 0x0941,
  UNIT_DEGREES_KELVIN = 0x0942,
  UNIT_DEGREES_CELSIUS = 0x0943,
  UNIT_PASCAL = 0x0944,
  UNIT_NEWTON = 0x0945,
  UNIT_METERS_SECOND = 0x0946,
  UNIT_KILOGRAM = 0x0947,
  UNIT_METER = 0x0948,
  UNIT_METERS_SECOND_SECOND = 0x0949,
  UNIT_FARAD = 0x094a,
  UNIT_AMPERE = 0x094b,
  UNIT_WATT = 0x094c,
  UNIT_HENRY = 0x094d,
  UNIT_OHM = 0x094e,
  UNIT_VOLT = 0x094f,
  UNIT_HERTZ = 0x0950,
  UNIT_BAR = 0x0951,
  UNIT_DEGREES_ANTI_CLOCKWISE = 0x0952,
  UNIT_DEGREES_CLOCKWISE = 0x0953,
  UNIT_DEGREES = 0x0954,
  UNIT_DEGREES_SECOND = 0x0955,
  UNIT_DEGREES_SECOND_SECOND = 0x0956,
  UNIT_KNOT = 0x0957,
  UNIT_PERCENT = 0x0958,
  UNIT_SECOND = 0x0959,
  UNIT_MILLISECOND = 0x095a,
  UNIT_G = 0x095b,
  UNIT_BYTES = 0x095c,
  UNIT_MILLIGAUSS = 0x095d,
  UNIT_BITS = 0x095e,
  ACTIVITY_STATE_NO_STATE_CHANGE = 0x0960,
  ACTIVITY_STATE_START_ACTIVITY = 0x0961,
  ACTIVITY_STATE_END_ACTIVITY = 0x0962,
  EXPONENT_0_E0 = 0x0970,
  EXPONENT_1_E1 = 0x0971,
  EXPONENT_2_E2 = 0x0972,
  EXPONENT_3_E3 = 0x0973,
  EXPONENT_4_E4 = 0x0974,
  EXPONENT_5_E5 = 0x0975,
  EXPONENT_6_E6 = 0x0976,
  EXPONENT_7_E7 = 0x0977,
  EXPONENT_8_E_8 = 0x0978,
  EXPONENT_9_E_7 = 0x0979,
  EXPONENT_A_E_6 = 0x097a,
  EXPONENT_B_E_5 = 0x097b,
  EXPONENT_C_E_4 = 0x097c,
  EXPONENT_D_E_3 = 0x097d,
  EXPONENT_E_E_2 = 0x097e,
  EXPONENT_F_E_1 = 0x097f,
  DEVICE_POSITION_UNKNOWN = 0x0980,
  DEVICE_POSITION_UNCHANGED = 0x0981,
  DEVICE_POSITION_ON_DESK = 0x0982,
  DEVICE_POSITION_IN_HAND = 0x0983,
  DEVICE_POSITION_MOVING_IN_BAG = 0x0984,
  DEVICE_POSITION_STATIONARY_IN_BAG = 0x0985,
  STEP_TYPE_UNKNOWN = 0x0990,
  STEP_TYPE_WALKING = 0x0991,
  STEP_TYPE_RUNNING = 0x0992,
  GESTURE_STATE_UNKNOWN = 0x09a0,
  GESTURE_STATE_STARTED = 0x09a1,
  GESTURE_STATE_COMPLETED = 0x09a2,
  GESTURE_STATE_CANCELLED = 0x09a3,
  HINGE_FOLD_CONTRIBUTING_PANEL_UNKNOWN = 0x09b0,
  HINGE_FOLD_CONTRIBUTING_PANEL_PANEL_1 = 0x09b1,
  HINGE_FOLD_CONTRIBUTING_PANEL_PANEL_2 = 0x09b2,
  HINGE_FOLD_CONTRIBUTING_PANEL_BOTH = 0x09b3,
  HINGE_FOLD_TYPE_UNKNOWN = 0x09b4,
  HINGE_FOLD_TYPE_INCREASING = 0x09b5,
  HINGE_FOLD_TYPE_DECREASING = 0x09b6,
  HUMAN_PRESENCE_DETECTION_TYPE_VENDOR_DEFINED_NON_BIOMETRIC = 0x09c0,
  HUMAN_PRESENCE_DETECTION_TYPE_VENDOR_DEFINED_BIOMETRIC = 0x09c1,
  HUMAN_PRESENCE_DETECTION_TYPE_FACIAL_BIOMETRIC = 0x09c2,
  HUMAN_PRESENCE_DETECTION_TYPE_AUDIO_BIOMETRIC = 0x09c3,
  CHANGE_SENSITIVITY_ABSOLUTE = 0x1000,
  MAXIMUM = 0x2000,
  MINIMUM = 0x3000,
  ACCURACY = 0x4000,
  RESOLUTION = 0x5000,
  THRESHOLD_HIGH = 0x6000,
  THRESHOLD_LOW = 0x7000,
  CALIBRATION_OFFSET = 0x8000,
  CALIBRATION_MULTIPLIER = 0x9000,
  REPORT_INTERVAL = 0xa000,
  FREQUENCY_MAX = 0xb000,
  PERIOD_MAX = 0xc000,
  CHANGE_SENSITIVITY_PERCENT_OF_RANGE = 0xd000,
  CHANGE_SENSITIVITY_PERCENT_RELATIVE = 0xe000,
  VENDOR_RESERVED = 0xf000,
};
} // namespace hid::page

#endif // __HID_PAGE_SENSOR_HPP_
