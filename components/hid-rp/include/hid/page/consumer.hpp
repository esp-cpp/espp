#ifndef __HID_PAGE_CONSUMER_HPP_
#define __HID_PAGE_CONSUMER_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class consumer : std::uint16_t;
template <> constexpr inline auto get_info<consumer>() {
  return info(
      0x000c, 0x0517, "Consumer",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "Consumer Control";
        case 0x0002:
          return "Numeric Key Pad";
        case 0x0003:
          return "Programmable Buttons";
        case 0x0004:
          return "Microphone";
        case 0x0005:
          return "Headphone";
        case 0x0006:
          return "Graphic Equalizer";
        case 0x0007:
          return "Keyboard Backlight";
        case 0x0020:
          return "+10";
        case 0x0021:
          return "+100";
        case 0x0022:
          return "AM/PM";
        case 0x0030:
          return "Power";
        case 0x0031:
          return "Reset";
        case 0x0032:
          return "Sleep";
        case 0x0033:
          return "Sleep After";
        case 0x0034:
          return "Sleep Mode";
        case 0x0035:
          return "Illumination";
        case 0x0036:
          return "Function Buttons";
        case 0x0040:
          return "Menu";
        case 0x0041:
          return "Menu Pick";
        case 0x0042:
          return "Menu Up";
        case 0x0043:
          return "Menu Down";
        case 0x0044:
          return "Menu Left";
        case 0x0045:
          return "Menu Right";
        case 0x0046:
          return "Menu Escape";
        case 0x0047:
          return "Menu Value Increase";
        case 0x0048:
          return "Menu Value Decrease";
        case 0x0060:
          return "Data On Screen";
        case 0x0061:
          return "Closed Caption";
        case 0x0062:
          return "Closed Caption Select";
        case 0x0063:
          return "VCR/TV";
        case 0x0064:
          return "Broadcast Mode";
        case 0x0065:
          return "Snapshot";
        case 0x0066:
          return "Still";
        case 0x0067:
          return "Picture-in-Picture Toggle";
        case 0x0068:
          return "Picture-in-Picture Swap";
        case 0x0069:
          return "Red Menu Button";
        case 0x006a:
          return "Green Menu Button";
        case 0x006b:
          return "Blue Menu Button";
        case 0x006c:
          return "Yellow Menu Button";
        case 0x006d:
          return "Aspect";
        case 0x006e:
          return "3D Mode Select";
        case 0x006f:
          return "Display Brightness Increment";
        case 0x0070:
          return "Display Brightness Decrement";
        case 0x0071:
          return "Display Brightness";
        case 0x0072:
          return "Display Backlight Toggle";
        case 0x0073:
          return "Display Set Brightness to Minimum";
        case 0x0074:
          return "Display Set Brightness to Maximum";
        case 0x0075:
          return "Display Set Auto Brightness";
        case 0x0076:
          return "Camera Access Enabled";
        case 0x0077:
          return "Camera Access Disabled";
        case 0x0078:
          return "Camera Access Toggle";
        case 0x0079:
          return "Keyboard Brightness Increment";
        case 0x007a:
          return "Keyboard Brightness Decrement";
        case 0x007b:
          return "Keyboard Backlight Set Level";
        case 0x007c:
          return "Keyboard Backlight Toggle";
        case 0x007d:
          return "Keyboard Backlight Set Minimum";
        case 0x007e:
          return "Keyboard Backlight Set Maximum";
        case 0x007f:
          return "Keyboard Backlight Auto";
        case 0x0080:
          return "Selection";
        case 0x0081:
          return "Assign Selection";
        case 0x0082:
          return "Mode Step";
        case 0x0083:
          return "Recall Last";
        case 0x0084:
          return "Enter Channel";
        case 0x0085:
          return "Order Movie";
        case 0x0086:
          return "Channel";
        case 0x0087:
          return "Media Selection";
        case 0x0088:
          return "Media Select Computer";
        case 0x0089:
          return "Media Select TV";
        case 0x008a:
          return "Media Select WWW";
        case 0x008b:
          return "Media Select DVD";
        case 0x008c:
          return "Media Select Telephone";
        case 0x008d:
          return "Media Select Program Guide";
        case 0x008e:
          return "Media Select Video Phone";
        case 0x008f:
          return "Media Select Games";
        case 0x0090:
          return "Media Select Messages";
        case 0x0091:
          return "Media Select CD";
        case 0x0092:
          return "Media Select VCR";
        case 0x0093:
          return "Media Select Tuner";
        case 0x0094:
          return "Quit";
        case 0x0095:
          return "Help";
        case 0x0096:
          return "Media Select Tape";
        case 0x0097:
          return "Media Select Cable";
        case 0x0098:
          return "Media Select Satellite";
        case 0x0099:
          return "Media Select Security";
        case 0x009a:
          return "Media Select Home";
        case 0x009b:
          return "Media Select Call";
        case 0x009c:
          return "Channel Increment";
        case 0x009d:
          return "Channel Decrement";
        case 0x009e:
          return "Media Select SAP";
        case 0x00a0:
          return "VCR Plus";
        case 0x00a1:
          return "Once";
        case 0x00a2:
          return "Daily";
        case 0x00a3:
          return "Weekly";
        case 0x00a4:
          return "Monthly";
        case 0x00b0:
          return "Play";
        case 0x00b1:
          return "Pause";
        case 0x00b2:
          return "Record";
        case 0x00b3:
          return "Fast Forward";
        case 0x00b4:
          return "Rewind";
        case 0x00b5:
          return "Scan Next Track";
        case 0x00b6:
          return "Scan Previous Track";
        case 0x00b7:
          return "Stop";
        case 0x00b8:
          return "Eject";
        case 0x00b9:
          return "Random Play";
        case 0x00ba:
          return "Select Disk";
        case 0x00bb:
          return "Enter Disk";
        case 0x00bc:
          return "Repeat";
        case 0x00bd:
          return "Tracking";
        case 0x00be:
          return "Track Normal";
        case 0x00bf:
          return "Slow Tracking";
        case 0x00c0:
          return "Frame Forward";
        case 0x00c1:
          return "Frame Back";
        case 0x00c2:
          return "Mark";
        case 0x00c3:
          return "Clear Mark";
        case 0x00c4:
          return "Repeat From Mark";
        case 0x00c5:
          return "Return To Mark";
        case 0x00c6:
          return "Search Mark Forward";
        case 0x00c7:
          return "Search Mark Backwards";
        case 0x00c8:
          return "Counter Reset";
        case 0x00c9:
          return "Show Counter";
        case 0x00ca:
          return "Tracking Increment";
        case 0x00cb:
          return "Tracking Decrement";
        case 0x00cc:
          return "Stop/Eject";
        case 0x00cd:
          return "Play/Pause";
        case 0x00ce:
          return "Play/Skip";
        case 0x00cf:
          return "Voice Command";
        case 0x00d0:
          return "Invoke Capture Interface";
        case 0x00d1:
          return "Start or Stop Game Recording";
        case 0x00d2:
          return "Historical Game Capture";
        case 0x00d3:
          return "Capture Game Screenshot";
        case 0x00d4:
          return "Show or Hide Recording Indicator";
        case 0x00d5:
          return "Start or Stop Microphone Capture";
        case 0x00d6:
          return "Start or Stop Camera Capture";
        case 0x00d7:
          return "Start or Stop Game Broadcast";
        case 0x00d8:
          return "Start or Stop Voice Dictation Session";
        case 0x00d9:
          return "Invoke/Dismiss Emoji Picker";
        case 0x00e0:
          return "Volume";
        case 0x00e1:
          return "Balance";
        case 0x00e2:
          return "Mute";
        case 0x00e3:
          return "Bass";
        case 0x00e4:
          return "Treble";
        case 0x00e5:
          return "Bass Boost";
        case 0x00e6:
          return "Surround Mode";
        case 0x00e7:
          return "Loudness";
        case 0x00e8:
          return "MPX";
        case 0x00e9:
          return "Volume Increment";
        case 0x00ea:
          return "Volume Decrement";
        case 0x00f0:
          return "Speed Select";
        case 0x00f1:
          return "Playback Speed";
        case 0x00f2:
          return "Standard Play";
        case 0x00f3:
          return "Long Play";
        case 0x00f4:
          return "Extended Play";
        case 0x00f5:
          return "Slow";
        case 0x0100:
          return "Fan Enable";
        case 0x0101:
          return "Fan Speed";
        case 0x0102:
          return "Light Enable";
        case 0x0103:
          return "Light Illumination Level";
        case 0x0104:
          return "Climate Control Enable";
        case 0x0105:
          return "Room Temperature";
        case 0x0106:
          return "Security Enable";
        case 0x0107:
          return "Fire Alarm";
        case 0x0108:
          return "Police Alarm";
        case 0x0109:
          return "Proximity";
        case 0x010a:
          return "Motion";
        case 0x010b:
          return "Duress Alarm";
        case 0x010c:
          return "Holdup Alarm";
        case 0x010d:
          return "Medical Alarm";
        case 0x0150:
          return "Balance Right";
        case 0x0151:
          return "Balance Left";
        case 0x0152:
          return "Bass Increment";
        case 0x0153:
          return "Bass Decrement";
        case 0x0154:
          return "Treble Increment";
        case 0x0155:
          return "Treble Decrement";
        case 0x0160:
          return "Speaker System";
        case 0x0161:
          return "Channel Left";
        case 0x0162:
          return "Channel Right";
        case 0x0163:
          return "Channel Center";
        case 0x0164:
          return "Channel Front";
        case 0x0165:
          return "Channel Center Front";
        case 0x0166:
          return "Channel Side";
        case 0x0167:
          return "Channel Surround";
        case 0x0168:
          return "Channel Low Frequency Enhancement";
        case 0x0169:
          return "Channel Top";
        case 0x016a:
          return "Channel Unknown";
        case 0x0170:
          return "Sub-channel";
        case 0x0171:
          return "Sub-channel Increment";
        case 0x0172:
          return "Sub-channel Decrement";
        case 0x0173:
          return "Alternate Audio Increment";
        case 0x0174:
          return "Alternate Audio Decrement";
        case 0x0180:
          return "Application Launch Buttons";
        case 0x0181:
          return "AL Launch Button Configuration Tool";
        case 0x0182:
          return "AL Programmable Button Configuration";
        case 0x0183:
          return "AL Consumer Control Configuration";
        case 0x0184:
          return "AL Word Processor";
        case 0x0185:
          return "AL Text Editor";
        case 0x0186:
          return "AL Spreadsheet";
        case 0x0187:
          return "AL Graphics Editor";
        case 0x0188:
          return "AL Presentation App";
        case 0x0189:
          return "AL Database App";
        case 0x018a:
          return "AL Email Reader";
        case 0x018b:
          return "AL Newsreader";
        case 0x018c:
          return "AL Voicemail";
        case 0x018d:
          return "AL Contacts/Address Book";
        case 0x018e:
          return "AL Calendar/Schedule";
        case 0x018f:
          return "AL Task/Project Manager";
        case 0x0190:
          return "AL Log/Journal/Timecard";
        case 0x0191:
          return "AL Checkbook/Finance";
        case 0x0192:
          return "AL Calculator";
        case 0x0193:
          return "AL A/V Capture/Playback";
        case 0x0194:
          return "AL Local Machine Browser";
        case 0x0195:
          return "AL LAN/WAN Browser";
        case 0x0196:
          return "AL Internet Browser";
        case 0x0197:
          return "AL Remote Networking/ISP Connect";
        case 0x0198:
          return "AL Network Conference";
        case 0x0199:
          return "AL Network Chat";
        case 0x019a:
          return "AL Telephony/Dialer";
        case 0x019b:
          return "AL Logon";
        case 0x019c:
          return "AL Logoff";
        case 0x019d:
          return "AL Logon/Logoff";
        case 0x019e:
          return "AL Terminal Lock/Screensaver";
        case 0x019f:
          return "AL Control Panel";
        case 0x01a0:
          return "AL Command Line Processor/Run";
        case 0x01a1:
          return "AL Process/Task Manager";
        case 0x01a2:
          return "AL Select Task/Application";
        case 0x01a3:
          return "AL Next Task/Application";
        case 0x01a4:
          return "AL Previous Task/Application";
        case 0x01a5:
          return "AL Preemptive Halt Task/Application";
        case 0x01a6:
          return "AL Integrated Help Center";
        case 0x01a7:
          return "AL Documents";
        case 0x01a8:
          return "AL Thesaurus";
        case 0x01a9:
          return "AL Dictionary";
        case 0x01aa:
          return "AL Desktop";
        case 0x01ab:
          return "AL Spell Check";
        case 0x01ac:
          return "AL Grammar Check";
        case 0x01ad:
          return "AL Wireless Status";
        case 0x01ae:
          return "AL Keyboard Layout";
        case 0x01af:
          return "AL Virus Protection";
        case 0x01b0:
          return "AL Encryption";
        case 0x01b1:
          return "AL Screen Saver";
        case 0x01b2:
          return "AL Alarms";
        case 0x01b3:
          return "AL Clock";
        case 0x01b4:
          return "AL File Browser";
        case 0x01b5:
          return "AL Power Status";
        case 0x01b6:
          return "AL Image Browser";
        case 0x01b7:
          return "AL Audio Browser";
        case 0x01b8:
          return "AL Movie Browser";
        case 0x01b9:
          return "AL Digital Rights Manager";
        case 0x01ba:
          return "AL Digital Wallet";
        case 0x01bc:
          return "AL Instant Messaging";
        case 0x01bd:
          return "AL OEM Features/Tips/Tutorial Browser";
        case 0x01be:
          return "AL OEM Help";
        case 0x01bf:
          return "AL Online Community";
        case 0x01c0:
          return "AL Entertainment Content Browser";
        case 0x01c1:
          return "AL Online Shopping Browser";
        case 0x01c2:
          return "AL SmartCard Information/Help";
        case 0x01c3:
          return "AL Market Monitor/Finance Browser";
        case 0x01c4:
          return "AL Customized Corporate News Browser";
        case 0x01c5:
          return "AL Online Activity Browser";
        case 0x01c6:
          return "AL Research/Search Browser";
        case 0x01c7:
          return "AL Audio Player";
        case 0x01c8:
          return "AL Message Status";
        case 0x01c9:
          return "AL Contact Sync";
        case 0x01ca:
          return "AL Navigation";
        case 0x01cb:
          return "AL Context-aware desktop assistant";
        case 0x0200:
          return "Generic GUI Application Controls";
        case 0x0201:
          return "AC New";
        case 0x0202:
          return "AC Open";
        case 0x0203:
          return "AC Close";
        case 0x0204:
          return "AC Exit";
        case 0x0205:
          return "AC Maximize";
        case 0x0206:
          return "AC Minimize";
        case 0x0207:
          return "AC Save";
        case 0x0208:
          return "AC Print";
        case 0x0209:
          return "AC Properties";
        case 0x021a:
          return "AC Undo";
        case 0x021b:
          return "AC Copy";
        case 0x021c:
          return "AC Cut";
        case 0x021d:
          return "AC Paste";
        case 0x021e:
          return "AC Select All";
        case 0x021f:
          return "AC Find";
        case 0x0220:
          return "AC Find and Replace";
        case 0x0221:
          return "AC Search";
        case 0x0222:
          return "AC Go To";
        case 0x0223:
          return "AC Home";
        case 0x0224:
          return "AC Back";
        case 0x0225:
          return "AC Forward";
        case 0x0226:
          return "AC Stop";
        case 0x0227:
          return "AC Refresh";
        case 0x0228:
          return "AC Previous Link";
        case 0x0229:
          return "AC Next Link";
        case 0x022a:
          return "AC Bookmarks";
        case 0x022b:
          return "AC History";
        case 0x022c:
          return "AC Subscriptions";
        case 0x022d:
          return "AC Zoom In";
        case 0x022e:
          return "AC Zoom Out";
        case 0x022f:
          return "AC Zoom";
        case 0x0230:
          return "AC Full Screen View";
        case 0x0231:
          return "AC Normal View";
        case 0x0232:
          return "AC View Toggle";
        case 0x0233:
          return "AC Scroll Up";
        case 0x0234:
          return "AC Scroll Down";
        case 0x0235:
          return "AC Scroll";
        case 0x0236:
          return "AC Pan Left";
        case 0x0237:
          return "AC Pan Right";
        case 0x0238:
          return "AC Pan";
        case 0x0239:
          return "AC New Window";
        case 0x023a:
          return "AC Tile Horizontally";
        case 0x023b:
          return "AC Tile Vertically";
        case 0x023c:
          return "AC Format";
        case 0x023d:
          return "AC Edit";
        case 0x023e:
          return "AC Bold";
        case 0x023f:
          return "AC Italics";
        case 0x0240:
          return "AC Underline";
        case 0x0241:
          return "AC Strikethrough";
        case 0x0242:
          return "AC Subscript";
        case 0x0243:
          return "AC Superscript";
        case 0x0244:
          return "AC All Caps";
        case 0x0245:
          return "AC Rotate";
        case 0x0246:
          return "AC Resize";
        case 0x0247:
          return "AC Flip Horizontal";
        case 0x0248:
          return "AC Flip Vertical";
        case 0x0249:
          return "AC Mirror Horizontal";
        case 0x024a:
          return "AC Mirror Vertical";
        case 0x024b:
          return "AC Font Select";
        case 0x024c:
          return "AC Font Color";
        case 0x024d:
          return "AC Font Size";
        case 0x024e:
          return "AC Justify Left";
        case 0x024f:
          return "AC Justify Center H";
        case 0x0250:
          return "AC Justify Right";
        case 0x0251:
          return "AC Justify Block H";
        case 0x0252:
          return "AC Justify Top";
        case 0x0253:
          return "AC Justify Center V";
        case 0x0254:
          return "AC Justify Bottom";
        case 0x0255:
          return "AC Justify Block V";
        case 0x0256:
          return "AC Indent Decrease";
        case 0x0257:
          return "AC Indent Increase";
        case 0x0258:
          return "AC Numbered List";
        case 0x0259:
          return "AC Restart Numbering";
        case 0x025a:
          return "AC Bulleted List";
        case 0x025b:
          return "AC Promote";
        case 0x025c:
          return "AC Demote";
        case 0x025d:
          return "AC Yes";
        case 0x025e:
          return "AC No";
        case 0x025f:
          return "AC Cancel";
        case 0x0260:
          return "AC Catalog";
        case 0x0261:
          return "AC Buy/Checkout";
        case 0x0262:
          return "AC Add to Cart";
        case 0x0263:
          return "AC Expand";
        case 0x0264:
          return "AC Expand All";
        case 0x0265:
          return "AC Collapse";
        case 0x0266:
          return "AC Collapse All";
        case 0x0267:
          return "AC Print Preview";
        case 0x0268:
          return "AC Paste Special";
        case 0x0269:
          return "AC Insert Mode";
        case 0x026a:
          return "AC Delete";
        case 0x026b:
          return "AC Lock";
        case 0x026c:
          return "AC Unlock";
        case 0x026d:
          return "AC Protect";
        case 0x026e:
          return "AC Unprotect";
        case 0x026f:
          return "AC Attach Comment";
        case 0x0270:
          return "AC Delete Comment";
        case 0x0271:
          return "AC View Comment";
        case 0x0272:
          return "AC Select Word";
        case 0x0273:
          return "AC Select Sentence";
        case 0x0274:
          return "AC Select Paragraph";
        case 0x0275:
          return "AC Select Column";
        case 0x0276:
          return "AC Select Row";
        case 0x0277:
          return "AC Select Table";
        case 0x0278:
          return "AC Select Object";
        case 0x0279:
          return "AC Redo/Repeat";
        case 0x027a:
          return "AC Sort";
        case 0x027b:
          return "AC Sort Ascending";
        case 0x027c:
          return "AC Sort Descending";
        case 0x027d:
          return "AC Filter";
        case 0x027e:
          return "AC Set Clock";
        case 0x027f:
          return "AC View Clock";
        case 0x0280:
          return "AC Select Time Zone";
        case 0x0281:
          return "AC Edit Time Zones";
        case 0x0282:
          return "AC Set Alarm";
        case 0x0283:
          return "AC Clear Alarm";
        case 0x0284:
          return "AC Snooze Alarm";
        case 0x0285:
          return "AC Reset Alarm";
        case 0x0286:
          return "AC Synchronize";
        case 0x0287:
          return "AC Send/Receive";
        case 0x0288:
          return "AC Send To";
        case 0x0289:
          return "AC Reply";
        case 0x028a:
          return "AC Reply All";
        case 0x028b:
          return "AC Forward Msg";
        case 0x028c:
          return "AC Send";
        case 0x028d:
          return "AC Attach File";
        case 0x028e:
          return "AC Upload";
        case 0x028f:
          return "AC Download (Save Target As)";
        case 0x0290:
          return "AC Set Borders";
        case 0x0291:
          return "AC Insert Row";
        case 0x0292:
          return "AC Insert Column";
        case 0x0293:
          return "AC Insert File";
        case 0x0294:
          return "AC Insert Picture";
        case 0x0295:
          return "AC Insert Object";
        case 0x0296:
          return "AC Insert Symbol";
        case 0x0297:
          return "AC Save and Close";
        case 0x0298:
          return "AC Rename";
        case 0x0299:
          return "AC Merge";
        case 0x029a:
          return "AC Split";
        case 0x029b:
          return "AC Distribute Horizontally";
        case 0x029c:
          return "AC Distribute Vertically";
        case 0x029d:
          return "AC Next Keyboard Layout Select";
        case 0x029e:
          return "AC Navigation Guidance";
        case 0x029f:
          return "AC Desktop Show All Windows";
        case 0x02a0:
          return "AC Soft Key Left";
        case 0x02a1:
          return "AC Soft Key Right";
        case 0x02a2:
          return "AC Desktop Show All Applications";
        case 0x02b0:
          return "AC Idle Keep Alive";
        case 0x02c0:
          return "Extended Keyboard Attributes Collection";
        case 0x02c1:
          return "Keyboard Form Factor";
        case 0x02c2:
          return "Keyboard Key Type";
        case 0x02c3:
          return "Keyboard Physical Layout";
        case 0x02c4:
          return "Vendor-Specific Keyboard Physical Layout";
        case 0x02c5:
          return "Keyboard IETF Language Tag Index";
        case 0x02c6:
          return "Implemented Keyboard Input Assist Controls";
        case 0x02c7:
          return "Keyboard Input Assist Previous";
        case 0x02c8:
          return "Keyboard Input Assist Next";
        case 0x02c9:
          return "Keyboard Input Assist Previous Group";
        case 0x02ca:
          return "Keyboard Input Assist Next Group";
        case 0x02cb:
          return "Keyboard Input Assist Accept";
        case 0x02cc:
          return "Keyboard Input Assist Cancel";
        case 0x02d0:
          return "Privacy Screen Toggle";
        case 0x02d1:
          return "Privacy Screen Level Decrement";
        case 0x02d2:
          return "Privacy Screen Level Increment";
        case 0x02d3:
          return "Privacy Screen Level Minimum";
        case 0x02d4:
          return "Privacy Screen Level Maximum";
        case 0x0500:
          return "Contact Edited";
        case 0x0501:
          return "Contact Added";
        case 0x0502:
          return "Contact Record Active";
        case 0x0503:
          return "Contact Index";
        case 0x0504:
          return "Contact Nickname";
        case 0x0505:
          return "Contact First Name";
        case 0x0506:
          return "Contact Last Name";
        case 0x0507:
          return "Contact Full Name";
        case 0x0508:
          return "Contact Phone Number Personal";
        case 0x0509:
          return "Contact Phone Number Business";
        case 0x050a:
          return "Contact Phone Number Mobile";
        case 0x050b:
          return "Contact Phone Number Pager";
        case 0x050c:
          return "Contact Phone Number Fax";
        case 0x050d:
          return "Contact Phone Number Other";
        case 0x050e:
          return "Contact Email Personal";
        case 0x050f:
          return "Contact Email Business";
        case 0x0510:
          return "Contact Email Other";
        case 0x0511:
          return "Contact Email Main";
        case 0x0512:
          return "Contact Speed Dial Number";
        case 0x0513:
          return "Contact Status Flag";
        case 0x0514:
          return "Contact Misc.";
        case 0x0515:
          return "Keyboard Brightness Next";
        case 0x0516:
          return "Keyboard Brightness Previous";
        case 0x0517:
          return "Keyboard Backlight Level Suggestion";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class consumer : std::uint16_t {
  CONSUMER_CONTROL = 0x0001,
  NUMERIC_KEY_PAD = 0x0002,
  PROGRAMMABLE_BUTTONS = 0x0003,
  MICROPHONE = 0x0004,
  HEADPHONE = 0x0005,
  GRAPHIC_EQUALIZER = 0x0006,
  KEYBOARD_BACKLIGHT = 0x0007,
  _10 = 0x0020,
  _100 = 0x0021,
  AM_PM = 0x0022,
  POWER = 0x0030,
  RESET = 0x0031,
  SLEEP = 0x0032,
  SLEEP_AFTER = 0x0033,
  SLEEP_MODE = 0x0034,
  ILLUMINATION = 0x0035,
  FUNCTION_BUTTONS = 0x0036,
  MENU = 0x0040,
  MENU_PICK = 0x0041,
  MENU_UP = 0x0042,
  MENU_DOWN = 0x0043,
  MENU_LEFT = 0x0044,
  MENU_RIGHT = 0x0045,
  MENU_ESCAPE = 0x0046,
  MENU_VALUE_INCREASE = 0x0047,
  MENU_VALUE_DECREASE = 0x0048,
  DATA_ON_SCREEN = 0x0060,
  CLOSED_CAPTION = 0x0061,
  CLOSED_CAPTION_SELECT = 0x0062,
  VCR_TV = 0x0063,
  BROADCAST_MODE = 0x0064,
  SNAPSHOT = 0x0065,
  STILL = 0x0066,
  PICTURE_IN_PICTURE_TOGGLE = 0x0067,
  PICTURE_IN_PICTURE_SWAP = 0x0068,
  RED_MENU_BUTTON = 0x0069,
  GREEN_MENU_BUTTON = 0x006a,
  BLUE_MENU_BUTTON = 0x006b,
  YELLOW_MENU_BUTTON = 0x006c,
  ASPECT = 0x006d,
  _3D_MODE_SELECT = 0x006e,
  DISPLAY_BRIGHTNESS_INCREMENT = 0x006f,
  DISPLAY_BRIGHTNESS_DECREMENT = 0x0070,
  DISPLAY_BRIGHTNESS = 0x0071,
  DISPLAY_BACKLIGHT_TOGGLE = 0x0072,
  DISPLAY_SET_BRIGHTNESS_TO_MINIMUM = 0x0073,
  DISPLAY_SET_BRIGHTNESS_TO_MAXIMUM = 0x0074,
  DISPLAY_SET_AUTO_BRIGHTNESS = 0x0075,
  CAMERA_ACCESS_ENABLED = 0x0076,
  CAMERA_ACCESS_DISABLED = 0x0077,
  CAMERA_ACCESS_TOGGLE = 0x0078,
  KEYBOARD_BRIGHTNESS_INCREMENT = 0x0079,
  KEYBOARD_BRIGHTNESS_DECREMENT = 0x007a,
  KEYBOARD_BACKLIGHT_SET_LEVEL = 0x007b,
  KEYBOARD_BACKLIGHT_TOGGLE = 0x007c,
  KEYBOARD_BACKLIGHT_SET_MINIMUM = 0x007d,
  KEYBOARD_BACKLIGHT_SET_MAXIMUM = 0x007e,
  KEYBOARD_BACKLIGHT_AUTO = 0x007f,
  SELECTION = 0x0080,
  ASSIGN_SELECTION = 0x0081,
  MODE_STEP = 0x0082,
  RECALL_LAST = 0x0083,
  ENTER_CHANNEL = 0x0084,
  ORDER_MOVIE = 0x0085,
  CHANNEL = 0x0086,
  MEDIA_SELECTION = 0x0087,
  MEDIA_SELECT_COMPUTER = 0x0088,
  MEDIA_SELECT_TV = 0x0089,
  MEDIA_SELECT_WWW = 0x008a,
  MEDIA_SELECT_DVD = 0x008b,
  MEDIA_SELECT_TELEPHONE = 0x008c,
  MEDIA_SELECT_PROGRAM_GUIDE = 0x008d,
  MEDIA_SELECT_VIDEO_PHONE = 0x008e,
  MEDIA_SELECT_GAMES = 0x008f,
  MEDIA_SELECT_MESSAGES = 0x0090,
  MEDIA_SELECT_CD = 0x0091,
  MEDIA_SELECT_VCR = 0x0092,
  MEDIA_SELECT_TUNER = 0x0093,
  QUIT = 0x0094,
  HELP = 0x0095,
  MEDIA_SELECT_TAPE = 0x0096,
  MEDIA_SELECT_CABLE = 0x0097,
  MEDIA_SELECT_SATELLITE = 0x0098,
  MEDIA_SELECT_SECURITY = 0x0099,
  MEDIA_SELECT_HOME = 0x009a,
  MEDIA_SELECT_CALL = 0x009b,
  CHANNEL_INCREMENT = 0x009c,
  CHANNEL_DECREMENT = 0x009d,
  MEDIA_SELECT_SAP = 0x009e,
  VCR_PLUS = 0x00a0,
  ONCE = 0x00a1,
  DAILY = 0x00a2,
  WEEKLY = 0x00a3,
  MONTHLY = 0x00a4,
  PLAY = 0x00b0,
  PAUSE = 0x00b1,
  RECORD = 0x00b2,
  FAST_FORWARD = 0x00b3,
  REWIND = 0x00b4,
  SCAN_NEXT_TRACK = 0x00b5,
  SCAN_PREVIOUS_TRACK = 0x00b6,
  STOP = 0x00b7,
  EJECT = 0x00b8,
  RANDOM_PLAY = 0x00b9,
  SELECT_DISK = 0x00ba,
  ENTER_DISK = 0x00bb,
  REPEAT = 0x00bc,
  TRACKING = 0x00bd,
  TRACK_NORMAL = 0x00be,
  SLOW_TRACKING = 0x00bf,
  FRAME_FORWARD = 0x00c0,
  FRAME_BACK = 0x00c1,
  MARK = 0x00c2,
  CLEAR_MARK = 0x00c3,
  REPEAT_FROM_MARK = 0x00c4,
  RETURN_TO_MARK = 0x00c5,
  SEARCH_MARK_FORWARD = 0x00c6,
  SEARCH_MARK_BACKWARDS = 0x00c7,
  COUNTER_RESET = 0x00c8,
  SHOW_COUNTER = 0x00c9,
  TRACKING_INCREMENT = 0x00ca,
  TRACKING_DECREMENT = 0x00cb,
  STOP_EJECT = 0x00cc,
  PLAY_PAUSE = 0x00cd,
  PLAY_SKIP = 0x00ce,
  VOICE_COMMAND = 0x00cf,
  INVOKE_CAPTURE_INTERFACE = 0x00d0,
  START_OR_STOP_GAME_RECORDING = 0x00d1,
  HISTORICAL_GAME_CAPTURE = 0x00d2,
  CAPTURE_GAME_SCREENSHOT = 0x00d3,
  SHOW_OR_HIDE_RECORDING_INDICATOR = 0x00d4,
  START_OR_STOP_MICROPHONE_CAPTURE = 0x00d5,
  START_OR_STOP_CAMERA_CAPTURE = 0x00d6,
  START_OR_STOP_GAME_BROADCAST = 0x00d7,
  START_OR_STOP_VOICE_DICTATION_SESSION = 0x00d8,
  INVOKE_DISMISS_EMOJI_PICKER = 0x00d9,
  VOLUME = 0x00e0,
  BALANCE = 0x00e1,
  MUTE = 0x00e2,
  BASS = 0x00e3,
  TREBLE = 0x00e4,
  BASS_BOOST = 0x00e5,
  SURROUND_MODE = 0x00e6,
  LOUDNESS = 0x00e7,
  MPX = 0x00e8,
  VOLUME_INCREMENT = 0x00e9,
  VOLUME_DECREMENT = 0x00ea,
  SPEED_SELECT = 0x00f0,
  PLAYBACK_SPEED = 0x00f1,
  STANDARD_PLAY = 0x00f2,
  LONG_PLAY = 0x00f3,
  EXTENDED_PLAY = 0x00f4,
  SLOW = 0x00f5,
  FAN_ENABLE = 0x0100,
  FAN_SPEED = 0x0101,
  LIGHT_ENABLE = 0x0102,
  LIGHT_ILLUMINATION_LEVEL = 0x0103,
  CLIMATE_CONTROL_ENABLE = 0x0104,
  ROOM_TEMPERATURE = 0x0105,
  SECURITY_ENABLE = 0x0106,
  FIRE_ALARM = 0x0107,
  POLICE_ALARM = 0x0108,
  PROXIMITY = 0x0109,
  MOTION = 0x010a,
  DURESS_ALARM = 0x010b,
  HOLDUP_ALARM = 0x010c,
  MEDICAL_ALARM = 0x010d,
  BALANCE_RIGHT = 0x0150,
  BALANCE_LEFT = 0x0151,
  BASS_INCREMENT = 0x0152,
  BASS_DECREMENT = 0x0153,
  TREBLE_INCREMENT = 0x0154,
  TREBLE_DECREMENT = 0x0155,
  SPEAKER_SYSTEM = 0x0160,
  CHANNEL_LEFT = 0x0161,
  CHANNEL_RIGHT = 0x0162,
  CHANNEL_CENTER = 0x0163,
  CHANNEL_FRONT = 0x0164,
  CHANNEL_CENTER_FRONT = 0x0165,
  CHANNEL_SIDE = 0x0166,
  CHANNEL_SURROUND = 0x0167,
  CHANNEL_LOW_FREQUENCY_ENHANCEMENT = 0x0168,
  CHANNEL_TOP = 0x0169,
  CHANNEL_UNKNOWN = 0x016a,
  SUB_CHANNEL = 0x0170,
  SUB_CHANNEL_INCREMENT = 0x0171,
  SUB_CHANNEL_DECREMENT = 0x0172,
  ALTERNATE_AUDIO_INCREMENT = 0x0173,
  ALTERNATE_AUDIO_DECREMENT = 0x0174,
  APPLICATION_LAUNCH_BUTTONS = 0x0180,
  AL_LAUNCH_BUTTON_CONFIGURATION_TOOL = 0x0181,
  AL_PROGRAMMABLE_BUTTON_CONFIGURATION = 0x0182,
  AL_CONSUMER_CONTROL_CONFIGURATION = 0x0183,
  AL_WORD_PROCESSOR = 0x0184,
  AL_TEXT_EDITOR = 0x0185,
  AL_SPREADSHEET = 0x0186,
  AL_GRAPHICS_EDITOR = 0x0187,
  AL_PRESENTATION_APP = 0x0188,
  AL_DATABASE_APP = 0x0189,
  AL_EMAIL_READER = 0x018a,
  AL_NEWSREADER = 0x018b,
  AL_VOICEMAIL = 0x018c,
  AL_CONTACTS_ADDRESS_BOOK = 0x018d,
  AL_CALENDAR_SCHEDULE = 0x018e,
  AL_TASK_PROJECT_MANAGER = 0x018f,
  AL_LOG_JOURNAL_TIMECARD = 0x0190,
  AL_CHECKBOOK_FINANCE = 0x0191,
  AL_CALCULATOR = 0x0192,
  AL_A_V_CAPTURE_PLAYBACK = 0x0193,
  AL_LOCAL_MACHINE_BROWSER = 0x0194,
  AL_LAN_WAN_BROWSER = 0x0195,
  AL_INTERNET_BROWSER = 0x0196,
  AL_REMOTE_NETWORKING_ISP_CONNECT = 0x0197,
  AL_NETWORK_CONFERENCE = 0x0198,
  AL_NETWORK_CHAT = 0x0199,
  AL_TELEPHONY_DIALER = 0x019a,
  AL_LOGON = 0x019b,
  AL_LOGOFF = 0x019c,
  AL_LOGON_LOGOFF = 0x019d,
  AL_TERMINAL_LOCK_SCREENSAVER = 0x019e,
  AL_CONTROL_PANEL = 0x019f,
  AL_COMMAND_LINE_PROCESSOR_RUN = 0x01a0,
  AL_PROCESS_TASK_MANAGER = 0x01a1,
  AL_SELECT_TASK_APPLICATION = 0x01a2,
  AL_NEXT_TASK_APPLICATION = 0x01a3,
  AL_PREVIOUS_TASK_APPLICATION = 0x01a4,
  AL_PREEMPTIVE_HALT_TASK_APPLICATION = 0x01a5,
  AL_INTEGRATED_HELP_CENTER = 0x01a6,
  AL_DOCUMENTS = 0x01a7,
  AL_THESAURUS = 0x01a8,
  AL_DICTIONARY = 0x01a9,
  AL_DESKTOP = 0x01aa,
  AL_SPELL_CHECK = 0x01ab,
  AL_GRAMMAR_CHECK = 0x01ac,
  AL_WIRELESS_STATUS = 0x01ad,
  AL_KEYBOARD_LAYOUT = 0x01ae,
  AL_VIRUS_PROTECTION = 0x01af,
  AL_ENCRYPTION = 0x01b0,
  AL_SCREEN_SAVER = 0x01b1,
  AL_ALARMS = 0x01b2,
  AL_CLOCK = 0x01b3,
  AL_FILE_BROWSER = 0x01b4,
  AL_POWER_STATUS = 0x01b5,
  AL_IMAGE_BROWSER = 0x01b6,
  AL_AUDIO_BROWSER = 0x01b7,
  AL_MOVIE_BROWSER = 0x01b8,
  AL_DIGITAL_RIGHTS_MANAGER = 0x01b9,
  AL_DIGITAL_WALLET = 0x01ba,
  AL_INSTANT_MESSAGING = 0x01bc,
  AL_OEM_FEATURES_TIPS_TUTORIAL_BROWSER = 0x01bd,
  AL_OEM_HELP = 0x01be,
  AL_ONLINE_COMMUNITY = 0x01bf,
  AL_ENTERTAINMENT_CONTENT_BROWSER = 0x01c0,
  AL_ONLINE_SHOPPING_BROWSER = 0x01c1,
  AL_SMARTCARD_INFORMATION_HELP = 0x01c2,
  AL_MARKET_MONITOR_FINANCE_BROWSER = 0x01c3,
  AL_CUSTOMIZED_CORPORATE_NEWS_BROWSER = 0x01c4,
  AL_ONLINE_ACTIVITY_BROWSER = 0x01c5,
  AL_RESEARCH_SEARCH_BROWSER = 0x01c6,
  AL_AUDIO_PLAYER = 0x01c7,
  AL_MESSAGE_STATUS = 0x01c8,
  AL_CONTACT_SYNC = 0x01c9,
  AL_NAVIGATION = 0x01ca,
  AL_CONTEXT_AWARE_DESKTOP_ASSISTANT = 0x01cb,
  GENERIC_GUI_APPLICATION_CONTROLS = 0x0200,
  AC_NEW = 0x0201,
  AC_OPEN = 0x0202,
  AC_CLOSE = 0x0203,
  AC_EXIT = 0x0204,
  AC_MAXIMIZE = 0x0205,
  AC_MINIMIZE = 0x0206,
  AC_SAVE = 0x0207,
  AC_PRINT = 0x0208,
  AC_PROPERTIES = 0x0209,
  AC_UNDO = 0x021a,
  AC_COPY = 0x021b,
  AC_CUT = 0x021c,
  AC_PASTE = 0x021d,
  AC_SELECT_ALL = 0x021e,
  AC_FIND = 0x021f,
  AC_FIND_AND_REPLACE = 0x0220,
  AC_SEARCH = 0x0221,
  AC_GO_TO = 0x0222,
  AC_HOME = 0x0223,
  AC_BACK = 0x0224,
  AC_FORWARD = 0x0225,
  AC_STOP = 0x0226,
  AC_REFRESH = 0x0227,
  AC_PREVIOUS_LINK = 0x0228,
  AC_NEXT_LINK = 0x0229,
  AC_BOOKMARKS = 0x022a,
  AC_HISTORY = 0x022b,
  AC_SUBSCRIPTIONS = 0x022c,
  AC_ZOOM_IN = 0x022d,
  AC_ZOOM_OUT = 0x022e,
  AC_ZOOM = 0x022f,
  AC_FULL_SCREEN_VIEW = 0x0230,
  AC_NORMAL_VIEW = 0x0231,
  AC_VIEW_TOGGLE = 0x0232,
  AC_SCROLL_UP = 0x0233,
  AC_SCROLL_DOWN = 0x0234,
  AC_SCROLL = 0x0235,
  AC_PAN_LEFT = 0x0236,
  AC_PAN_RIGHT = 0x0237,
  AC_PAN = 0x0238,
  AC_NEW_WINDOW = 0x0239,
  AC_TILE_HORIZONTALLY = 0x023a,
  AC_TILE_VERTICALLY = 0x023b,
  AC_FORMAT = 0x023c,
  AC_EDIT = 0x023d,
  AC_BOLD = 0x023e,
  AC_ITALICS = 0x023f,
  AC_UNDERLINE = 0x0240,
  AC_STRIKETHROUGH = 0x0241,
  AC_SUBSCRIPT = 0x0242,
  AC_SUPERSCRIPT = 0x0243,
  AC_ALL_CAPS = 0x0244,
  AC_ROTATE = 0x0245,
  AC_RESIZE = 0x0246,
  AC_FLIP_HORIZONTAL = 0x0247,
  AC_FLIP_VERTICAL = 0x0248,
  AC_MIRROR_HORIZONTAL = 0x0249,
  AC_MIRROR_VERTICAL = 0x024a,
  AC_FONT_SELECT = 0x024b,
  AC_FONT_COLOR = 0x024c,
  AC_FONT_SIZE = 0x024d,
  AC_JUSTIFY_LEFT = 0x024e,
  AC_JUSTIFY_CENTER_H = 0x024f,
  AC_JUSTIFY_RIGHT = 0x0250,
  AC_JUSTIFY_BLOCK_H = 0x0251,
  AC_JUSTIFY_TOP = 0x0252,
  AC_JUSTIFY_CENTER_V = 0x0253,
  AC_JUSTIFY_BOTTOM = 0x0254,
  AC_JUSTIFY_BLOCK_V = 0x0255,
  AC_INDENT_DECREASE = 0x0256,
  AC_INDENT_INCREASE = 0x0257,
  AC_NUMBERED_LIST = 0x0258,
  AC_RESTART_NUMBERING = 0x0259,
  AC_BULLETED_LIST = 0x025a,
  AC_PROMOTE = 0x025b,
  AC_DEMOTE = 0x025c,
  AC_YES = 0x025d,
  AC_NO = 0x025e,
  AC_CANCEL = 0x025f,
  AC_CATALOG = 0x0260,
  AC_BUY_CHECKOUT = 0x0261,
  AC_ADD_TO_CART = 0x0262,
  AC_EXPAND = 0x0263,
  AC_EXPAND_ALL = 0x0264,
  AC_COLLAPSE = 0x0265,
  AC_COLLAPSE_ALL = 0x0266,
  AC_PRINT_PREVIEW = 0x0267,
  AC_PASTE_SPECIAL = 0x0268,
  AC_INSERT_MODE = 0x0269,
  AC_DELETE = 0x026a,
  AC_LOCK = 0x026b,
  AC_UNLOCK = 0x026c,
  AC_PROTECT = 0x026d,
  AC_UNPROTECT = 0x026e,
  AC_ATTACH_COMMENT = 0x026f,
  AC_DELETE_COMMENT = 0x0270,
  AC_VIEW_COMMENT = 0x0271,
  AC_SELECT_WORD = 0x0272,
  AC_SELECT_SENTENCE = 0x0273,
  AC_SELECT_PARAGRAPH = 0x0274,
  AC_SELECT_COLUMN = 0x0275,
  AC_SELECT_ROW = 0x0276,
  AC_SELECT_TABLE = 0x0277,
  AC_SELECT_OBJECT = 0x0278,
  AC_REDO_REPEAT = 0x0279,
  AC_SORT = 0x027a,
  AC_SORT_ASCENDING = 0x027b,
  AC_SORT_DESCENDING = 0x027c,
  AC_FILTER = 0x027d,
  AC_SET_CLOCK = 0x027e,
  AC_VIEW_CLOCK = 0x027f,
  AC_SELECT_TIME_ZONE = 0x0280,
  AC_EDIT_TIME_ZONES = 0x0281,
  AC_SET_ALARM = 0x0282,
  AC_CLEAR_ALARM = 0x0283,
  AC_SNOOZE_ALARM = 0x0284,
  AC_RESET_ALARM = 0x0285,
  AC_SYNCHRONIZE = 0x0286,
  AC_SEND_RECEIVE = 0x0287,
  AC_SEND_TO = 0x0288,
  AC_REPLY = 0x0289,
  AC_REPLY_ALL = 0x028a,
  AC_FORWARD_MSG = 0x028b,
  AC_SEND = 0x028c,
  AC_ATTACH_FILE = 0x028d,
  AC_UPLOAD = 0x028e,
  AC_DOWNLOAD_SAVE_TARGET_AS = 0x028f,
  AC_SET_BORDERS = 0x0290,
  AC_INSERT_ROW = 0x0291,
  AC_INSERT_COLUMN = 0x0292,
  AC_INSERT_FILE = 0x0293,
  AC_INSERT_PICTURE = 0x0294,
  AC_INSERT_OBJECT = 0x0295,
  AC_INSERT_SYMBOL = 0x0296,
  AC_SAVE_AND_CLOSE = 0x0297,
  AC_RENAME = 0x0298,
  AC_MERGE = 0x0299,
  AC_SPLIT = 0x029a,
  AC_DISTRIBUTE_HORIZONTALLY = 0x029b,
  AC_DISTRIBUTE_VERTICALLY = 0x029c,
  AC_NEXT_KEYBOARD_LAYOUT_SELECT = 0x029d,
  AC_NAVIGATION_GUIDANCE = 0x029e,
  AC_DESKTOP_SHOW_ALL_WINDOWS = 0x029f,
  AC_SOFT_KEY_LEFT = 0x02a0,
  AC_SOFT_KEY_RIGHT = 0x02a1,
  AC_DESKTOP_SHOW_ALL_APPLICATIONS = 0x02a2,
  AC_IDLE_KEEP_ALIVE = 0x02b0,
  EXTENDED_KEYBOARD_ATTRIBUTES_COLLECTION = 0x02c0,
  KEYBOARD_FORM_FACTOR = 0x02c1,
  KEYBOARD_KEY_TYPE = 0x02c2,
  KEYBOARD_PHYSICAL_LAYOUT = 0x02c3,
  VENDOR_SPECIFIC_KEYBOARD_PHYSICAL_LAYOUT = 0x02c4,
  KEYBOARD_IETF_LANGUAGE_TAG_INDEX = 0x02c5,
  IMPLEMENTED_KEYBOARD_INPUT_ASSIST_CONTROLS = 0x02c6,
  KEYBOARD_INPUT_ASSIST_PREVIOUS = 0x02c7,
  KEYBOARD_INPUT_ASSIST_NEXT = 0x02c8,
  KEYBOARD_INPUT_ASSIST_PREVIOUS_GROUP = 0x02c9,
  KEYBOARD_INPUT_ASSIST_NEXT_GROUP = 0x02ca,
  KEYBOARD_INPUT_ASSIST_ACCEPT = 0x02cb,
  KEYBOARD_INPUT_ASSIST_CANCEL = 0x02cc,
  PRIVACY_SCREEN_TOGGLE = 0x02d0,
  PRIVACY_SCREEN_LEVEL_DECREMENT = 0x02d1,
  PRIVACY_SCREEN_LEVEL_INCREMENT = 0x02d2,
  PRIVACY_SCREEN_LEVEL_MINIMUM = 0x02d3,
  PRIVACY_SCREEN_LEVEL_MAXIMUM = 0x02d4,
  CONTACT_EDITED = 0x0500,
  CONTACT_ADDED = 0x0501,
  CONTACT_RECORD_ACTIVE = 0x0502,
  CONTACT_INDEX = 0x0503,
  CONTACT_NICKNAME = 0x0504,
  CONTACT_FIRST_NAME = 0x0505,
  CONTACT_LAST_NAME = 0x0506,
  CONTACT_FULL_NAME = 0x0507,
  CONTACT_PHONE_NUMBER_PERSONAL = 0x0508,
  CONTACT_PHONE_NUMBER_BUSINESS = 0x0509,
  CONTACT_PHONE_NUMBER_MOBILE = 0x050a,
  CONTACT_PHONE_NUMBER_PAGER = 0x050b,
  CONTACT_PHONE_NUMBER_FAX = 0x050c,
  CONTACT_PHONE_NUMBER_OTHER = 0x050d,
  CONTACT_EMAIL_PERSONAL = 0x050e,
  CONTACT_EMAIL_BUSINESS = 0x050f,
  CONTACT_EMAIL_OTHER = 0x0510,
  CONTACT_EMAIL_MAIN = 0x0511,
  CONTACT_SPEED_DIAL_NUMBER = 0x0512,
  CONTACT_STATUS_FLAG = 0x0513,
  CONTACT_MISC = 0x0514,
  KEYBOARD_BRIGHTNESS_NEXT = 0x0515,
  KEYBOARD_BRIGHTNESS_PREVIOUS = 0x0516,
  KEYBOARD_BACKLIGHT_LEVEL_SUGGESTION = 0x0517,
};
} // namespace hid::page

#endif // __HID_PAGE_CONSUMER_HPP_
