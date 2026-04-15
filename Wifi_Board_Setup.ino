// #include <ESP8266WiFi.h>
// #include <ESP8266WebServer.h>

// ESP8266WebServer server(80);

// /*
//  * =========================================================================
//  * CUBESAT TEST MANEUVERS (9 Total, 3 per Configuration)
//  * Note: To reverse the maneuver (move backwards or rotate CCW), simply 
//  * invert the sign of the non-zero target value (e.g., x=-0.5).
//  * =========================================================================
//  * * --- CONFIGURATION T1 ---
//  * 1. X-Axis Forward: http://192.168.4.1/?config=T1&x=0.15&y=0.0&z=0.0&roll=0.0&pitch=0.0&yaw=0.0
//  * 2. Y-Axis Forward: http://192.168.4.1/?config=T1&x=0.0&y=0.15&z=0.0&roll=0.0&pitch=0.0&yaw=0.0
//  * 3. Z-Axis Rot CW:  http://192.168.4.1/?config=T1&x=0.0&y=0.0&z=0.0&roll=0.0&pitch=0.0&yaw=0.5
//  * * --- CONFIGURATION T2 ---
//  * 4. X-Axis Forward: http://192.168.4.1/?config=T2&x=0.15&y=0.0&z=0.0&roll=0.0&pitch=0.0&yaw=0.0
//  * 5. Y-Axis Forward: http://192.168.4.1/?config=T2&x=0.0&y=0.0&z=0.15&roll=0.0&pitch=0.0&yaw=0.0
//  * 6. Z-Axis Rot CW:  http://192.168.4.1/?config=T2&x=0.0&y=0.0&z=0.0&roll=0.0&pitch=0.5&yaw=0.0
//  * * --- CONFIGURATION T3 ---
//  * 7. X-Axis Forward: http://192.168.4.1/?config=T3&x=0.0&y=0.15&z=0.0&roll=0.0&pitch=0.0&yaw=0.0
//  * 8. Y-Axis Forward: http://192.168.4.1/?config=T3&x=0.0&y=0.0&z=0.0&roll=0.15&pitch=0.0&yaw=0.0
//  * 9. Z-Axis Rot CW:  http://192.168.4.1/?config=T3&x=0.0&y=0.0&z=0.0&roll=0.5&pitch=0.0&yaw=0.0
//  */

// void handleRoot() {
//   // Get config and 6DOF pose commands from URL
//   String config_str = server.arg("config");
//   String x_str = server.arg("x");
//   String y_str = server.arg("y");
//   String z_str = server.arg("z");
//   String roll_str = server.arg("roll");
//   String pitch_str = server.arg("pitch");
//   String yaw_str = server.arg("yaw");

//   // Default arguments if missing
//   if (config_str == "") config_str = "T1";
//   if (x_str == "") x_str = "0.0";
//   if (y_str == "") y_str = "0.0";
//   if (z_str == "") z_str = "0.0";
//   if (roll_str == "") roll_str = "0.0";
//   if (pitch_str == "") pitch_str = "0.0";
//   if (yaw_str == "") yaw_str = "0.0";

//   // Format payload as <config,x,y,z,roll,pitch,yaw>
//   String payload = "<" + config_str + "," + x_str + "," + y_str + "," + z_str + "," + roll_str + "," + pitch_str + "," + yaw_str + ">";
  
//   // Send to Mega via TX
//   Serial.println(payload);
//   server.send(200, "text/plain", "Config & Pose Updated: " + payload);
// }

// void setup() {
//   Serial.begin(115200);
//   WiFi.softAP("NodeMCU-LED", "12345678");
//   Serial.println("Wi-Fi started.");
  
//   server.on("/", handleRoot);
//   server.begin();
//   Serial.println("HTTP server started.");
// }

// void loop() {
//   server.handleClient();
// }