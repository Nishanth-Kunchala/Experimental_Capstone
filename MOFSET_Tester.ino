// /*
//  * MOSFET & Solenoid Sequential Test Script
//  * Cycles through all 12 thruster pins one by one.
//  */

// // Define the number of thrusters and their corresponding pins based on original code
// const int NUM_THRUSTERS = 12; 
// const int thrusterPins[12] = {43, 47, 49, 53, 45, 51, 48, 52, 42, 46, 50, 44};

// // const int NUM_THRUSTERS = 1;
// // const int thrusterPins[1] = {51};

// // Define how long each solenoid should stay open (in milliseconds)
// const int testDelay = 50; 

// void setup() {
//   // Start the serial monitor for debugging
//   Serial.begin(115200);
//   Serial.println("Starting MOSFET Test Sequence...");
//   Serial.println("Initializing pins...");

//   // Set all thruster pins to OUTPUT and ensure they start LOW (Off)
//   for (int i = 0; i < NUM_THRUSTERS; i++) {
//     pinMode(thrusterPins[i], OUTPUT);
//     digitalWrite(thrusterPins[i], LOW);
//   }
  
//   delay(2000); // Brief pause before the test begins
//   Serial.println("Sequence beginning...");
// }

// void loop() {
//   // Loop through each thruster pin sequentially
//   for (int i = 0; i < NUM_THRUSTERS; i++) {
    
//     Serial.print("Firing Thruster on Pin: ");
//     Serial.println(thrusterPins[i]);

//     // Turn the MOSFET ON (solenoid opens)
//     digitalWrite(thrusterPins[i], HIGH);
    
//     // Keep it on for the specified delay
//     delay(3000);
    
//     Serial.print("unFiring Thruster on Pin: ");
//     Serial.println(thrusterPins[i]);
//     // Turn the MOSFET OFF (solenoid closes)
//     digitalWrite(thrusterPins[i], LOW);
    
//     // Add a small delay between firing the next one to clearly distinguish them
//     delay(2000);
//   }

//   // digitalWrite(thrusterPins[0], HIGH);
    
//   // // Keep it on for the specified delay
//   // delay(testDelay);
    
//   // // Turn the MOSFET OFF (solenoid closes)
//   // digitalWrite(thrusterPins[0], LOW);

//   Serial.println("Cycle complete. Restarting in 3 seconds...");
//   Serial.println("---");
//   // delay(3000); // Wait 3 seconds before starting the loop all over again

//   // for (int i = 0; i < NUM_THRUSTERS; i++) {
    
//   //   Serial.print("unFiring Thruster on Pin: ");
//   //   Serial.println(thrusterPins[i]);

//   //   // Turn the MOSFET ON (solenoid opens)
//   //   // digitalWrite(thrusterPins[i], HIGH);
    
//   //   // Keep it on for the specified delay
//   //   delay(testDelay);
    
//   //   // Turn the MOSFET OFF (solenoid closes)
//   //   digitalWrite(thrusterPins[i], LOW);
    
//   //   // Add a small delay between firing the next one to clearly distinguish them

//   // }

//   // delay(5000);
// }