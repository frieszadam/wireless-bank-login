/*
  Author: Adam Friesz, Brandon Chang
  Date: 8/15/2024
  Filename: bank_rx2.ino
  Description: Simulates a server-side bank that allows a remote login, deposit, withdrawal and password changes.
  Wireless transmission is accomplished via ESPNOW. Mutexes are used to protect access to critical sections in
  which shared variables are accessed. A queue is used to store data transmitted to the bank, providing potential to
  scale the design to accomodate multiple remote transmitters. 
*/

// ------------------ INCLUDES -------------------------//
#include <esp_now.h>
#include <WiFi.h>
#include <LiquidCrystal_I2C.h>

// ------------------- MACROS ---------------------------//
#define trigPin D9 // for ultrasonic
#define echoPin D10 // for ultrasonic
#define PIN_LED D2 // for login_status
#define LOGIN_TIME 10000000 // for user login - 10 seconds: 10,000,000

// ------------------- Function Prototypes -------------------//
// Tasks
void taskSetPW(void*);
void taskCheckLoginInput(void*);
void taskLED(void*);
void taskLCDWrite(void*);
void taskCheckDistance(void*);
// Interrupts
void IRAM_ATTR setNewRemoteTranmission(const uint8_t*, const uint8_t*, int);

// ------------------------ GLOBALS --------------------------//
static SemaphoreHandle_t sem_pw, sem_login_status; // semaphore handles
static QueueHandle_t q_remote_data;  // queue handle for remote data received queue

LiquidCrystal_I2C lcd(0x27, 16, 2); // Initialize the LCD address to 0x27
uint8_t broadcastAddress[] = {0x74, 0x4D, 0xBD, 0x7D, 0x20, 0xF4}; // Receiver's MAC address

hw_timer_t * timer = NULL; // declare a timer object and initialize to null
volatile int login_status, pw, balance;  // global variables storing state of the remote user

volatile int lockdown;  // flag register storing whether we are in a lockdown

void setup() {
  // Set up Serial Monitor with baud rate = 9600
  Serial.begin(9600);

  // Initialize global state variables
  pw = 0;
  balance = 0;
  lockdown = 0;
  
  // Configure and start timer with 1 microsecond ticks
  timer = timerBegin(0, 80, true); // Timer 0, prescaler 80, count up from 0
  timerStart(timer); // Start the timer
  
  // Create mutex to protect access to password which is a shared global var
  sem_pw = xSemaphoreCreateMutex();
  if (sem_pw == NULL) {  // check for error creating mutex
    Serial.println("Failed to create sem_pw.");
    while(1); // Halt the system
  }
  xSemaphoreGive(sem_pw);

  // Create a mutex for the login status of a user
  sem_login_status = xSemaphoreCreateMutex();
  if (sem_login_status == NULL) {  // check for error creating mutex
    Serial.println("Failed to create sem_login_status.");
    while(1); // Halt the system
  }
  xSemaphoreGive(sem_login_status);

  // Sets up a queue for data input
  q_remote_data = xQueueCreate(4, 2);  // holds 4 elements of size 2 bytes
  if (q_remote_data == NULL) {  // check for error creating queue
    Serial.println("Failed to create queue.");
    while(1); // Halt the system
  }

  // initialize pins
  pinMode(PIN_LED, OUTPUT); // login status
  pinMode(trigPin, OUTPUT); // ultrasonic
  pinMode(echoPin, INPUT); // ultrasonic
  
  // Set up lcd
  lcd.backlight();
  lcd.init();

  //----------------------- Setup for ESPNOW communication -----------------------//
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) return; // Initialize ESP-NOW and check for success

  esp_now_peer_info_t peerInfo; // Data structure for handling peer information
  memset(&peerInfo, 0, sizeof(peerInfo));
  // Copy the receiver's MAC address to peer information
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);  
  peerInfo.channel = 0; // Set WiFi channel to 0 (default)
  peerInfo.encrypt = false; // Disable encryption
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) 
    return; // Add peer and check for success

  esp_now_register_recv_cb(setNewRemoteTranmission);  // when data is recieved call setNewRemoteTranmission

  // ---------------------------- Create tasks and pin to cores. ---------------------//
  xTaskCreatePinnedToCore(taskCheckLoginInput, "Validate Login Attempt Task", 8192, NULL, 2, NULL, tskNO_AFFINITY); // bigger than 4096
  xTaskCreatePinnedToCore(taskSetPW, "Password Set Task", 2048, NULL, 2, NULL, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(taskLED, "LED task", 1024, NULL, 2, NULL, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(taskLCDWrite, "LCD Writing", 4096, NULL, 2, NULL, tskNO_AFFINITY); // bigger than 2048
  xTaskCreatePinnedToCore(taskCheckDistance, "Distance Checker", 1024, NULL, 2, NULL, tskNO_AFFINITY); // if using serial, increase stack to 2048

  // Ensures Serial is ready to receive input
  while(!Serial){}
  Serial.println("Finished setup.");
}

void loop() {}  // Loop not used in FreeRTOS

/*
  Description: Checks the login status and updates flags according to whether the system is on lockdown, 
  a valid action is done, or if the user has been logged in for too long. Also lifts lockdown after a certain period of time
  Takes data from the queue and determines an action's validity (is the password is wrong, the reply will be zero)
  Reply value is either 1 or 0, depending on the action validity
*/
void taskCheckLoginInput(void* arg) {
  // initialize local heap-allocated static variables
  static unsigned long lastTime = 0;  // last time login occurred
  static uint16_t transmit_data;  // data being transmitted
  static uint8_t reply;  // reply to send out
  static int mem_status = 0;  // previous login status
  static int send_reply = 0;  // to send a reply or not
  while (1) {
    // Check if transmission data was received from the remote user
    if (xQueueReceive(q_remote_data, &transmit_data, (TickType_t) 20) == pdTRUE) {
      // extract data components based on encoding scheme: 2 LSB = ctrl bits, rest = data bits
      int ctrl_bits = transmit_data & 0b11;
      int data = transmit_data >> 2;
      switch (ctrl_bits) {
        // LOGIN ATTEMPT
        case 0b00: // if the user is trying to login
                  int success;
                  if (lockdown) {  // if in lockdown ignore transmit data and reply 0
                    reply = 0;
                  } else {
                    if (xSemaphoreTake(sem_pw, (TickType_t) 20) == pdTRUE) {
                      success = pw==data;  // if given pw matches stored pw we succeeded
                      xSemaphoreGive(sem_pw);
                    } else {  // if can't access pw
                      success = 0;
                    }
                    if (xSemaphoreTake(sem_login_status, (TickType_t) 30) == pdTRUE) {
                      login_status = success;  // update login status
                      xSemaphoreGive(sem_login_status);
                      mem_status = success;  // update saved login status
                    }
                    lastTime = success? timerRead(timer): lastTime;  // update last time of login to allow auto-log out
                    reply = success;  // reply whether it was success or not
                  }
                  break;
        // DEPOSIT ATTEMPT
        case 0b01: if (xSemaphoreTake(sem_login_status, (TickType_t) 20) == pdTRUE) { // if the user is trying to deposit
                    balance = login_status? balance+data: balance;  // if logged in, allow deposit
                    reply = login_status;  // allow if logged in
                    xSemaphoreGive(sem_login_status);
                  } else { // if cant access mutex
                    reply = 0;
                  }
                  break;
        // WITHDRAWAL ATTEMPT
        case 0b10: if (xSemaphoreTake(sem_login_status, (TickType_t) 20) == pdTRUE) { // if the user is trying to withdraw
                    if (data > balance || login_status == 0) { // withdraw amount must be less than the balance
                      reply = 0;
                    } else { // if logged in and withdrawal amount is lt. balance
                      balance = balance-data;  // update balance
                      reply = 1;  // allow withdrawal 
                    }
                    xSemaphoreGive(sem_login_status); // return mutex
                  } else {
                    reply = 0;
                  }
                  break;
        // PASSWORD CHANGE ATTEMPT
        case 0b11: if (xSemaphoreTake(sem_login_status, (TickType_t) 20) == pdTRUE) { // if the user sent a password change request
                    if (login_status == 1) { // if logged in, allow pw change
                      if (xSemaphoreTake(sem_pw, (TickType_t) 20) == pdTRUE) {
                        pw = data;  // change password
                        xSemaphoreGive(sem_pw);
                        reply = 1;
                      } else {  // if can't get pw to change it
                        reply = 0;
                      }
                    } else {  // if not logged in dont allow pw change
                      reply = 0;
                    }
                    xSemaphoreGive(sem_login_status);
                  } else { // if cant get login status to check it
                    reply = 0;
                  }
                  break;
        // UNKNOWN CASE
        default: Serial.println("Unknown input from remote.");
      }
      send_reply = 1;
    } else if (xSemaphoreTake(sem_login_status, (TickType_t) 20) == pdTRUE) { // Checks whether the login status was changed
      if (login_status != mem_status) {  // if we logged out due to lockdown
        send_reply = 1;
        mem_status = login_status;
        reply = login_status;
      }
      xSemaphoreGive(sem_login_status);
    }

    // Logs user out if elapsed time has occured
    if (mem_status == 1 && timerRead(timer)-lastTime >= LOGIN_TIME) { // if login time elapsed
      if (xSemaphoreTake(sem_login_status, (TickType_t) 20) == pdTRUE) {
        login_status = 0;  // logout
        xSemaphoreGive(sem_login_status);
        mem_status = 0;
        reply = 0;  
        send_reply = 1;  // alert user of logout, unsolicited tx case
      }
    }

    // If long enough has passed, stop the lockdown
    if (timerRead(timer)-lastTime > 2*LOGIN_TIME) {
      lockdown = 0;
    }


    // Sends a response to user if necessary
    if (send_reply == 1) {      
      Serial.print("Reply val: ");
      Serial.println(reply);

      esp_err_t send_status = esp_now_send(broadcastAddress, &reply, 1); // send data to MAC address
      if (send_status != ESP_OK) {
        Serial.println("Error sending to remote.");
        break;
      }
      send_reply = 0;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); // 10 ms delay
  }
}

/*
  Description: Sets the remote user's password to the given value from Serial Monitor.
*/
void taskSetPW(void* arg) {
  while (1) {
    if (Serial.available()) {
      int val = Serial.parseInt();  // GET USER INPUT AND CONVERT TO INT
      if (xSemaphoreTake(sem_pw, (TickType_t) 20) == pdTRUE) {
        pw = val;  // update the password
        xSemaphoreGive(sem_pw);
      }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS); // 20 ms delay
  }
}

/*
  Description: Updates LCD to ensure current login status and balance are apparent to the user.
*/
void taskLCDWrite(void* arg) {
  // create locally accesible heap-allocated data holding previous states
  static int mem_status = -1;
  static int mem_balance = -1;
  while (1) {
    int curr_status, curr_balance;
    if (xSemaphoreTake(sem_login_status, (TickType_t) 20) == pdTRUE) {
      curr_status = login_status;  // save login status to reduce amount of time semaphore is held
      xSemaphoreGive(sem_login_status);
    }
    curr_balance = balance;
    
    // update the LCD if there has been a change in the login status or balance
    if (mem_status != curr_status || mem_balance != curr_balance) {
      // display login status
      lcd.clear();
      lcd.setCursor(0,0);
      switch (curr_status) {
        case 0:
            lcd.print("Logged out.");
            break;
        case 1:
          lcd.print("Logged in.");
          break;
        default: ;
      }
      // display balance
      lcd.setCursor(0,1);
      lcd.print("Balance: ");
      lcd.print(curr_balance);
    }
    // update static variables
    mem_status = curr_status;
    mem_balance = curr_balance;
    vTaskDelay(15 / portTICK_PERIOD_MS); // 15 ms delay
  }
}

/* 
  Description: Writes the current login status to an LED.
*/
void taskLED(void* arg) {
  while (1) {
    if (xSemaphoreTake(sem_login_status, (TickType_t) 20) == pdTRUE) {
      digitalWrite(PIN_LED, login_status);  // turn LED on if logged in, off if logged out
      xSemaphoreGive(sem_login_status);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // 10 ms period
  }
}


/*
  Description: Monitors the distance from the closest object to the bank vault. If someone or something
  gets too close it sends the bank into a lockdown state to protect all resources. When lockdown is triggered
  all users are logged out and no one is allowed to log in, withdraw, deposit, or change their password. 
*/
  void taskCheckDistance(void* arg) {
    while(1) {
      // Pulse the trigPin of the ultrasonic sensor to send out a high frequency wave
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
    
      // Calculate distance by measuring how long it took for the waves reflection
      // to reach the sensor.
      float duration = pulseIn(echoPin, HIGH); // inbuilt function
      float distance = (duration*0.343)/2;
      
      // lockdown logic occurs if something is closer than 300 mm
      if (distance <= 300) { // if closer than 300mm
        lockdown = 1;  // lockdown
        login_status = 0;  // logout
      }
      vTaskDelay(50 / portTICK_PERIOD_MS); // 50 ms period
    } 
  }


/*
  Descripiton: Interrupt triggered on data received from remote login. Adds received data
  to a processing queue which is checked and responded to by other tasks. The ISR is kept
  as short as possible to provide minimal disruption to control flow, decreasing the risk
  of unanticipated errors.
*/
void IRAM_ATTR setNewRemoteTranmission(const uint8_t * mac, const uint8_t *incomingData, int len) {
  uint16_t remote_data = *incomingData | (*(incomingData+1) << 8);  // assuming msb bits sent second
  xQueueSend(q_remote_data, &remote_data, (TickType_t) 20);
}
