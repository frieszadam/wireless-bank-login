/*
  Filename: remote_login_tx.ino
  Author: Adam Friesz, Brandon Chang
  Date: August 14th, 2024
  Description: Remote, transmitter side, of wireless communication scheme allowing login,
  deposit, withdrawl, and change of password via rotary encoder binary input scheme.
  Wireless communication accomplished via ESPNow and tasks scheduled using FreeRTOS,
  pre-emptive priority scheduler with scheduler tick rate of approximately 1ms.
*/

// -------------- INCLUDES ----------------//
#include <esp_now.h>
#include <WiFi.h>
#include <LiquidCrystal_I2C.h>

// -------------- MACROS ----------------//
#define PIN_CLK D4
#define PIN_DT D5
#define PIN_SW D6
#define PIN_BUTTON D3
#define PIN_BUZZER D2

extern const unsigned long LOGIN_TIME = 5000000;
extern const unsigned long DEBOUNCE_TIME = 500000;
extern const uint16_t NULL_REQ = 0xFFFF;
extern const uint16_t LOGOUT_DATA = 0xFFFC;

// -------------- FUNCTION PROTOTYPES ------//
// Tasks
void taskPollRotaryLogin(void*);
void taskSendToBank(void*);
void taskLCDWrite(void*);
void taskWithdrawDeposit(void*);

// Interrupts
void lockSafeISR();
void IRAM_ATTR updateBankResponse(const uint8_t*, const uint8_t*, int);

// -------------- QUEUE/SEMAPHORE HANDLES ------//
static QueueHandle_t q_transmit_data;  // queue handle
static SemaphoreHandle_t sem_login_status, sem_response, sem_balance;  // semaphore handles

// ---------- LIBRARY RELATED GLOBALS -----------//
const uint8_t broadcastAddress[] = {0x80, 0x65, 0x99, 0xC5, 0xED, 0x2C}; // Receiver's MAC address
LiquidCrystal_I2C lcd(0x27, 16, 2); // Initialize the LCD address to 0x27
TaskHandle_t taskUpdateLocalState_handle = NULL;  // task handle
hw_timer_t * timer = NULL; // declare a timer object and initialize to null

// ---------- GLOBAL VARIABLES -------------------//
int login_status, balance, button_pressed;
uint8_t req_response;
uint16_t curr_request;

// Runs once, sets up everything before anything else is run
void setup() {
  // Begin serial monitor transmission
  Serial.begin(9600);
  while(!Serial);

  // Configure and start timer with 1 microsecond ticks
  timer = timerBegin(0, 80, true);  // upcounting timer w/ prescaler of 80
  timerStart(timer);

  // Create queue that stores 4 elements of size 2 bytes
  q_transmit_data = xQueueCreate(4, 2);
  if (q_transmit_data == NULL) {  // check for error creating queue
    Serial.println("Failed to create queue.");
    while(1); // Halt the system
  }
  
  // Initialize global variables
  login_status = 0;
  balance = 0;
  button_pressed = 0;
  curr_request = NULL_REQ;  // value used to check for response recieved
  req_response = NULL_REQ;  // ""

  // Create mutex to protect access to the login_status global variable
  sem_login_status = xSemaphoreCreateMutex();
  if (sem_login_status == NULL) {
    Serial.println("Failed to create sem_login_status.");
    while(1); // Halt the system
  }
  xSemaphoreGive(sem_login_status);

  // Create mutex to protect access to the balance global variable
  sem_balance = xSemaphoreCreateMutex();
  if (sem_balance == NULL) {  // check for error creating mutex
    Serial.println("Failed to create sem_balance.");
    while(1); // Halt the system
  }
  xSemaphoreGive(sem_balance);  // make sem available

  // Create mutex to protect access to the curr_request global variable
  sem_response = xSemaphoreCreateMutex();
  if (sem_response == NULL) {  // check for error creating mutex
    Serial.println("Failed to create sem_response.");
    while(1); // Halt the system
  }
  xSemaphoreGive(sem_response);  // make sem avaiable

  // initialize digital GPIO pins
  pinMode(PIN_CLK, INPUT);  // rotary encoder
  pinMode(PIN_DT, INPUT);  // rotary encoder
  pinMode(PIN_SW, INPUT_PULLUP);  // rotary encoder

  // set up safe reset ISR
  pinMode(PIN_BUTTON, INPUT_PULLUP);  // button
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), &lockSafeISR, RISING); // Triggers on rising edge

  // Set up LCD
  lcd.backlight();
  lcd.init();

  // ------------------- Setup for ESPNOW communication -------------------//
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) return; // Initialize ESP-NOW and check for success

  // Create data structure for handling peer information
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);  // Copy the bank's MAC address to peer information
  peerInfo.channel = 0; // Set WiFi channel to 0 (default)
  peerInfo.encrypt = false; // Disable encryption

  // Add peer and check for success
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP peer.");
    while(1); // Halt the system
  }
  
  // upon receiving a message, call updateBankResponse
  esp_now_register_recv_cb(updateBankResponse);

  // ---------------------------- Create tasks and pin to cores. ---------------------//
  xTaskCreatePinnedToCore(taskPollRotaryLogin, "Take Input Task", 2048, NULL, 4, NULL, tskNO_AFFINITY);  // CORE 0
  xTaskCreatePinnedToCore(taskSendToBank, "Transmit Task", 2048, NULL, 2, NULL, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(taskLogout, "Logout Task", 2048, NULL, 2, NULL, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(taskWithdrawDeposit, "Withdraw, Deposit Task", 2048, NULL, 2, NULL, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(taskLCDWrite, "Write LCD Task", 2048, NULL, 2, NULL, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(taskUpdateLocalState, "Update State Task", 2048, NULL, 3, &taskUpdateLocalState_handle, tskNO_AFFINITY);

  Serial.println("Finished setup.");
}

void loop() {}  // Loop not used in FreeRTOS

/*
  Description: Checks for rotary encoder input from the user and adds to the password
  being entered. Rotary input is taken through a binary encoding scheme where a clockwise
  turn corresponds to a 1 and counter clockwise turn corresponds to a 0 with the password
  being assembled starting from the least significant bits. 
    ex. CW, CCW, CW, CW, SW == 0b1101
  The assembled password is transmitted to the bank upon pressing the encoder's switch. 
  If not logged in the password is checked as a login attempt, if already logged in the
  password is changed to the given input.
*/
void taskPollRotaryLogin(void* arg) {
  // initialize local heap-allocated static variables
  static unsigned long lastTime = 0;  // stores last transmit time
  static uint16_t pw_input = 0;  // stores password being assembled
  static int head = 0;  // stores bit index in pw_input with LSB = 0
  static int prev_button_state = 1;  // was the button previously pressed
  static int prev_CLK;  // previous state of CLK
  while (1) {
    int curr_CLK = digitalRead(PIN_CLK);  // read rotary encoder data
    
    // If last and current state of CLK are different, then pulse occurred
	// React to only 1 state change to avoid double count
    if (curr_CLK == 1 && curr_CLK != prev_CLK) {
      pw_input |= digitalRead(PIN_DT)<<head;  // DT && CLK corresponds to CW=1, !DT && CLK corresponds CCW=0
      head++;  // increment bit index in pw_input
    }
    int curr_button_state = digitalRead(PIN_SW);  // read rotary encoder button input
    int complete = curr_button_state==0 && prev_button_state==1;  // button is active low, avoid multiple transmits

    // transmit if button pressed for first time, and haven't transmitted in last 0.5 seconds
    // or we have reached max password size. 
    if ((complete && (timerRead(timer)-lastTime) > 500000) || head==14) {
      lastTime = timerRead(timer);  // update time of last transmission
      head = 0;  // reset pw_input index
      
      // LOGIC FOR PW CHANGE
      int ctrl_bits = 0b00;  // assume pw login attempt
      if (xSemaphoreTake(sem_login_status, (TickType_t) 20) == pdTRUE) {
        if (login_status==1)  // if already logged in then count as pw change attempt
          ctrl_bits = 0b11;
        xSemaphoreGive(sem_login_status);
      }

      uint16_t send_data = (pw_input << 2) | ctrl_bits;  // adhere to transmission scheme
      xQueueSend(q_transmit_data, (void*) &send_data, (TickType_t) 20);
      
      pw_input = 0;  // reset pw_input
    }

    // update memory
    prev_CLK = curr_CLK;
    prev_button_state = curr_button_state;

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

/*
  Description: Reads integers from the Serial Monitor and attempts deposits or withdrawls of the
  given amount. If negative int given, withdraw attempted. If positive int given, deposit attempted.
  Deposits or withdrawls can only occur when logged in so the bank may reject some attempts. Login
  status is checked solely by the bank, not checked by the remote transmitter.
*/
void taskWithdrawDeposit(void* arg) {
  while (1) {
    if (Serial.available()) {
      int val = Serial.parseInt();  // convert user input in Serial Monitor to an integer
      uint8_t abs_val = (uint8_t) (val<0)? -val: val;  // convert given int to abs value
      uint16_t send_data = abs_val << 2 | ((val<0)? 0b10: 0b01);  // determine ctrl bits: 01 for deposit, 10 for withdrawl
      xQueueSend(q_transmit_data, &send_data, (TickType_t) 20);  // send to transmit queue
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  } 
}

/*
  Description: Sends data from the transmit queue, which collects login deposit, withdrawl, and pw change
  attempts, to the bank receiver via ESPNOW. Saves the transmitted request data to be checked when response
  is received from the bank.
*/
void taskSendToBank(void* arg) {
  while (1) {
    uint16_t transmit_data;
    // if there is data to be transmitted
    if(xQueueReceive(q_transmit_data, (void*) &transmit_data, (TickType_t) 20) == pdTRUE) {
      
      // Format data to transmit in 2 8 bit packets with data[0] = LSB, data[1] = MSB
      uint8_t send_data[2];
      send_data[0] = transmit_data & 0xFF;  // least sig. 8 bits of transmit data
      send_data[1] = transmit_data >> 8;  // most sig. 8 bits of transmit data
      curr_request = transmit_data;  // save data being transmitted in a global var for later reference
      esp_err_t send_status = esp_now_send(broadcastAddress, send_data, 2);  // send data to bank_rx
      if (send_status != ESP_OK) // check status of send and alert user
        Serial.println("Failed to send data.");
      else {
        int ctrl_bits = transmit_data & 0b11;
        // Report what type of request was sent to bank_rx
        switch (ctrl_bits) {
          case 0b00: Serial.println(transmit_data == LOGOUT_DATA? "Sent logout request.": "Sent login request."); break;
          case 0b01: Serial.println("Sent deposit request."); break;
          case 0b10: Serial.println("Sent withdrawal request."); break;
          case 0b11: Serial.println("Sent password change request.");
          default: Serial.println("Unknown request."); break;
        }
      }
    }  // xQueueReceive
    vTaskDelay(10 / portTICK_PERIOD_MS);
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
    
    // get current login status, safely
    if (xSemaphoreTake(sem_login_status, (TickType_t) 20) == pdTRUE) {
      curr_status = login_status;
      xSemaphoreGive(sem_login_status);
    }
    
    // get current balance, safely
    if (xSemaphoreTake(sem_balance, (TickType_t) 20) == pdTRUE) {
      curr_balance = balance;
      xSemaphoreGive(sem_balance);
    }
    
    // update the LCD if there has been a change in the login status or balance
    if (mem_status != curr_status || mem_balance != curr_balance) {
      // display login status
      lcd.clear();
      lcd.setCursor(0,0);
      switch (curr_status) {
        case 0: lcd.print("Logged out."); break;
        case 1: lcd.print("Logged in."); break;
        default: ;
      }
      // display balance
      lcd.setCursor(0,1);
      lcd.print("Balance: $");
      lcd.print(curr_balance);
    }
    // update static variables
    mem_status = curr_status;
    mem_balance = curr_balance;
    vTaskDelay(15 / portTICK_PERIOD_MS);
  }
}

/*
  Description: Check state of global flag variable, updated by ISR upon button press, and send
  debounced logout request to bank.
*/
void taskLogout(void* arg) {
  static unsigned long lastTimePressed = 0;  // last time the button was pressed
  while (1) {
    // check for button press without allowing multiple counting of one press.
    if (button_pressed == 1 && ((timerRead(timer) - lastTimePressed) > DEBOUNCE_TIME)) {
      button_pressed = 0;  // update flag reg
      uint16_t send_data = LOGOUT_DATA;
      xQueueSend(q_transmit_data, &send_data, (TickType_t) 20);  // add logout request to tx queue
      lastTimePressed = timerRead(timer);  // update last time pressed
    }
    vTaskDelay(25 / portTICK_PERIOD_MS);
  }
}

/*
  Description: Task is triggered solely by a response received from the bank. Updates local state
  ie. global variables, buzzer sound, based on the bank response recieved and the transmission
  attempted.
*/
void taskUpdateLocalState(void* arg) {
  while (1) {
    uint32_t ulNotificationValue;
    // Wait for this task to be notified by the updateBankResponse interrupt
    ulNotificationValue = ulTaskNotifyTakeIndexed(0, pdFALSE, portMAX_DELAY);
    
    // If we received a notification, this means we have recieved tx from the bank
    if(ulNotificationValue == 1) {
      // store the transmssion the bank is responding to and extract data + ctrl
      int sent_request = curr_request;
      curr_request = NULL_REQ;  // update global variable saving current transmission to bank
      int ctrl_bits = sent_request & 0b11;
      int data = sent_request >> 2;
      
      // store response from bank_rx locally, to prevent changes in middle of task
      int received_response = req_response;

      // if response was unsolicited we are either in lockdown or time out
      if (sent_request == NULL_REQ) {
          if (xSemaphoreTake(sem_login_status, (TickType_t) 20) == pdTRUE){ 
            login_status = received_response;  // update login status
            xSemaphoreGive(sem_login_status);
            tone(PIN_BUZZER, 400, 500);  // play logout sound
          }      
      } else {
        // determine action based on what request the bank responded to
        switch (ctrl_bits) {
          // LOGIN ATTEMPT
          case 0b00: if (xSemaphoreTake(sem_login_status, (TickType_t) 20) == pdTRUE){ 
                      login_status = received_response;  // update login status
                      xSemaphoreGive(sem_login_status);
                      if (!received_response && sent_request != LOGOUT_DATA)
                        tone(PIN_BUZZER, 400, 500);  // play sound if logged out
                    }
                    break;
          // DEPOSIT ATTEMPT
          case 0b01: if (xSemaphoreTake(sem_balance, (TickType_t) 20) == pdTRUE){ 
                      balance += received_response? data: 0;  // update balance
                      xSemaphoreGive(sem_balance);
                      if (received_response)
                        Serial.println("Made a deposit.");  // alert user via serial
                      else
                        tone(PIN_BUZZER, 400, 500);  // play sound if denied
                    }
                    break; 
          // WITHDRAWAL ATTEMPT
          case 0b10: if (xSemaphoreTake(sem_balance, (TickType_t) 20) == pdTRUE){ 
                      balance -= received_response? data: 0;  // update balance
                      xSemaphoreGive(sem_balance);
                      if (received_response)
                        Serial.println("Made a withdrawal.");  // alert user via serial
                      else
                        tone(PIN_BUZZER, 400, 500);  // play sound if denied
                    }
                    break;
          // PASSWORD CHANGE ATTEMPT
          case 0b11: if (received_response)
                      Serial.println("Changed password.");  // alert user via serial
                    break;
          // UNKNOWN CASE
          default: Serial.println("Unknown response.");
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

/*
  Description: Interrupt triggered upon receiving ESPNOW transmission (from the bank). Saves
  response and awakes a task to update local variables based on it.
*/
void IRAM_ATTR updateBankResponse(const uint8_t * mac, const uint8_t *incomingData, int len) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  req_response = *incomingData;

  // Notify taskUpdateLocalState that response was receieved so it should update local state.  
  vTaskNotifyGiveIndexedFromISR(taskUpdateLocalState_handle, 0, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);  // if a higher priority task became available go to that one
}

/*
  Description: ISR to lock safe. Since the operation is atomic we do not need to access
  via mutex. 
*/
void lockSafeISR() {
  button_pressed = 1;  // atomic operation setting flag register
}
