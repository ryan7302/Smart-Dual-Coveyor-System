#include "Arduino.h"

// Add macro definitions
#define CONVEYOR1_SPEED 100.0f  // cm/s
#define CONVEYOR2_SPEED 80.0f  // cm/s

// Add macro definitions for acceptable length and weight ranges
#define LENGTH_MIN 4.8f
#define LENGTH_MAX 5.2f
#define WEIGHT_MIN 95.0f
#define WEIGHT_MAX 105.0f

// Add macro definitions for sensor positions
#define DISTANCE_BARCODE 20.0f      // Distance from obstruction sensor to barcode sensor
#define DISTANCE_GATE1 40.0f        // Distance from obstruction sensor to Gate1
#define DISTANCE_TRANSFER 60.0f     // Distance from obstruction sensor to conveyor transfer point
#define DISTANCE_WEIGHT_SENSOR 20.0f // Distance from conveyor2 starting point to weight sensor
#define DISTANCE_GATE2 40.0f        // Distance from conveyor2 starting point to Gate2
#define DISTANCE_TO_OBSTRUCTION_SENSOR 10.0f // Distance from conveyor2 starting point to obstruction sensor    
#define   DEBUG_PIN                  12

enum class ProductEventType {
    OBSTRUCTION_SENSOR_RISE,
    OBSTRUCTION_SENSOR_FALL,
    BARCODE_SENSOR_ENTER_WINDOW,
    BARCODE_SENSOR_EXIT_WINDOW,
    GATE1_CHECK,
    TRANSFER_TO_CONVEYOR2,
    WEIGHT_SENSOR_ENTER_WINDOW,
    WEIGHT_SENSOR_EXIT_WINDOW,
    GATE2_CHECK,
    FINISH
};

// Store product information
struct Product {
    float length;         // Product length
    float weight;         // Product weight
    int   barcodeID;      // Barcode ID
};

// Each event
struct ProductEvent {
    uint64_t         triggerTimeMicros; // Microseconds relative to the product's "entry onto conveyor at point 0"
    ProductEventType eventType;
};

// Manage all events for a product
// *** MODIFIED for ESP32 Arduino Core 3.0.0+ ***
// hw_timer_t* replaced by esp_timer_handle_t; added startTimeUs to track scheduling base.
struct ProductScheduler {
    Product product;

    ProductEvent events[16];  // Store all events for this product
    int totalEvents;
    int currentEventIndex;

    int debugPin;
    // ESP timer handle (nullptr when idle) – compatible with Core 3.0.0+
    esp_timer_handle_t timerHandle;
    // Base time (esp_timer_get_time()) when the scheduler was started, used to convert
    // absolute event times into relative delays for the new esp_timer API.
    uint64_t startTimeUs;
};
extern void startSimulator(void);

/*!!!!!!!!!!!!!!!!!!ANY CODE ABOVE THIS LINE SHOULD NOT BE MODIFIED !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*************************INTERFACE BETWEEN SIMULATOR AND CODE **************************************************** */

/*
* g_obstructionSensorPin outputs the physical level signal of the obstruction sensor, which you can observe with an oscilloscope. 
* It is recommended to bind its rising or falling edge to an interrupt for precise product length detection.
*/
#define   g_obstructionSensorPin    32

/*
* You can control gate1CtrlPin to output the physical level signal of gate1, which can be observed through an oscilloscope. 
* The simulator itself does not read this signal - it is only for your debugging purposes.
*/

#define   gate1CtrlPin              33
/*
* You can control gate2CtrlPin to output the physical level signal of gate2, which can be observed through an oscilloscope. 
* The simulator itself does not read this signal - it is only for your debugging purposes.
*/
#define   gate2CtrlPin              15


/*
* You can simulate any number of products as needed to test your control programme. This template only provides 5 products as examples.
* You can determine if your programme is working correctly by checking whether it can correctly obtain the three attributes of each product.
*/

/*
const int MAX_PRODUCTS = 5; // Define maximum number of products
Product productArray[MAX_PRODUCTS] = {
    {5.0f, 100.0f, 1001},  // length, weight, barcodeID
    {4.9f, 98.0f, 1002},
    {5.1f, 102.0f, 1003},
    {5.0f, 97.0f, 1004},
    {4.8f, 103.0f, 1005}
};*/


const int MAX_PRODUCTS = 30; // Define maximum number of products 
Product productArray[MAX_PRODUCTS] = { // 19 accept, 11 fail
    {5.0f, 100.0f, 1001}, // length, weight, barcodeID
    {4.9f, 98.0f, 1002},
    {5.1f, 102.0f, 1003},
    {5.0f, 97.0f, 1004},
    {4.8f, 103.0f, 1005},
    {4.2f, 102.0f, 1006},  // length fail ---> weight not calulated during averaging
    {5.7f, 90.0f, 1007},   // length fail ---> weight not calulated during averaging
    {4.1f, 113.0f, 1008},  // length fail ---> weight not calulated during averaging
    {5.0f, 100.0f, 1009},
    {4.95f, 96.0f, 1010},
    {5.05f, 104.0f, 1011},
    {5.2f, 106.0f, 1012},  // weight fail 
    {4.75f, 99.0f, 1013},  // length fail ---> weight not calulated during averaging
    {4.85f, 100.0f, 1014},
    {5.15f, 101.0f, 1015}, // length fail ---> weight not calulated during averaging
    {5.0f, 95.0f, 1016},
    {4.9f, 97.0f, 1017},
    {4.8f, 94.0f, 1018},   // weight fail
    {5.1f, 105.0f, 1019},
    {5.0f, 99.0f, 1020},
    {4.7f, 100.0f, 1021},  // length fail ---> weight not calulated during averaging
    {5.3f, 102.0f, 1022},  // length fail ---> weight not calulated during averaging
    {5.0f, 100.0f, 1023},
    {4.9f, 98.0f, 1024},
    {5.1f, 102.0f, 1025},
    {5.0f, 97.0f, 1026},
    {4.8f, 103.0f, 1027},
    {4.2f, 102.0f, 1028},  // length fail
    {5.0f, 90.0f, 1029},
    {4.1f, 113.0f, 1030}  // length fail   
};

/*
const int MAX_PRODUCTS = 4;
Product productArray[MAX_PRODUCTS] = {
    {5.0f, 100.0f, 1001},  // length, weight, barcodeID
    {4.0f, 110.0f, 1002}, 
    {60.0f,  80.0f, 1003},
    {4.9f, 103.0f, 1004} 
};
*/

/*
* PLEASE DO NOT MODIFY THE FOLLOWING VARIABLE NAMES. 
* The simulator will use these 5 variables to provide sensor signals needed by your control programme and to obtain and evaluate your Gate signals.
*/
// ========== Simulator writes, students read ==========
volatile bool   g_obstructionSensor = false;  // Obstruction sensor
volatile int    g_barcodeReader     = 0;      // Barcode sensor reading (0=invalid)
volatile float    g_weightSensor      = 0;      // Weight sensor reading (0=invalid)
// ========== Students write, simulator reads ==========
volatile bool   g_gate1Ctrl = false;  // Gate1 control switch
volatile bool   g_gate2Ctrl = false;  // Gate2 control switch

/*********************************Implement your RTOS code below this line*************************************************** */
//The version of this board library（ESP32 by Espressif） is V3.1.1 used in this template.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_timer.h" // For esp_timer_get_time()

// Gate response time (ms)
#define GATE_RESPONSE_TIME_MS 2
#define GATE_RESPONSE_TIME_US (GATE_RESPONSE_TIME_MS * 1000)

// Task priorities
#define TASK_PRIORITY_HIGH    (configMAX_PRIORITIES - 2) //As the higest was assigned to the simulator's task
#define TASK_PRIORITY_MEDIUM  (configMAX_PRIORITIES - 3)
#define TASK_PRIORITY_LOW     (configMAX_PRIORITIES - 4)

// Task stack sizes
#define TASK_STACK_SIZE 4096

// Status update interval (ms)
#define STATUS_UPDATE_INTERVAL_MS 1000

// Product status for tracking
typedef enum {
    PRODUCT_STATUS_NONE,
    PRODUCT_STATUS_ON_CONV1,
    PRODUCT_STATUS_LENGTH_MEASURED,
    PRODUCT_STATUS_BARCODED,
    PRODUCT_STATUS_AWAITING_GATE1_DECISION,
    PRODUCT_STATUS_REJECTED_AT_GATE1,
    PRODUCT_STATUS_TRANSFERRED_TO_CONV2,
    PRODUCT_STATUS_ON_CONV2,
    PRODUCT_STATUS_WEIGHT_MEASURED,
    PRODUCT_STATUS_AWAITING_GATE2_DECISION,
    PRODUCT_STATUS_REJECTED_AT_GATE2,
    PRODUCT_STATUS_ACCEPTED_FINAL,
    PRODUCT_STATUS_PROCESSED  // Generic term for removed from active tracking on a conveyor
} ProductInternalStatus;

// Product data structure for tracking within student's code
struct TrackedProduct {
    int barcodeID;      // Unique product identifier, initially 0     
    float length;       // Measured length in cm    
    float weight;       // Measured weight in g     
    uint64_t entryTimeConv1;  // Time (micros) obstruction sensor rose for this product
    uint64_t exitTimeConv1Obstruction; // Time (micros) obstruction sensor fell
    uint64_t entryTimeConv2; // Time (micros) product effectively started on conveyor 2
    
    ProductInternalStatus status;  // Current internal status of the product
    bool isLengthOK;               // Check for Length within specified max-min
    bool isWeightOK;               // Check for Weight within specified max-min
    
    // Fields for timed gate actuation
    bool gate1DecisionMade;
    bool gate1TargetState; // true to open, false to close
    uint64_t gate1ActuationTimeUs; // Time to actuate gate1
    bool gate1Actuated;
    
    bool gate2DecisionMade;
    bool gate2TargetState;
    uint64_t gate2ActuationTimeUs;
    bool gate2Actuated;
    
    int tempID; // Temporary ID for products before barcode is known
    bool forRealTimeReport; // Flag to include in the next real-time report
};


#define MAX_PRODUCTS_ON_CONV1 10 // Max products actively tracked on conveyor 1 as the distance of gate from obstruction sensor is 40 cm only 10 items are possible
#define MAX_PRODUCTS_ON_CONV2 10 // Max products actively tracked on conveyor 2 as the distance of the 2nd gate from the first conveyor is also 40 cm
#define MAX_PRODUCTS_FOR_REPORT 20 // Max products to hold for the 1-second realtime report window

// Minimum re-read delay of sensors in microseconds if product length is zero (fallback) - This is defined by max length (5.2 cm) and fastest conveyor speed (100cm/s) ==> 52 ms
#define MIN_SENSOR_REREAD_DELAY_US 52000 // 52ms 

TrackedProduct productsOnConv1[MAX_PRODUCTS_ON_CONV1];
volatile int productCountConv1 = 0;
int nextTempID = 1; 

TrackedProduct productsForReport[MAX_PRODUCTS_FOR_REPORT];
volatile int productCountForReport = 0;

// System state tracking variables
volatile uint32_t acceptCount = 0;
volatile uint32_t rejectCount = 0;

// Statistical tracking variables
volatile double totalLengthSum = 0.0; 
volatile double acceptableLengthSum = 0.0;
volatile double defectiveLengthSum = 0.0;
volatile double totalWeightSum = 0.0;
volatile double acceptableWeightSum = 0.0;
volatile double defectiveWeightSum = 0.0;
volatile int totalLengthCount = 0;
volatile int acceptableLengthCount = 0;
volatile int defectiveLengthCount = 0;
volatile int totalWeightCount = 0;
volatile int acceptableWeightCount = 0;
volatile int defectiveWeightCount = 0;

// Timing variables for obstruction sensor (captured by ISR)
volatile uint64_t obstructionSensorRiseTimeUs = 0;
volatile uint64_t obstructionSensorFallTimeUs = 0;
volatile bool newLengthMeasurementAvailable = false;

// FreeRTOS resources
SemaphoreHandle_t conv1Mutex = NULL;          // Mutex for productsOnConv1
SemaphoreHandle_t conv2Mutex = NULL;          // Mutex for productsOnConv2 (defined in conveyor2ControlTask)
SemaphoreHandle_t reportMutex = NULL;         // Mutex for productsForReport
SemaphoreHandle_t statsMutex = NULL;          // Mutex for accessing statistical data
SemaphoreHandle_t serialMutex = NULL;         // Mutex for serial port access
QueueHandle_t conv1ToConv2Queue = NULL;       // Queue for passing products from conveyor1 to conveyor2

// System emergency state
volatile bool emergencyStop = false;

// Task handles
TaskHandle_t conveyor1TaskHandle = NULL;
TaskHandle_t conveyor2TaskHandle = NULL;
TaskHandle_t statusUpdateTaskHandle = NULL;
TaskHandle_t userInterfaceTaskHandle = NULL;

// --- Forward Function Declarations ---
void setupPins();
void setupRTOSTasks();
void clearAllStats();
void triggerEmergencyStop();
void IRAM_ATTR obstructionSensorISR();
void conveyor1ControlTask(void* parameter);
void conveyor2ControlTask(void* parameter);
void statusReportingTask(void* parameter);
void userCommandTask(void* parameter);
void addProductToReport(TrackedProduct product);
static inline uint64_t calculate_product_clear_time_us(float product_length, float conveyor_speed);
static inline uint64_t mydistanceToMicros(float distance, float speed); 


// --- ISR for Obstruction Sensor--
void IRAM_ATTR obstructionSensorISR() {
    uint64_t currentTime = esp_timer_get_time();   // Get current time in microseconds
    if (g_obstructionSensor) {                     // Rising edge has occurred (g_obstructionSensor is now true)
        obstructionSensorRiseTimeUs = currentTime;
    } else {                                       // Falling edge has occurred (g_obstructionSensor is now false)
        obstructionSensorFallTimeUs = currentTime;
        if (obstructionSensorRiseTimeUs > 0 && obstructionSensorFallTimeUs > obstructionSensorRiseTimeUs) {
            newLengthMeasurementAvailable = true;  // Signal main task
        }
    }
}

// --- Utility Functions ---

//Calculate Product clear time based on Product length for barcode and weight sensor activation and delay
static inline uint64_t calculate_product_clear_time_us(float product_length, float conveyor_speed) {
    if (product_length <= 0.0f || conveyor_speed <= 0.0f) {  // In case of unregistered values or missing deadline 
        return MIN_SENSOR_REREAD_DELAY_US; // Used as a fallback approch to debug
    }
    // product length / speed = time(s) => convert to microseconds
    return (uint64_t)((product_length / conveyor_speed) * 1000000.0f);
}


// Convert distance and conveyor speed (m/s or cm/s) to microseconds
static inline uint64_t mydistanceToMicros(float distance, float speed) {
    if (speed <= 0.0f) return UINT64_MAX; // Check for non-positive speed
    // distance / speed = time(s) => convert to microseconds
    double tSec = (double)distance / (double)speed;
    return (uint64_t)(tSec * 1e6);
}


// --- Core 0: Conveyor 1, Status ---
void conveyor1ControlTask(void* parameter) {
    pinMode(g_obstructionSensorPin, INPUT); 
    attachInterrupt(digitalPinToInterrupt(g_obstructionSensorPin), obstructionSensorISR, CHANGE);

    uint64_t lastBarcodeReadTime = 0;
    uint64_t currentBarcodeRereadDelayUs = MIN_SENSOR_REREAD_DELAY_US; // Declaration and Initialization

    //*Debug*//Serial.println("Conveyor 1 Task Started on Core 0");

    // Check for Emergency
    while (true) {
        if (emergencyStop) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

         // Process New Length Measurement
        if (newLengthMeasurementAvailable) {
            newLengthMeasurementAvailable = false; // Reset flag
            
            uint64_t riseTime = obstructionSensorRiseTimeUs;
            uint64_t fallTime = obstructionSensorFallTimeUs;
            obstructionSensorRiseTimeUs = 0;  // Clear for next measurement

            if (fallTime > riseTime) {
                uint64_t transitTimeUs = fallTime - riseTime;
                float measuredLength = (CONVEYOR1_SPEED * transitTimeUs) / 1000000.0f; // product length calculated in cm

                if (xSemaphoreTake(conv1Mutex, portMAX_DELAY) == pdTRUE) {
                    if (productCountConv1 < MAX_PRODUCTS_ON_CONV1) {
                        TrackedProduct* p = &productsOnConv1[productCountConv1];
                        p->tempID = nextTempID++;
                        p->barcodeID = 0; // Barcode not known yet
                        p->length = measuredLength;
                        p->weight = 0.0f; // Weight not known yet
                        p->entryTimeConv1 = riseTime;
                        p->exitTimeConv1Obstruction = fallTime;
                        p->status = PRODUCT_STATUS_LENGTH_MEASURED;
                        p->isLengthOK = (measuredLength >= LENGTH_MIN && measuredLength <= LENGTH_MAX);
                        p->gate1DecisionMade = false;
                        p->gate1Actuated = false;
                        p->forRealTimeReport = false; // Will be set true when barcode is known

                        productCountConv1++;
                         //*Debug*//Serial.printf("[C1] New Product (TempID %d): Length %.2f cm. Length OK: %s\n", p->tempID, p->length, p->isLengthOK ? "Yes" : "No");

                        // Update length statistics
                        if (xSemaphoreTake(statsMutex, portMAX_DELAY) == pdTRUE) {
                            totalLengthSum += p->length;
                            totalLengthCount++;
                            if (p->isLengthOK) {
                                acceptableLengthSum += p->length;
                                acceptableLengthCount++;
                            } else {
                                defectiveLengthSum += p->length;
                                defectiveLengthCount++;
                            }
                            xSemaphoreGive(statsMutex);
                        }
                    } else {
                        //*Debug*//Serial.println("[C1-ERR] Max product limit reached on Conveyor 1. Cannot add new product.");
                    }
                    xSemaphoreGive(conv1Mutex);
                }
            }
        }

        // Process Barcode Scanner is read when `g_barcodeReader` is made non-zero by the simulator when a product is scannable.
        if (g_barcodeReader != 0) {
            int currentSimBarcode = g_barcodeReader; // Capture it as it can change
            uint64_t now = esp_timer_get_time();

            // A debouncing for ensuring a new barcode only once per window 
            // The sesnor waits for the current product to pass through based on the length of the product and conveyor speed before taking in the next detected value
            // In case of Maxmumium possible wait time in case of max possible length of the product (40 cm) on the conveyor / conyeor speed (100 cm) ==>  40s divided by 2 ==> 20s
            // Thus 20s of wait time for a product full length of 40 cm to pass through the sensor and read the next value when available. 
            if (now - lastBarcodeReadTime > currentBarcodeRereadDelayUs) { 
                if (xSemaphoreTake(conv1Mutex, portMAX_DELAY) == pdTRUE) {
                    TrackedProduct* productToBarcode = NULL;
                    // Find the first product that has length measured but not yet barcoded (FIFO)
                    for (int i = 0; i < productCountConv1; ++i) {
                        if (productsOnConv1[i].status == PRODUCT_STATUS_LENGTH_MEASURED) {
                            productToBarcode = &productsOnConv1[i];
                            break;
                        }
                    }
                    if (productToBarcode) {
                        lastBarcodeReadTime = now; 
                        productToBarcode->barcodeID = currentSimBarcode;
                        productToBarcode->status = PRODUCT_STATUS_BARCODED;
                        productToBarcode->forRealTimeReport = true; // Now ready for report
                        addProductToReport(*productToBarcode); // Add a *copy* for reporting
                        
                        currentBarcodeRereadDelayUs = calculate_product_clear_time_us(productToBarcode->length, CONVEYOR1_SPEED); // The wait time is updated basedo on current length of product
                        
                        
                        //*Debug*//Serial.printf("[C1] Product (TempID %d) got Barcode: %d\n", productToBarcode->tempID, productToBarcode->barcodeID);
                    } else {
                        // This is an edge case and can happen when barcode scanner triggers before length measurement of the product (length sensor missed).
                        //*Debug*//Serial.printf("[C1-WARN] Barcode %d read, but no product ready for barcoding.\n", currentSimBarcode);
                    }
                    xSemaphoreGive(conv1Mutex);
                }
            }
        } else {
            lastBarcodeReadTime = 0; // Reset if barcode reader is clear
            currentBarcodeRereadDelayUs = MIN_SENSOR_REREAD_DELAY_US; // Uses the 52ms delay as a fallback
        }

        // Gate 1 Logic & Product Transfer
        if (xSemaphoreTake(conv1Mutex, portMAX_DELAY) == pdTRUE) {
            for (int i = 0; i < productCountConv1; ++i) {
                TrackedProduct* p = &productsOnConv1[i];
                
                 // Make Gate 1 Decision (if barcoded and decision not yet made)
                if (p->status == PRODUCT_STATUS_BARCODED && !p->gate1DecisionMade) {
                    p->gate1TargetState = p->isLengthOK; // Open if length is OK
                    
                    // Calculate when the product's front edge reaches Gate 1
                    uint64_t arrivalAtGate1Us = p->entryTimeConv1 + mydistanceToMicros(DISTANCE_GATE1, CONVEYOR1_SPEED);
                    p->gate1ActuationTimeUs = arrivalAtGate1Us - GATE_RESPONSE_TIME_US;
                    p->gate1DecisionMade = true;
                    //*Debug*// Serial.printf("[C1] Gate 1 decision for BC %d: %s. Actuation at %llu us.\n", p->barcodeID, p->gate1TargetState ? "OPEN" : "CLOSE", p->gate1ActuationTimeUs);
                }
                
                // Actuate Gate 1 (if decision made and time is right)
                if (p->gate1DecisionMade && !p->gate1Actuated && esp_timer_get_time() >= p->gate1ActuationTimeUs) {
                    g_gate1Ctrl = p->gate1TargetState;
                    p->gate1Actuated = true;
                    digitalWrite(gate1CtrlPin, g_gate1Ctrl); 
                    //*Debug*// Serial.printf("[C1] Gate 1 for BC %d %s (Actual time: %llu us)\n", p->barcodeID, g_gate1Ctrl ? "OPENED" : "CLOSED", esp_timer_get_time());
                }
                
                // Transfer Product (if it passed Gate 1 and is at transfer point)
                // The simulator's TRANSFER_TO_CONVEYOR2 event tells us when transfer happens.
                // We need to check if the product *should* be transferred based on its state and gate
                if (p->gate1Actuated) {  // Gate action has been taken
                    // Calculate when product's front edge reaches the transfer point
                    uint64_t arrivalAtTransferUs = p->entryTimeConv1 + mydistanceToMicros(DISTANCE_TRANSFER, CONVEYOR1_SPEED);
                    
                    if (esp_timer_get_time() >= arrivalAtTransferUs) {
                        if (p->isLengthOK) { // Product passed Gate 1
                            p->status = PRODUCT_STATUS_TRANSFERRED_TO_CONV2;
                            p->entryTimeConv2 = esp_timer_get_time(); // Mark entry time to Conv2
                           
                            TrackedProduct productToSend = *p; // Send a copy
                            if (xQueueSend(conv1ToConv2Queue, &productToSend, pdMS_TO_TICKS(10)) == pdPASS) {
                                //*Debug*// Serial.printf("[C1] Product BC %d (L:%.2f) transferred to Conv2.\n", p->barcodeID, p->length);
                            } else {
                                // Edge case of product getting stuck or an emergency
                                //*Debug*// Serial.printf("[C1-ERR] Failed to queue product BC %d for Conv2!\n", p->barcodeID);
                            }
                        } else { // Product rejected at Gate 1
                            p->status = PRODUCT_STATUS_REJECTED_AT_GATE1;
                            if (xSemaphoreTake(statsMutex, portMAX_DELAY) == pdTRUE) {
                                rejectCount++;
                                xSemaphoreGive(statsMutex);
                            }
                            //*Debug*//Serial.printf("[C1] Product BC %d (L:%.2f) rejected at Gate 1.\n", p->barcodeID, p->length);
                            
                            // Product is now "processed" on conv1
                            addProductToReport(*p); // Update rejected product to report
                        }

                        // Remove product from Conveyor 1 active tracking
                        for (int j = i; j < productCountConv1 - 1; ++j) {
                            productsOnConv1[j] = productsOnConv1[j + 1];
                        }
                        productCountConv1--;
                        i--;  // Adjust loop index because the array was shifted
                    }
                }
            }
            xSemaphoreGive(conv1Mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Task loop delay for avoiding stravation
    }
}

// --- Core 1: Conveyor 2 ---
void conveyor2ControlTask(void* parameter) {
    TrackedProduct productsOnConv2[MAX_PRODUCTS_ON_CONV2];
    int productCountConv2 = 0;
    // conv2Mutex is local to this task and is created.
    conv2Mutex = xSemaphoreCreateMutex(); // Initialize the mutex for productsOnConv2
    if (conv2Mutex == NULL) {
        //*Debug*//Serial.println("[C2-FATAL] Failed to create conv2Mutex!");
        vTaskDelete(NULL); 
    }
    uint64_t lastWeightReadTime = 0;
    //*Preious Approach: Resulted in Overflow*//uint64_t currentWeightRereadDelayUs = MIN_SENSOR_REREAD_DELAY_US; // Declaration and Initialization of same approach as the barcode reader

    //*Debug*//Serial.println("Conveyor 2 Task Started on Core 1");

    while (true) {
        if (emergencyStop) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        // Receive products from Conveyor 1
        TrackedProduct newProductFromQueue;
        if (xQueueReceive(conv1ToConv2Queue, &newProductFromQueue, 0) == pdPASS) {
            if (xSemaphoreTake(conv2Mutex, portMAX_DELAY) == pdTRUE) {
                if (productCountConv2 < MAX_PRODUCTS_ON_CONV2) {
                    productsOnConv2[productCountConv2] = newProductFromQueue;
                    productsOnConv2[productCountConv2].status = PRODUCT_STATUS_ON_CONV2;
                    // Reset gate 2 specific flags
                    productsOnConv2[productCountConv2].gate2DecisionMade = false;
                    productsOnConv2[productCountConv2].gate2Actuated = false;
                    productCountConv2++;
                    //*Debug*// Serial.printf("[C2] Received Product BC %d on Conveyor 2.\n", newProductFromQueue.barcodeID);
                } else {
                     //*Debug*// Serial.println("[C2-ERR] Max product limit reached on Conveyor 2.");
                }
                xSemaphoreGive(conv2Mutex);
            }
        }
        
        // Process Weight Sensor when `g_weightSensor` is made non-zero value smae approach as the barcode reader by the simulator.
        if (g_weightSensor > 0.0f) { 
            float currentSimWeight = g_weightSensor;
            uint64_t now = esp_timer_get_time();


            // Similiar modifed wait time approach as the barcode reader (The same preious Approach: Resulted in Overflow due to funbctional share between two core, thus was modified)
            // However, this time the maximum possible length can only be 5.2cm 
            // 5.2cm divided by 80cm/s ==> 65ms window before next product arrives on weigth sensor
            // In case of the shortest value of length 4.8cm, it will be 60ms
            // In case of 5.2cm product length followed by 4.8cm there is a window gap of 65ms -60ms = 5 ms
            // However, this 5ms gap is too short for any product to miss as the required length to miss product weight will be 0.4 cm 
            // In such case the product is already rejected by Gate 1
            if (now - lastWeightReadTime > (LENGTH_MAX/CONVEYOR2_SPEED *1000000)) { // 65ms (65,000us) delay, dynamic based on conveyor speed and acceptable length
                if (xSemaphoreTake(conv2Mutex, portMAX_DELAY) == pdTRUE) {
                    TrackedProduct* productToWeigh = NULL;

                    // Find the first product on Conv2 that hasn't been weighed (FIFO) same approach as previous one
                    for (int i = 0; i < productCountConv2; ++i) {
                        if (productsOnConv2[i].status == PRODUCT_STATUS_ON_CONV2) {
                            productToWeigh = &productsOnConv2[i];
                            break;
                        }
                    }
                    if (productToWeigh) {
                        lastWeightReadTime = now; 
                        productToWeigh->weight = currentSimWeight;
                        productToWeigh->isWeightOK = (productToWeigh->weight >= WEIGHT_MIN && productToWeigh->weight <= WEIGHT_MAX);
                        
                        // Update global report data for this product
                        productToWeigh->status = PRODUCT_STATUS_WEIGHT_MEASURED;
                        addProductToReport(*productToWeigh);
                        
                        //currentWeightRereadDelayUs = calculate_product_clear_time_us(productToWeigh->length, CONVEYOR2_SPEED); //update the weight sensor delay as the current product length
                        
                        //*Debug*// Serial.printf("[C2] Product BC %d Weighed: %.2f g. Weight OK: %s\n", productToWeigh->barcodeID, productToWeigh->weight, productToWeigh->isWeightOK ? "Yes" : "No");

                        // Update weight statistics
                        if (xSemaphoreTake(statsMutex, portMAX_DELAY) == pdTRUE) {
                            totalWeightSum += productToWeigh->weight;
                            totalWeightCount++;
                            if (productToWeigh->isWeightOK) {
                                acceptableWeightSum += productToWeigh->weight;
                                acceptableWeightCount++;
                            } else {
                                defectiveWeightSum += productToWeigh->weight;
                                defectiveWeightCount++;
                            }
                            xSemaphoreGive(statsMutex);
                        }
                    } else {
                       //*Debug*// Serial.printf("[C2-WARN] Weight %.2f g read, but no product ready for weighing.\n", currentSimWeight);
                    }
                    xSemaphoreGive(conv2Mutex);
                }
            }
        } else {
            lastWeightReadTime = 0;  // Reset if weight sensor is clear
            //*Preious Approach: Resulted in Overflow*//currentWeightRereadDelayUs = MIN_SENSOR_REREAD_DELAY_US; // Uses the 52ms delay window in case of fallback
        }
        
        // Gate 2 Logic & Final Product-Processing
        if (xSemaphoreTake(conv2Mutex, portMAX_DELAY) == pdTRUE) {
            for (int i = 0; i < productCountConv2; ++i) {
                TrackedProduct* p = &productsOnConv2[i];

                // Make Gate 2 Decision (if weight measured and decision not yet made)
                if (p->status == PRODUCT_STATUS_WEIGHT_MEASURED && !p->gate2DecisionMade) {
                    // Final acceptance depends on BOTH length AND weight being OK
                    p->gate2TargetState = (p->isLengthOK && p->isWeightOK);
                    uint64_t arrivalAtGate2Us = p->entryTimeConv2 + mydistanceToMicros(DISTANCE_GATE2, CONVEYOR2_SPEED);
                    p->gate2ActuationTimeUs = arrivalAtGate2Us - GATE_RESPONSE_TIME_US;
                    p->gate2DecisionMade = true;
                   //*Debug*// Serial.printf("[C2] Gate 2 decision for BC %d: %s. Actuation at %llu us.\n", p->barcodeID, p->gate2TargetState ? "OPEN (ACCEPT)" : "CLOSE (REJECT)", p->gate2ActuationTimeUs);
                }
                
                // Actuate Gate 2
                if (p->gate2DecisionMade && !p->gate2Actuated && esp_timer_get_time() >= p->gate2ActuationTimeUs) {
                    g_gate2Ctrl = p->gate2TargetState;
                    p->gate2Actuated = true;
                    digitalWrite(gate2CtrlPin, g_gate2Ctrl); 
                    //*Debug*// Serial.printf("[C2] Gate 2 for BC %d %s (Actual time: %llu us)\n", p->barcodeID, g_gate2Ctrl ? "OPENED" : "CLOSED", esp_timer_get_time());
                }

                // Finalize Product (if gate actuated and product is past gate)
                if (p->gate2Actuated) {
                    // Consider product's tail clearing the gate area
                    uint64_t tailClearGate2TimeUs = p->entryTimeConv2 + mydistanceToMicros(DISTANCE_GATE2 + p->length, CONVEYOR2_SPEED); 
                    if (esp_timer_get_time() >= tailClearGate2TimeUs) {
                        if (xSemaphoreTake(statsMutex, portMAX_DELAY) == pdTRUE) {
                            if (p->isLengthOK && p->isWeightOK) {
                                p->status = PRODUCT_STATUS_ACCEPTED_FINAL;
                                acceptCount++;
                                 //*Debug*// Serial.printf("[C2] Product BC %d ACCEPTED.\n", p->barcodeID);
                            } else {
                                p->status = PRODUCT_STATUS_REJECTED_AT_GATE2;
                                rejectCount++;
                                 //*Debug*// Serial.printf("[C2] Product BC %d REJECTED (L:%s, W:%s).\n", p->barcodeID, p->isLengthOK ? "OK" : "Fail", p->isWeightOK ? "OK" : "Fail");
                            }
                            xSemaphoreGive(statsMutex);
                        }
                        // Update product in report list one last time with final status
                        addProductToReport(*p);
                        
                        // Remove from Conveyor 2 active tracking
                        for (int j = i; j < productCountConv2 - 1; ++j) {
                            productsOnConv2[j] = productsOnConv2[j + 1];
                        }
                        productCountConv2--;
                        i--; // Adjust loop index because the array was shifted
                    }
                }
            }
            xSemaphoreGive(conv2Mutex);
        }

        // Emergency Case: Collision Detection on Conveyor 2 
        if (xSemaphoreTake(conv2Mutex, portMAX_DELAY) == pdTRUE) {
            if (productCountConv2 >= 2) {
                for (int i = 0; i < productCountConv2 - 1; ++i) { // i is the leader product 
                    TrackedProduct* leader = &productsOnConv2[i];
                    TrackedProduct* follower = &productsOnConv2[i + 1]; // i+1 is the follower product
                    uint64_t currentTimeUs = esp_timer_get_time();
                    
                    // Calculation for current position of the FRONT of each product from the start of Conveyor 2
                    double leader_front_pos_cm = ((double)currentTimeUs - leader->entryTimeConv2) * CONVEYOR2_SPEED / 1000000.0;
                    double follower_front_pos_cm = ((double)currentTimeUs - follower->entryTimeConv2) * CONVEYOR2_SPEED / 1000000.0;
                    
                    // Calculate current position of the TAIL of the LEADER product
                    double leader_tail_pos_cm = leader_front_pos_cm - leader->length;
                    
                    
                    // Check for collision if follower has started moving onto the conveyor 
                    // and the leader is also on the conveyor.
                    // A small tolerance (e.g., follower->length / 4.0) ensures follower is substantially on.
                    if (follower_front_pos_cm > -(follower->length / 4.0) && leader_front_pos_cm > 0) { 
                        if (follower_front_pos_cm >= leader_tail_pos_cm) {
                            if (!emergencyStop) { 
                                /*
                                Serial.printf("[C2-EMERGENCY] Collision Detected! Follower BC %d (front: %.2fcm) caught Leader BC %d (tail: %.2fcm)\n",
                                              follower->barcodeID != 0 ? follower->barcodeID : follower->tempID, 
                                              follower_front_pos_cm, 
                                              leader->barcodeID != 0 ? leader->barcodeID : leader->tempID, 
                                              leader_tail_pos_cm);
                                */
                                triggerEmergencyStop();
                            }
                            break; // Stop checking further collisions in this cycle as emergency is triggered
                        }
                    }
                }
            }
            xSemaphoreGive(conv2Mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Task loop delay
    }
}

void statusReportingTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    char statusBuffer[512]; 
    
    //*Debug*// Serial.println("Status Reporting Task Started on Core 0");
    while(true) {
        if (emergencyStop) {
            vTaskDelay(pdMS_TO_TICKS(100));  // Less frequrent check the emergency fkag, as the reoport should not occupy much time on the core.
            continue;
        }

        // Format: "convRT: AcceptCount,RejectCount,SN,Length,Weight,SN,Length,Weight"
        if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
            if (xSemaphoreTake(statsMutex, portMAX_DELAY) == pdTRUE) { // For accept/reject counts
                snprintf(statusBuffer, sizeof(statusBuffer), "convRT:%03d,%03d", (int)acceptCount, (int)rejectCount);
                xSemaphoreGive(statsMutex);
            } else { // Fallback if statsMutex not available
                 snprintf(statusBuffer, sizeof(statusBuffer), "convRT:---,---");
            }
            if (xSemaphoreTake(reportMutex, portMAX_DELAY) == pdTRUE) {
                for (int i = 0; i < productCountForReport; ++i) {
                    TrackedProduct* p = &productsForReport[i];

                     // Only report products that have an actual barcodeID and have been processed for length
                    if (p->barcodeID != 0 && p->length > 0) { 
                        char productDetail[64];
                        // If rejected at Gate 1, or not yet weighed, weight is 0.0 for the report
                        float reportWeight = p->weight;
                        if (p->status == PRODUCT_STATUS_REJECTED_AT_GATE1 || ! (p->status >= PRODUCT_STATUS_WEIGHT_MEASURED) ) {
                            reportWeight = 0.0f;
                        }
                        // Ensure length is also valid before reporting
                        float reportLength = p->length > 0 ? p->length : 0.0f;

                        snprintf(productDetail, sizeof(productDetail), ",%d,%.1f,%.1f",
                                 p->barcodeID, reportLength, reportWeight);
                        
                        // Check buffer capacity before adding data
                        if (strlen(statusBuffer) + strlen(productDetail) < sizeof(statusBuffer)) {
                            strncat(statusBuffer, productDetail, sizeof(statusBuffer) - strlen(statusBuffer) - 1);
                        } else {
                            // Serial.println("[REPORT-ERR] Buffer full, stopping report generation for this cycle.");
                            break; 
                        }
                    }
                }
                productCountForReport = 0; // Clear report list after printing
                xSemaphoreGive(reportMutex);
            }
            Serial.println(statusBuffer);
            xSemaphoreGive(serialMutex);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(STATUS_UPDATE_INTERVAL_MS));
    }
}

void userCommandTask(void* parameter) {
    String command; // Using Arduino String as it is easier to setup
    char responseBuffer[256];
    
    // Serial.println("User Command Task Started on Core 0");
    
    while(true) {
        if (Serial.available() > 0) {
            command = Serial.readStringUntil('\n');
            command.trim();
            
            if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
                //*Debug*// Serial.print("Received command: '"); Serial.print(command); Serial.println("'");
                if (command.equals("stat")) {
                    if (xSemaphoreTake(statsMutex, portMAX_DELAY) == pdTRUE) {
                        double avgTotalLength = totalLengthCount > 0 ? totalLengthSum / totalLengthCount : 0.0;
                        double avgAcceptableLength = acceptableLengthCount > 0 ? acceptableLengthSum / acceptableLengthCount : 0.0;
                        double avgDefectiveLength = defectiveLengthCount > 0 ? defectiveLengthSum / defectiveLengthCount : 0.0;
                        double avgTotalWeight = totalWeightCount > 0 ? totalWeightSum / totalWeightCount : 0.0;
                        double avgAcceptableWeight = acceptableWeightCount > 0 ? acceptableWeightSum / acceptableWeightCount : 0.0;
                        double avgDefectiveWeight = defectiveWeightCount > 0 ? defectiveWeightSum / defectiveWeightCount : 0.0;
                        
                        // Format: "convSTAT:AvgTotalLen,AvgAccLen,AvgDefLen,AvgTotalWeight,AvgAccWeight,AvgDefWeight"
                        // Lengths with 2 decimal places, weights with 1 decimal place.
                        snprintf(responseBuffer, sizeof(responseBuffer), "convSTAT:%.2f,%.2f,%.2f,%.1f,%.1f,%.1f",
                                 avgTotalLength, avgAcceptableLength, avgDefectiveLength,
                                 avgTotalWeight, avgAcceptableWeight, avgDefectiveWeight);
                        Serial.println(responseBuffer);
                        xSemaphoreGive(statsMutex);
                    } else {
                         //*Debug*// Serial.println("convSTAT:Error accessing statistics.");
                    }
                } else if (command.equals("reset")) {
                    //*Debug*// Serial.println("CMD: Resetting system...");
                    clearAllStats();
                    //*Debug*// Serial.println("System reset complete.");
                } else if (command.equals("emergency")) {
                    triggerEmergencyStop(); 
                } else {
                    // This is handled by triggerEmergencyStop()
                    Serial.println("Unknown command.");
                }
                xSemaphoreGive(serialMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // To avoid overusage of resources
    }
}

void addProductToReport(TrackedProduct product) {
    if (xSemaphoreTake(reportMutex, portMAX_DELAY) == pdTRUE) {
        bool found = false;
        for (int i = 0; i < productCountForReport; ++i) {
            if (product.barcodeID != 0 && productsForReport[i].barcodeID == product.barcodeID) { 
                productsForReport[i] = product; 
                found = true;
                break;
            } else if (product.barcodeID == 0 && productsForReport[i].tempID == product.tempID && productsForReport[i].barcodeID == 0) {
                productsForReport[i] = product;
                found = true;
                break;
            }
        }
        if (!found && productCountForReport < MAX_PRODUCTS_FOR_REPORT) {
            productsForReport[productCountForReport++] = product;
        } else if (!found) {
            // Serial.printf("[REPORT-WARN] Report list full or product BC %d (Temp %d) not added/updated.\n", product.barcodeID, product.tempID);
        }
        xSemaphoreGive(reportMutex);
    }
}

void setupPins() {
    pinMode(gate1CtrlPin, OUTPUT);
    pinMode(gate2CtrlPin, OUTPUT);
    digitalWrite(gate1CtrlPin, LOW); 
    digitalWrite(gate2CtrlPin, LOW);
    // g_obstructionSensorPin is set up in conveyor1ControlTask as INPUT together with ISR
}

void setupRTOSTasks() {
    // Core 0 Tasks
    xTaskCreatePinnedToCore(conveyor1ControlTask, "Conv1Ctrl", TASK_STACK_SIZE, NULL, TASK_PRIORITY_HIGH, &conveyor1TaskHandle, 0);
    xTaskCreatePinnedToCore(statusReportingTask, "StatusReport", TASK_STACK_SIZE, NULL, TASK_PRIORITY_MEDIUM, &statusUpdateTaskHandle, 0);
    xTaskCreatePinnedToCore(userCommandTask, "UserCmd", TASK_STACK_SIZE, NULL, TASK_PRIORITY_LOW, &userInterfaceTaskHandle, 0);
    
    // Core 1 Tasks
    xTaskCreatePinnedToCore(conveyor2ControlTask, "Conv2Ctrl", TASK_STACK_SIZE, NULL, TASK_PRIORITY_HIGH, &conveyor2TaskHandle, 1);
}

void clearAllStats() {
    // Take all relevant mutexes before clearing shared data
    if (xSemaphoreTake(statsMutex, portMAX_DELAY) == pdTRUE) {
        acceptCount = 0; rejectCount = 0;
        totalLengthSum = 0.0; acceptableLengthSum = 0.0; defectiveLengthSum = 0.0;
        totalWeightSum = 0.0; acceptableWeightSum = 0.0; defectiveWeightSum = 0.0;
        totalLengthCount = 0; acceptableLengthCount = 0; defectiveLengthCount = 0;
        totalWeightCount = 0; acceptableWeightCount = 0; defectiveWeightCount = 0;
        xSemaphoreGive(statsMutex);
    }
    if (xSemaphoreTake(conv1Mutex, portMAX_DELAY) == pdTRUE) {
        productCountConv1 = 0;
        nextTempID = 1;
        for(int i=0; i<MAX_PRODUCTS_ON_CONV1; ++i) productsOnConv1[i] = {}; 
        xSemaphoreGive(conv1Mutex);
    }
    if (conv2Mutex != NULL && xSemaphoreTake(conv2Mutex, portMAX_DELAY) == pdTRUE) {
        /* Conveyor2ControlTask locally assigned productsOnConv2 also need clearing if task is restarted
           However, productCountConv2 would be reset if task itself as new products replace the values.
        */
        xSemaphoreGive(conv2Mutex);
    }
    if (conv1ToConv2Queue != NULL) {
        xQueueReset(conv1ToConv2Queue); // Clear any pending items in the queue
    }
     if (xSemaphoreTake(reportMutex, portMAX_DELAY) == pdTRUE) {
        productCountForReport = 0;
        for(int i=0; i<MAX_PRODUCTS_FOR_REPORT; ++i) productsForReport[i] = {}; // Clear array
        xSemaphoreGive(reportMutex);
    }
    emergencyStop = false; // Reset emergency state
    g_gate1Ctrl = false;
    g_gate2Ctrl = false;
    digitalWrite(gate1CtrlPin, LOW);
    digitalWrite(gate2CtrlPin, LOW);
}

void triggerEmergencyStop() {
    if (!emergencyStop) { // Tp ensure this logic runs only once per emergency event
        emergencyStop = true;
        // Close gates
        g_gate1Ctrl = false; 
        g_gate2Ctrl = false;
        digitalWrite(gate1CtrlPin, LOW);
        digitalWrite(gate2CtrlPin, LOW);
        Serial.println("EMERGENCY: Conveyor halted");   
        // Tasks will see emergencyStop flag and stop processing.    
    }
}

// --- Main Setup and Loop ---
void setup() {
    Serial.begin(115200);
    unsigned long startTime = millis();
    while (!Serial && (millis() - startTime < 2000)); // Wait for serial connection, timeout after 2s
    
    Serial.println("\nSystem Initializing...");
    
    // Create Mutexes and Queues
    conv1Mutex = xSemaphoreCreateMutex();
    // conv2Mutex is created inside conveyor2ControlTask
    reportMutex = xSemaphoreCreateMutex();
    statsMutex = xSemaphoreCreateMutex();
    serialMutex = xSemaphoreCreateMutex();
    conv1ToConv2Queue = xQueueCreate(MAX_PRODUCTS_ON_CONV1 + 2, sizeof(TrackedProduct)); // Queue size, bit larger just in case
    
    if (!conv1Mutex || !reportMutex || !statsMutex || !serialMutex || !conv1ToConv2Queue) {
        Serial.println("FATAL ERROR: Failed to create RTOS Kernel objects!");
        while(1) { delay(1000); } // Stop Everything
    }
    //Pin stepup
    setupPins();
    
    // Initialize stats and product counts 
    clearAllStats(); 
    
    //Create and setup Tasks 
    setupRTOSTasks();
    
    // Starting Simulator
    startSimulator(); 
    Serial.println("Dual-Stage Smart Conveyor System Initialized and Running.");
}

void loop() {
    // Empty loop as FreeRTOS tasks are managing the system.
    vTaskDelay(portMAX_DELAY);
}


/********************************************END OF  CODE**************************************************************************/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!ANY CODE BELOW THIS LINE SHOULD NOT BE MODIFIED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*********************************START OF SIMULATOR PLEASE DON'T CHANGE ANYTHING BELOW THIS LINE********************************/

// *** START OF UPDATED SIMULATOR CODE for ESP32 Arduino Core 3.0.0+ ***
// The original hardware timer API (hw_timer_t, timerBegin, timerAttachInterruptArg, etc.)
// is deprecated in Core 3.0.0+. It has been replaced by the esp_timer API.

volatile int currentProductIndex = 0;  // Index of the current product to be loaded
// Create multiple schedulers for overlapping products
const int MAX_SCHEDULERS = 4;  // Need at least 3 schedulers for 1200ms/450ms < 3 overlapping products
ProductScheduler g_schedulers[MAX_SCHEDULERS];
volatile int currentSchedulerIndex = 0;

// Convert distance and conveyor speed (m/s or cm/s) to microseconds
static inline uint64_t distanceToMicros(float distance, float speed) {
    // distance / speed = time(s) => convert to microseconds
    double tSec = (double)distance / (double)speed;
    return (uint64_t)(tSec * 1e6);
}

void fillProductEvents(ProductScheduler &sch) {
    const Product &p = sch.product;
    sch.totalEvents = 0;
    sch.currentEventIndex = 0;

    // For convenience
    auto &events = sch.events;
    int idx = 0;

    uint64_t leadTime = distanceToMicros(DISTANCE_TO_OBSTRUCTION_SENSOR, CONVEYOR1_SPEED);


    // 1. Product front edge reaches obstruction sensor
    events[idx++] = {
        .triggerTimeMicros = leadTime,
        .eventType         = ProductEventType::OBSTRUCTION_SENSOR_RISE
    };

    // 2. Product back edge leaves obstruction sensor
    uint64_t leaveTime = distanceToMicros(p.length, CONVEYOR1_SPEED);
    events[idx++] = {
        .triggerTimeMicros = leaveTime+leadTime,
        .eventType         = ProductEventType::OBSTRUCTION_SENSOR_FALL
    };

    // 3. Barcode reading window
    //   Example: Distance from obstruction sensor to barcode sensor = d_barcode
    //   Time when product center reaches barcode sensor => 0 + d_barcode/speed + (length/2)/speed
    uint64_t centerAtBarcode = distanceToMicros(DISTANCE_BARCODE + p.length/2.0f, CONVEYOR1_SPEED);
    // +/- 20% length
    uint64_t halfWindow = distanceToMicros(p.length * 0.1f, CONVEYOR1_SPEED) ;
    events[idx++] = {
        .triggerTimeMicros = centerAtBarcode - halfWindow+leadTime,
        .eventType         = ProductEventType::BARCODE_SENSOR_ENTER_WINDOW
    };
    events[idx++] = {
        .triggerTimeMicros = centerAtBarcode + halfWindow+leadTime,
        .eventType         = ProductEventType::BARCODE_SENSOR_EXIT_WINDOW
    };

    // 4. Gate1 check timing
    uint64_t gate1Time = distanceToMicros(DISTANCE_GATE1, CONVEYOR1_SPEED);
    events[idx++] = {
        .triggerTimeMicros = gate1Time+leadTime,
        .eventType         = ProductEventType::GATE1_CHECK
    };

    // 5. Transfer to conveyor2
    uint64_t transferTime = distanceToMicros(DISTANCE_TRANSFER, CONVEYOR1_SPEED);
    events[idx++] = {
        .triggerTimeMicros = transferTime+leadTime,
        .eventType         = ProductEventType::TRANSFER_TO_CONVEYOR2
    };

    // 6. Weight sensor reading window (on conveyor2)
    uint64_t centerAtWeight = transferTime + distanceToMicros(DISTANCE_WEIGHT_SENSOR + p.length/2.0f, CONVEYOR2_SPEED);
    uint64_t halfWindow2 = distanceToMicros(p.length * 0.1f, CONVEYOR2_SPEED) ;
    events[idx++] = {
        .triggerTimeMicros = centerAtWeight - halfWindow2+leadTime,
        .eventType         = ProductEventType::WEIGHT_SENSOR_ENTER_WINDOW
    };
    events[idx++] = {
        .triggerTimeMicros = centerAtWeight + halfWindow2+leadTime,
        .eventType         = ProductEventType::WEIGHT_SENSOR_EXIT_WINDOW
    };

    // 7. Gate2 check
    uint64_t gate2Time = transferTime + distanceToMicros(DISTANCE_GATE2, CONVEYOR2_SPEED);
    events[idx++] = {
        .triggerTimeMicros = gate2Time+leadTime,
        .eventType         = ProductEventType::GATE2_CHECK
    };

    // 8. End of production line
    events[idx++] = {
        .triggerTimeMicros = gate2Time + distanceToMicros(0.3f, CONVEYOR2_SPEED)+leadTime,
        .eventType         = ProductEventType::FINISH
    };

    sch.totalEvents = idx;

    // Sort events by time (in case insertion order was mixed)
    for(int i=0; i<sch.totalEvents-1; i++){
        for(int j=i+1; j<sch.totalEvents; j++){
            if(events[i].triggerTimeMicros > events[j].triggerTimeMicros){
                ProductEvent tmp = events[i];
                events[i] = events[j];
                events[j] = tmp;
            }
        }
    }
}

void printProductEvents(const ProductScheduler &sch) {
  Serial.println("===== Product Event Timeline =====");
  Serial.print("Length: "); Serial.print(sch.product.length); Serial.println(" cm");
  Serial.print("Weight: "); Serial.print(sch.product.weight); Serial.println(" g");
  Serial.print("Barcode: "); Serial.println(sch.product.barcodeID);
  Serial.println();
  
  Serial.println("Events:");
  for (int i = 0; i < sch.totalEvents; i++) {
    const ProductEvent &evt = sch.events[i];
    float timeMs = evt.triggerTimeMicros / 1000.0f;
    
    Serial.print(i); Serial.print(": ");
    Serial.print(timeMs); Serial.print(" ms - ");
    
    switch (evt.eventType) {
      case ProductEventType::OBSTRUCTION_SENSOR_RISE:
        Serial.println("OBSTRUCTION_SENSOR_RISE");
        break;
      case ProductEventType::OBSTRUCTION_SENSOR_FALL:
        Serial.println("OBSTRUCTION_SENSOR_FALL");
        break;
      case ProductEventType::BARCODE_SENSOR_ENTER_WINDOW:
        Serial.println("BARCODE_SENSOR_ENTER_WINDOW");
        break;
      case ProductEventType::BARCODE_SENSOR_EXIT_WINDOW:
        Serial.println("BARCODE_SENSOR_EXIT_WINDOW");
        break;
      case ProductEventType::GATE1_CHECK:
        Serial.println("GATE1_CHECK");
        break;
      case ProductEventType::TRANSFER_TO_CONVEYOR2:
        Serial.println("TRANSFER_TO_CONVEYOR2");
        break;
      case ProductEventType::WEIGHT_SENSOR_ENTER_WINDOW:
        Serial.println("WEIGHT_SENSOR_ENTER_WINDOW");
        break;
      case ProductEventType::WEIGHT_SENSOR_EXIT_WINDOW:
        Serial.println("WEIGHT_SENSOR_EXIT_WINDOW");
        break;
      case ProductEventType::GATE2_CHECK:
        Serial.println("GATE2_CHECK");
        break;
      case ProductEventType::FINISH:
        Serial.println("FINISH");
        break;
      default:
        Serial.println("UNKNOWN_EVENT");
    }
  }
  Serial.println("=======================");
}

// *** REPLACED Timer Callback function – uses esp_timer instead of hw_timer_t ***
void onTimerInterrupt(void* arg) {
    // Retrieve the scheduler object
    ProductScheduler* sch = (ProductScheduler*)arg;
    esp_timer_handle_t timer = sch->timerHandle;

    if(!timer) {
        // Timer handle is invalid
        return;
    }

    // If no more events, stop and delete the timer
    if(sch->currentEventIndex >= sch->totalEvents) {
        esp_timer_stop(timer);
        esp_timer_delete(timer);
        sch->timerHandle = nullptr;
        return;
    }

    // Get current event
    ProductEvent evt = sch->events[sch->currentEventIndex];
    sch->currentEventIndex++;

    // Execute simulation logic for the corresponding event
    switch(evt.eventType) {
        case ProductEventType::OBSTRUCTION_SENSOR_RISE:
            g_obstructionSensor = true;
            digitalWrite(g_obstructionSensorPin, g_obstructionSensor);
            break;

        case ProductEventType::OBSTRUCTION_SENSOR_FALL:
            g_obstructionSensor = false;
            digitalWrite(g_obstructionSensorPin, g_obstructionSensor);
            break;

        case ProductEventType::BARCODE_SENSOR_ENTER_WINDOW:
            g_barcodeReader = sch->product.barcodeID; // Start returning valid barcode
            break;

        case ProductEventType::BARCODE_SENSOR_EXIT_WINDOW:
            g_barcodeReader = 0; // Exit window
            break;

        case ProductEventType::GATE1_CHECK:
            {
                // Use macro definitions to check if length is acceptable
                bool isLenOk = (sch->product.length >= LENGTH_MIN && sch->product.length <= LENGTH_MAX);
                bool shouldOpen = isLenOk;  // Acceptable => Gate1 should open
                // Compare with g_gate1Ctrl, record errors or perform other processing if inconsistent
            }
            break;

        case ProductEventType::TRANSFER_TO_CONVEYOR2:
            // Product has entered conveyor2, status can be recorded here
            break;

        case ProductEventType::WEIGHT_SENSOR_ENTER_WINDOW:
            g_weightSensor = sch->product.weight; // Return valid weight
            break;

        case ProductEventType::WEIGHT_SENSOR_EXIT_WINDOW:
            g_weightSensor = 0; // Exit window
            break;

        case ProductEventType::GATE2_CHECK:
            {
                // Use macro definitions to check if weight is acceptable
                bool isWeightOk = (sch->product.weight >= WEIGHT_MIN && sch->product.weight <= WEIGHT_MAX);
                bool shouldOpen = isWeightOk;
                // Compare with g_gate2Ctrl
            }
            break;

        case ProductEventType::FINISH:
            // Product leaves the production line, can be counted or other processing
            break;
    }
    digitalWrite(sch->debugPin, !digitalRead(sch->debugPin));
    
    // If there is a next event, arm the timer with the relative delay
    if(sch->currentEventIndex < sch->totalEvents) {
        // Compute elapsed time since scheduling started
        uint64_t now = esp_timer_get_time();
        int64_t elapsed = (int64_t)(now - sch->startTimeUs);
        // Next event absolute time (relative to scheduling start)
        int64_t nextAbs = sch->events[sch->currentEventIndex].triggerTimeMicros;
        int64_t delay = nextAbs - elapsed;
        if (delay < 0) delay = 0; // Safety clamp
        esp_timer_start_once(timer, delay);
    } else {
        // No more events, clean up the timer
        esp_timer_stop(timer);
        esp_timer_delete(timer);
        sch->timerHandle = nullptr;
    }
}

// *** REPLACED allocateTimerForProduct – uses esp_timer_create instead of deprecated hardware timer ***
esp_timer_handle_t allocateTimerForProduct(ProductScheduler &sch) {
    digitalWrite(sch.debugPin, HIGH);
    fillProductEvents(sch);
    
    // Configure the esp_timer
    esp_timer_create_args_t timer_args = {
        .callback = &onTimerInterrupt,
        .arg = (void*)&sch,
        .dispatch_method = ESP_TIMER_TASK,  // Runs in a dedicated timer task, not ISR
        .name = "prodTimer"
    };
    
    esp_timer_handle_t timer_handle = nullptr;
    esp_err_t err = esp_timer_create(&timer_args, &timer_handle);
    if (err != ESP_OK) {
        return nullptr;
    }
    
    sch.timerHandle = timer_handle;
    // Record the time at which scheduling begins – used later for relative delays
    sch.startTimeUs = esp_timer_get_time();
    
    // If there are events, start the timer with the first event's absolute time
    if (sch.totalEvents > 0) {
        uint64_t firstEventTime = sch.events[0].triggerTimeMicros;
        esp_timer_start_once(timer_handle, firstEventTime);
    }
    
    digitalWrite(sch.debugPin, LOW);
    return timer_handle;
}

// Product loading task
void productLoadingTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while (true) {
        if (currentProductIndex < MAX_PRODUCTS) {
            // Find an available scheduler (one that's not currently handling events)
            bool schedulerFound = false;
            for (int i = 0; i < MAX_SCHEDULERS; i++) {
                if (g_schedulers[i].timerHandle == nullptr) {
                    // This scheduler is free
                    g_schedulers[i].product = productArray[currentProductIndex];
                    g_schedulers[i].debugPin = DEBUG_PIN+currentProductIndex%3;
                    
                    // Allocate timer (now using esp_timer)
                    esp_timer_handle_t t = allocateTimerForProduct(g_schedulers[i]);
                    if (!t) {
                        Serial.println("ERROR: No timer available!");
                    } else {
                        currentProductIndex++;
                        schedulerFound = true;
                    }
                    break;
                }
            }            
            if (!schedulerFound) {
                Serial.println("WARNING: No available scheduler found, skipping this cycle");
            }
        }        
        // Wait for the next cycle using vTaskDelayUntil
        vTaskDelayUntil(&xLastWakeTime, 450/portTICK_PERIOD_MS);
    }
}

void startSimulator(void) {
    pinMode(g_obstructionSensorPin, OUTPUT);
    pinMode(DEBUG_PIN, OUTPUT);
    pinMode(DEBUG_PIN+1, OUTPUT);
    pinMode(DEBUG_PIN+2, OUTPUT);
    digitalWrite(DEBUG_PIN, LOW);   
    digitalWrite(DEBUG_PIN+1, LOW);
    digitalWrite(DEBUG_PIN+2, LOW);
    // Initialise global sensor outputs
    g_obstructionSensor = false;
    g_barcodeReader = 0;
    g_weightSensor = 0;

    // Initialise gate controls
    g_gate1Ctrl = false;
    g_gate2Ctrl = false;

    // Reset indices and initialise schedulers (timerHandle is now esp_timer_handle_t)
    currentProductIndex = 0;
    for (int i = 0; i < MAX_SCHEDULERS; i++) {
        g_schedulers[i].timerHandle = nullptr;
    }
    // Create the product loading task with high priority
    xTaskCreate(
        productLoadingTask,        // Task function
        "ProductLoader",           // Task name
        2048,                      // Stack size (bytes)
        NULL,                      // Task parameters
        configMAX_PRIORITIES - 1,  // Task priority (highest)
        NULL                       // Task handle
    );
}