#include <FaceDetect_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#else
#error "Camera model not selected"
#endif

/* OLED Display Settings ------------------------------------------------- */
#define OLED_SDA          13    // GPIO13 for indicator
#define OLED_SCL          15    // GPIO15 is available (not used by camera)
#define OLED_RESET        -1    // Use internal reset
#define SCREEN_WIDTH      128   // OLED display width, in pixels
#define SCREEN_HEIGHT     64    // OLED display height, in pixels

// Create OLED display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

/* Door Lock System Pin Definitions --------------------------------------- */
#define RELAY_PIN          2    // GPIO2 for relay control (solenoid door lock)
#define RED_LED_PIN        12   // GPIO12 for red LED
#define BUZZER_PIN         14   // GPIO14 for buzzer

/* System Variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf; //points to the output of the capture

// Variables for door lock state management
bool doorLocked = true; // door is locked
bool lastFaceClearDetected = false;
unsigned long lastDetectionTime = 0;
const unsigned long DOOR_UNLOCK_DURATION = 5000; // 5 seconds door unlock duration
const float CONFIDENCE_THRESHOLD = 0.7; // 70% confidence threshold

// Simple OLED display variables
unsigned long lastOLEDUpdate = 0;
const unsigned long OLED_UPDATE_INTERVAL = 100; // Fast updates every 100ms
String currentDoorStatus = "LOCKED";
String currentSystemStatus = "Ready";
String lastFaceDetected = "None";
unsigned long doorUnlockTimeLeft = 0; // Time left until door locks

// Optimization variables
unsigned long lastFrameTime = 0;
const unsigned long MIN_FRAME_INTERVAL = 50; // 20 FPS max (50ms between frames)

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 10, // Reduced from 12 to 10 for faster encoding (0-63 lower number means higher quality)
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) ;

/* SIMPLE OLED Display Functions ---------------------------------- */
void setupOLED() {
    Wire.begin(OLED_SDA, OLED_SCL);
    
    // Initialize OLED display with minimal delay
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("OLED init failed"));
        return;
    }
    
    // Quick startup screen
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 10);
    display.println("Face Lock System");
    display.setCursor(10, 30);
    display.println("Starting...");
    display.display();
    delay(500); // Reduced delay
    Serial.println("OLED Ready");
}

void updateOLEDSimple() {
    // Update door unlock timer
    if (!doorLocked) {
        unsigned long timeElapsed = millis() - lastDetectionTime;
        if (timeElapsed < DOOR_UNLOCK_DURATION) {
            doorUnlockTimeLeft = (DOOR_UNLOCK_DURATION - timeElapsed) / 1000;
        } else {
            doorUnlockTimeLeft = 0;
        }
    } else {
        doorUnlockTimeLeft = 0;
    }
    
    // Display
    display.clearDisplay();
    // Title
    display.setTextSize(1);
    display.setCursor(20, 0);
    display.println("FACE LOCK SYSTEM");
    // Draw simple separator line
    display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
    // Door Status (BIG TEXT)
    display.setTextSize(2);
    display.setCursor(20, 15);
    if (doorLocked) {
        display.println("LOCKED");
    } else {
        display.println("UNLOCKED");
    }
    
    // Status info
    display.setTextSize(1);
    display.setCursor(0, 35);
    display.print("Status: ");
    display.println(currentSystemStatus);
    display.setCursor(0, 45);
    display.print("Face: ");
    display.println(lastFaceDetected);
    // Timer display when unlocked
    if (!doorLocked && doorUnlockTimeLeft > 0) {
        display.setCursor(0, 55);
        display.print("Lock in: ");
        display.print(doorUnlockTimeLeft);
        display.println("s");
    }

    display.display();
}

/* Door Lock System Functions ---------------------------------------------- */
void setupDoorSystem() {
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    // Initialize all outputs to OFF state
    digitalWrite(RELAY_PIN, LOW); // Relay OFF (solenoid locked)
    digitalWrite(RED_LED_PIN, LOW); // Red LED OFF
    digitalWrite(BUZZER_PIN, LOW); // Buzzer OFF
    Serial.println("Door System: LOCKED");
}

void unlockDoor() {
    digitalWrite(RELAY_PIN, HIGH); // Relay ON (solenoid unlocked)
    digitalWrite(RED_LED_PIN, HIGH); // Red LED ON
    doorLocked = false;
    lastDetectionTime = millis();
    currentDoorStatus = "UNLOCKED";
    currentSystemStatus = "Access Granted";
    Serial.println("Door UNLOCKED");
    // Short beep
    tone(BUZZER_PIN, 1000, 200);
}

void lockDoor() {
    digitalWrite(RELAY_PIN, LOW); // Relay OFF (solenoid locked)
    digitalWrite(RED_LED_PIN, LOW); // Red LED OFF
    doorLocked = true;
    
    currentDoorStatus = "LOCKED";
    currentSystemStatus = "Ready";
    
    Serial.println("Door LOCKED");
    
    // Short beep
    tone(BUZZER_PIN, 800, 200);
}

void checkDoorTimeout() {
    if (!doorLocked && (millis() - lastDetectionTime > DOOR_UNLOCK_DURATION)) {
        lockDoor();
        currentSystemStatus = "Auto Locked";
    }
}

/**
* @brief      Arduino setup function
*/
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    //comment out the below line to start inference immediately after upload
    while (!Serial);
    Serial.println("Face Recognition Door Lock");
    
    // Initialize OLED display
    setupOLED();
    
    // Initialize door lock system
    setupDoorSystem();
    
    if (ei_camera_init() == false) {
        ei_printf("Camera init failed!\r\n");
        currentSystemStatus = "Camera Error";
        updateOLEDSimple();
    }
    else {
        ei_printf("Camera ready\r\n");
        currentSystemStatus = "Camera Ready";
        updateOLEDSimple();
    }

    // Pre-allocate snapshot buffer
    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
    
    if(snapshot_buf == nullptr) {
        Serial.println("Memory Error!");
        currentSystemStatus = "Memory Error";
    }

    ei_printf("\nStarting...\n");
    currentSystemStatus = "Starting...";
    updateOLEDSimple();
    
    delay(1000); // Reduced startup delay
}
/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop()
{
    // OLED Display
    if (millis() - lastOLEDUpdate > OLED_UPDATE_INTERVAL) {
        updateOLEDSimple();
        lastOLEDUpdate = millis();
    }
    
    // Check if door should be locked due to timeout
    checkDoorTimeout();

    // Frame rate limiting
    unsigned long currentTime = millis();
    if (currentTime - lastFrameTime < MIN_FRAME_INTERVAL) {
        delay(1);
        return;
    }
    lastFrameTime = currentTime;

    if (ei_sleep(5) != EI_IMPULSE_OK) {
        return;
    }

    // Use pre-allocated buffer
    if (snapshot_buf == nullptr) {
        snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
        
        if(snapshot_buf == nullptr) {
            ei_printf("Buffer Error!\n");
            currentSystemStatus = "Buffer Error";
            return;
        }
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        ei_printf("Capture failed\r\n");
        currentSystemStatus = "Capture Error";
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("Classifier error (%d)\n", err);
        currentSystemStatus = "Inference Error";
        return;
    }

    // SIMPLE DETECTION LOGIC - Fast
    bool face_clear_detected = false;
    bool face_covering_detected = false;
    float highestConfidence = 0.0;
    String detectedFace = "None";

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        
        if (bb.value > highestConfidence) {
            highestConfidence = bb.value;
            detectedFace = bb.label;
        }
        
        if (strstr(bb.label, "face_clear") != NULL && bb.value > CONFIDENCE_THRESHOLD) {
            face_clear_detected = true;
        }
        else if (strstr(bb.label, "face_covering") != NULL && bb.value > CONFIDENCE_THRESHOLD) {
            face_covering_detected = true;
        }
    }

#else
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        float confidence = result.classification[i].value;
        
        if (confidence > highestConfidence) {
            highestConfidence = confidence;
            detectedFace = ei_classifier_inferencing_categories[i];
        }
        
        if (strstr(ei_classifier_inferencing_categories[i], "face_clear") != NULL) {
            if (confidence > CONFIDENCE_THRESHOLD) {
                face_clear_detected = true;
            }
        }
        else if (strstr(ei_classifier_inferencing_categories[i], "face_covering") != NULL) {
            if (confidence > CONFIDENCE_THRESHOLD) {
                face_covering_detected = true;
            }
        }
    }
#endif

    if (highestConfidence > 0.3) {
        lastFaceDetected = detectedFace;
        currentSystemStatus = "Detecting...";
    } else {
        lastFaceDetected = "None";
        if (doorLocked) {
            currentSystemStatus = "Ready";
        }
    }

    // Control door lock
    if (face_clear_detected && !face_covering_detected) {
        if (doorLocked || !lastFaceClearDetected) {
            unlockDoor();
        }
        lastFaceClearDetected = true;
    } 
    else if (face_covering_detected) {
        if (!doorLocked) {
            lockDoor();
        }
        lastFaceClearDetected = false;
        currentSystemStatus = "Covered Face";
        tone(BUZZER_PIN, 500, 1000);
    }
    else {
        lastFaceClearDetected = false;
    }
}
/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {

    if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x\n", err);
      return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1);
      s->set_brightness(s, 1);
      s->set_saturation(s, 0);
    }

    if (s->id.PID == OV2640_PID) {
      s->set_brightness(s, 0);
      s->set_contrast(s, 0);
      s->set_saturation(s, 0);
      s->set_special_effect(s, 0);
      s->set_whitebal(s, 1);
      s->set_awb_gain(s, 1);
      s->set_wb_mode(s, 0);
      s->set_exposure_ctrl(s, 1);
      s->set_aec2(s, 0);
      s->set_ae_level(s, 0);
      s->set_aec_value(s, 300);
      s->set_gain_ctrl(s, 1);
      s->set_agc_gain(s, 0);
      s->set_gainceiling(s, (gainceiling_t)0);
      s->set_bpc(s, 0);
      s->set_wpc(s, 1);
      s->set_raw_gma(s, 1);
      s->set_lenc(s, 1);
      s->set_hmirror(s, 0);
      s->set_vflip(s, 0);
      s->set_dcw(s, 1);
      s->set_colorbar(s, 0);
    }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_awb_gain(s, 1);
#endif

    is_initialised = true;
    return true;
}
/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {

    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK)
    {
        ei_printf("Camera deinit failed\n");
        return;
    }

    is_initialised = false;
    
    if (snapshot_buf != nullptr) {
        free(snapshot_buf);
        snapshot_buf = nullptr;
    }
    
    return;
}


/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

   bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

   esp_camera_fb_return(fb);

   if(!converted){
       ei_printf("Conversion failed\n");
       return false;
   }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];

        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    return 0;
}