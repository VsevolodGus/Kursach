#include <WiFi.h>                                                       // библиотека для работы с WiFi
#include <WiFiClientSecure.h>                                                 // библиотека для работы с WiFi клиентом с поддержкой TSL
#include "UniversalTelegramBot.h"                                             // библиотека для управления Telegram ботом

#include <time.h>                                                   // библиотека для получения времени от NTP сервера
#include "SPIFFS.h"                                                         // библиотека для работы с SPIFFS памятью
#include "esp_wifi.h"                                                           // библиотека для расширенной работы с WiFi на плате ESP32-CAM
#include "esp_system.h"                                                 // библиотека для настройки компонентов системы
#include "esp_camera.h"                                                   // библиотека для работы с камерой на плате ESP32-CAM

// библиотеки для работы с детектором отключения питания
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include <ArduinoJson.h> // библиотека для работы с JSON

// указываем пины к которым подключена камера на плате ESP32-CAM Ai Thinker




#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

#define timezone 1                                                                // ваш часовой пояс
#define pirSensorPin 13                                                             // пин к которому подключен PIR датчик
#define getMessageInterval 1000                                                     // интервал получения сообщений от Telegram пользователя

const char WiFiSSID[] = "Xiaomi_Mi";                                                    // название вашей WiFi сети
const char WiFiPASS[] = "2444666668888888";                                       // пароль вашей WiFi сети
const String botToken = "5819955943:AAG356TK1hvi_bFVykh0oCQNf8JmzMJchjc";           // токен вашего Telegram бота
const String yourChatIDs[] = {"737740775"};                                       // chat id Telegram пользователей, которые могут управлять вашим Telegram ботом   "809045959" , , "809045959" "416621375"  737740775    737740775

WiFiClientSecure client; // создаём WiFi клиент с поддержкой TSL
UniversalTelegramBot bot(botToken, client); // создаём объект для управления вашим Telegram ботом

String videoFileName; // переменная для хранения названия видео файла, который плата ESP32-CAM отправит вам на Telegram
String minutes; // переменная для хранения минут
String hours; // переменная для хранения часов
String days; // переменная для хранения дней с начала месяца
String month; // переменная для хранения номера месяца
int year; // переменная для хранения года

String settings; // переменная для хранения параметров изображения, параметров сигнализации и ускорения видео
int firstHashIndex; // переменная для хранения индекса первого разделительного хештега
int secondHashIndex; // переменная для хранения индекса второго разделительного хештега
int thirdHashIndex; // переменная для хранения индекса третьего разделительного хештега
int fourthHashIndex; // переменная для хранения индекса четвёртого разделительного хештега
int fifthHashIndex; // переменная для хранения индекса пятого разделительного хештега
int sixthHashIndex; // переменная для хранения индекса шестого разделительного хештега
int seventhHashIndex; // переменная для хранения индекса седьмого разделительного хештега
int eighthHashIndex; // переменная для хранения индекса восьмого разделительного хештега
int ninthHashIndex; // переменная для хранения индекса девятого разделительного хештега

String message; // переменная для хранения сообщения, полученного от Telegram пользователя
String chatID; // переменная для хранения chat id Telegram пользователя, который отправил сообщение вашему Telegram боту
String videoChatID; // переменная для хранения chat id Telegram пользователя, который отправил команду на запись и отправку видео на Telegram
String alarmChatID; // переменная для хранения chat id Telegram пользователя, который отправил команду на запуск сигнализации движения
String keyboardJson; // переменная для хранения Telegram клавиатуры в формате JSON
String settingsMessage; // переменная для хранения сообщения с параметрами, которое мы отправим вам на Telegram

int framesizeInt; // переменная для хранения разрешения изображения как int
String framesize = "VGA"; // переменная для хранения разрешения изображения как строка
String flipImage = "Нет"; // переменная для хранения того, будет ли переворачиваться изображение
String brightness = "100%"; // переменная для хранения яркости изображения
String turnOnFlash = "Нет"; // переменная для хранения того, будет ли включаться вспышка для фото
String specialEffect = "Обычный"; // переменная для хранения спец эффекта изображения
String whiteBalanceMode = "Авто баланс"; // переменная для хранения режима баланса белого изображения

int pirSensorDelay = 15; // переменная для хранения задержки PIR датчика
bool setImageFlipMode; // переменная для хранения того, запущен ли режим для настройки того, будет ли переворачиваться изображение
bool setPirSensorDelayMode; // переменная для хранения того, запущен ли режим для настройки задержки PIR датчика
bool isAlarmRunning = false; // переменная для хранения того, запущена ли сигнализация движения как boolean
String alarmRunning = "Нет"; // переменная для хранения того, запущена ли сигнализация движения как строка
String alarmMessageType = "Видео"; // переменная для хранения типа сообщения сигнализации движения

bool flashState; // переменная для хранения того, включена ли вспышка

int frameInterval = 125; // переменная для хранения интервала кадров в видео в миллисекундах
float videoSpeedFloat = 1; // переменная для хранения ускорения видео как float
String videoSpeedString = "1x"; // переменная для хранения ускорения видео как строка

// константа для хранения команд вашего Telegram бота
const String botCommands = F("["
  "{\"command\":\"takephoto\", \"description\":\"отправить фото\"},"
  "{\"command\":\"recordvideo\", \"description\":\"отправить видео\"},"
  "{\"command\":\"alarm\", \"description\":\"управление сигнализацией\"},"
  "{\"command\":\"flash\", \"description\":\"управление вспышкой\"},"
  "{\"command\":\"videospeed\", \"description\":\"ускорение видео\"},"
  "{\"command\":\"alarmsettings\", \"description\":\"параметры сигнализации\"},"
  "{\"command\":\"imagesettings\", \"description\":\"параметры изображения\"}"
  "]");

unsigned long lastMessageMillis; // переменная для хранения времени получения последнего сообщения от Telegram пользователя
unsigned long pirSensorDelayMillis; // переменная для хранения времени последней проверки того, обнаружено ли движение

sensor_t * camera = NULL; // объект для настройки параметров изображения
camera_fb_t * fb = NULL; // объект для хранения фотографии, которую мы отправим на Telegram
camera_fb_t * videoFb = NULL; // переменная для хранения кадра видео
camera_fb_t * nextFb = NULL; // переменная для хранения следующего кадра видео
camera_fb_t * currentFb = NULL; // переменная для хранения текущего кадра видео

int videoBufferSize;
int idxBufferSize;

int videoPtr;
uint8_t* videoBuffer;
size_t videoFileLength;

uint8_t * psramVideoBuffer = NULL;
uint8_t * psramIdxBuffer = NULL;
uint8_t * psramVideoPtr = 0;
uint8_t * psramIdxPtr = 0;

bool videoRecorded; // переменная для хранения того, завершилась ли запись видео
bool motionDetected; // переменная для хранения того, обнаружено ли сейчас движение

float fps; // переменная для хранения частоты кадров в видео
float realFPS; // переменная для хранения реальной частоты кадров в видео
uint8_t attainedFPS; // переменная для хранения округлённой частоты кадров в видео
uint32_t attainedPerFrame; // переменная для хранения округлённой длительности кадра в видео в микросекундах
float microsecondsPerFrame; // переменная для хранения длительности кадра в видео в микросекундах

int getGoodPictureFailures; // переменная для хранения количества неудачных попыток получить хороший кадр для видео

int fbBlockLength;
int jpegSizeRemnant;
uint8_t* fbBlockStart;

unsigned long videoStartMillis; // переменная для хранения времени начала записи видео
unsigned long videoEndMillis; // переменная для хранения времени завершения записи видео
unsigned long lastFrameMillis; // переменная для хранения времени получения последнего кадра видео
unsigned long frameWriteMillis; // переменная для хранения времени записи кадра видео в PSRAM
unsigned long currentFrameMillis; // переменная для хранения времени получения текущего кадра видео
unsigned long startMillis;
unsigned long fbBlockMillis;

uint32_t videoDuration; // переменная для хранения длительности записи видео в миллисекундах
uint16_t frameCount; // переменная для хранения количества кадров в видео
uint16_t remnant;
uint32_t videoLength; 

unsigned long videoSize;
unsigned long idxOffset;
unsigned long jpegSize;

uint8_t dcBuffer[4] = {0x30, 0x30, 0x64, 0x63};
uint8_t zeroBuffer[4] = {0x00, 0x00, 0x00, 0x00};
uint8_t idx1Buffer[4] = {0x69, 0x64, 0x78, 0x31};
uint8_t video1Buffer[4] = {0x41, 0x56, 0x49, 0x31};

bool isMoreDataAvailable();

int currentByte;
uint8_t* fbBuffer;
size_t fbLength;

// функция для настройки и запуска камеры на плате ESP32-CAM
bool setupCamera() {
  camera_config_t config; // создаём объект для настройки камеры на плате ESP32-CAM
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000; // настраиваем частоту камеры
  config.pixel_format = PIXFORMAT_JPEG; // настраиваем формат изображения (PIXFORMAT_ + YUV422|GRAYSCALE|RGB565|JPEG)

  config.frame_size = FRAMESIZE_VGA; // настраиваем разрешение изображения (FRAMESIZE_ + UXGA|SXGA|XGA|HD|SVGA|VGA|HVGA|CIF|QVGA|240X240|HQVGA|QCIF|QQVGA|96X96)

  if (psramFound()) { // если обнаружен достаточный размер PSRAM
    config.jpeg_quality = 5; // настраиваем качество изображения (1 - 63, меньше число, выше качество изображения)
    config.fb_count = 4; // настраиваем максимальное количество фото, которое можно сохранить в PSRAM
  } else {
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  static char * memTmp1 = (char*) malloc(32 * 1024);
  static char * memTmp2 = (char*) malloc(32 * 1024);

  esp_err_t err = esp_camera_init(&config); // запускаем камеру на плате ESP32-CAM
  if (err != ESP_OK) { // если запустить камеру на плате ESP32-CAM не удалось
    Serial.printf("Возникла следующая ошибка камеры: 0x%x ", err);
    return false; // возвращаем то, что запустить камеру на плате ESP32-CAM не удалось
  }

  // выделяем место в PSRAM
  free(memTmp2);
  memTmp2 = NULL;
  free(memTmp1);
  memTmp1 = NULL;

  camera = esp_camera_sensor_get(); // получаем модуль камеры на плате ESP32-CAM
  camera->set_quality(camera, 10); // устанавливаем качество изображения
  delay(500);
  return true; // возвращаем то, что камера на плате ESP32-CAM запущена
}

// функция для отправки фото на Telegram
void sendPhotoToTelegram() {
  if (turnOnFlash == "Да") { // если включение вспышки для фото установлено
    digitalWrite(flashPin, HIGH); // включаем вспышку
  } else {
    digitalWrite(flashPin, LOW); // выключаем вспышку
  }
  delay(50); // ждём 50 миллисекунд, пока вспышка включится

  fb = NULL; // сбрасываем фото
  fb = getGoodPicture(); // получаем и записываем фото
  digitalWrite(flashPin, LOW); // выключаем вспышку

  if (fb) { // если фото успешно получено
    currentByte = 0; // обнуляем текущий байт
    fbLength = fb->len; // записываем размер фото
    fbBuffer = fb->buf; // записываем буфер фото

    if (motionDetected) { // если обнаружено движение
      bot.sendChatAction(alarmChatID, "upload_photo"); // отправляем Telegram пользователю, который запустил сигнализацию, действие в чате о том, что фото отправляется на Telegram
    } else {
      bot.sendChatAction(chatID, "upload_photo"); // отправляем Telegram пользователю, который отправил команду на фото, действие в чате о том, что фото отправляется на Telegram
    }

    if (motionDetected) { // если обнаружено движение
      // отправляем фото с подписью Telegram пользователю, который запустил сигнализацию
      bot.sendMultipartFormDataToTelegramWithCaption("sendPhoto", "photo", "img.jpg", "image/jpeg", 
      "Обнаружено движение!", alarmChatID, fbLength, isMoreDataAvailable, getNextByte, nullptr, nullptr);
    } else {
      // отправляем фото Telegram пользователю, который отправил команду на фото
      bot.sendPhotoByBinary(chatID, "image/jpeg", fbLength, isMoreDataAvailable, getNextByte, nullptr, nullptr);
    }
    esp_camera_fb_return(fb); // сохраняем фото в PSRAM
  } else {
    Serial.println("Не удалось получить фото :(");
    if (motionDetected) { // если обнаружено движение
      bot.sendMessage(alarmChatID, "Не удалось получить фото :(", "Markdown");
    } else {
      bot.sendMessage(chatID, "Не удалось получить фото :(", "Markdown");
    }
    motionDetected = false; // сбрасываем то, что обнаружено движение
  }
}

// функция для записи и отправки видео на Telegram


void sendVideoToTelegram() {
  if (motionDetected) { // если обнаружено движение
    // Уведомляем пользователя, что плата ESP32-CAM записывает видео
    bot.sendChatAction(alarmChatID, "record_video"); 
  } else {
    // Уведомляем пользователя, что плата ESP32-CAM записывает видео    
    bot.sendChatAction(videoChatID, "record_video"); 
  }

  // Создаём freeRTOS задачу для записи видео
  xTaskCreatePinnedToCore(
    videoRecordingTask,   // Функция записи видео
    "videoRecordingTask", 
    10000,               
    NULL,                 
    1,                    
    NULL,                 
    1);                   
}



// функция для записи параметров изображения, параметров сигнализации и ускорения видео в SPIFFS память
void saveSettingsToSPIFFS() {
  // записываем все параметры в одну переменную, разделив их с помощью хештега
  settings = framesize + "#" + brightness + "#" + specialEffect + "#" + whiteBalanceMode + "#" + flipImage + "#" + turnOnFlash + "#";
  settings = settings + videoSpeedString + "#" + String(pirSensorDelay) + "#" + alarmMessageType + "#" + alarmRunning;
  File settingsFile = SPIFFS.open("/Settings.txt", FILE_WRITE); // открываем файл с параметрами для записи

  if (settingsFile) { // если файл с параметрами успешно открыт
    Serial.println("Файл с параметрами успешно открыт для записи!");
  } else {
    Serial.println("Не удалось открыть файл с параметрами для записи :(");
    return;
  }
    
  if (settingsFile.print(settings)) { // если параметры успешно записаны в файл
    Serial.println("Файл с параметрами был успешно записан!");
  } else {
    Serial.println("Не удалось записать файл с параметрами :(");
    return;
  }

  settingsFile.close(); // закрываем файл с параметрами
}

// функция для установки разрешения изображения
void setImageFramesize() {
  if (framesize == "XGA") {
    framesizeInt = FRAMESIZE_XGA;
    camera->set_framesize(camera, FRAMESIZE_XGA); // устанавливаем разрешение выше среднего
  } else if (framesize == "SVGA") {
    framesizeInt = FRAMESIZE_SVGA;
    camera->set_framesize(camera, FRAMESIZE_SVGA); // устанавливаем среднее разрешение
  } else if (framesize == "VGA") {
    framesizeInt = FRAMESIZE_VGA;
    camera->set_framesize(camera, FRAMESIZE_VGA); // устанавливаем нормальное разрешение
  } else if (framesize == "CIF") {
    framesizeInt = FRAMESIZE_CIF;
    camera->set_framesize(camera, FRAMESIZE_CIF); // устанавливаем низкое разрешение
  } else if (framesize == "QVGA") {
    framesizeInt = FRAMESIZE_QVGA;
    camera->set_framesize(camera, FRAMESIZE_QVGA); // устанавливаем минимальное разрешение
  }
}

// функция для установки яркости изображения
void setImageBrightness() {
  if (brightness == "200%") {
    camera->set_brightness(camera, 2); // устанавливаем максимальную яркость
  } else if (brightness == "150%") {
    camera->set_brightness(camera, 1); // устанавливаем яркость выше стандартной
  } else if (brightness == "100%") {
    camera->set_brightness(camera, 0); // устанавливаем стандартную яркость
  } else if (brightness == "75%") {
    camera->set_brightness(camera, -1); // устанавливаем яркость ниже стандартной
  } else if (brightness == "50%") {
    camera->set_brightness(camera, -2); // устанавливаем минимальную яркость
  }
}

// функция для установки спец эффекта изображения
void setImageSpecialEffect() {
  if (specialEffect == "Обычный") {
    camera->set_special_effect(camera, 0); // устанавливаем спец эффект "Без эффектов"
  } else if (specialEffect == "Негатив") {
    camera->set_special_effect(camera, 1); // устанавливаем спец эффект "Негатив"
  } else if (specialEffect == "Серый") {
    camera->set_special_effect(camera, 2); // устанавливаем спец эффект "Черно-белый"
  } else if (specialEffect == "Красный") {
    camera->set_special_effect(camera, 3); // устанавливаем спец эффект "Оттенки красного"
  } else if (specialEffect == "Зелёный") {
    camera->set_special_effect(camera, 4); // устанавливаем спец эффект "Оттенки зелёного"
  } else if (specialEffect == "Синий") {
    camera->set_special_effect(camera, 5); // устанавливаем спец эффект "Оттенки синего"
  } else if (specialEffect == "Сепия") {
    camera->set_special_effect(camera, 6); // устанавливаем спец эффект "Сепию"
  }
}

// функция для установки режима баланса белого изображения
void setImageWhiteBalanceMode() {
  if (whiteBalanceMode == "Авто Баланс") {
    camera->set_wb_mode(camera, 0); // устанавливаем режим баланса белого "Авто баланс"
  } else if (whiteBalanceMode == "Солнечный") {
    camera->set_wb_mode(camera, 1); // устанавливаем режим баланса белого "Солнечный"
  } else if (whiteBalanceMode == "Облачный") {
    camera->set_wb_mode(camera, 2); // устанавливаем режим баланса белого "Облачный"
  } else if (whiteBalanceMode == "Офис") {
    camera->set_wb_mode(camera, 3); // устанавливаем режим баланса белого "Офисный"
  } else if (whiteBalanceMode == "Домашний") {
    camera->set_wb_mode(camera, 4); // устанавливаем режим баланса белого "Домашний"
  }
}

// функция для установки того, будет ли переворачиваться изображение
void setImageFlip() {
  if (flipImage == "Да") {
    camera->set_vflip(camera, 1); // устанавливаем то, что изображение будет переворачиваться
  } else {
    camera->set_vflip(camera, 0); // устанавливаем то, что изображение не будет переворачиваться
  }
}

// функция для установки ускорения видео
void setVideoSpeed() {
  if (videoSpeedString == "0.5x") {
    videoSpeedFloat = 0.5; // записываем то, что ускорение видео 50%
    frameInterval = 0; // записываем то, что у видео 25fps
  } else if (videoSpeedString == "1x") {
   videoSpeedFloat = 1; // записываем то, что ускорение видео 100%
   frameInterval = 125; // записываем то, что у видео 8fps
 } else if (videoSpeedString == "2.5x") {
   videoSpeedFloat = 2.5; // записываем то, что ускорение видео 250%
   frameInterval = 250; // записываем то, что у видео 4fps
 } else if (videoSpeedString == "5x") {
   videoSpeedFloat = 5; // записываем то, что ускорение видео 500%
   frameInterval = 500; // записываем то, что у видео 2fps
 }
}

// функция для проверки того, является ли сообщение от Telegram пользователя числом
bool isMessageNumber() {
  for (int i = 0; i < message.length(); i ++) { // цикл для проверки каждого символа в полученном сообщении
    if (isDigit(message.charAt(i))) { // если символ в полученном сообщении является числом
      continue; // продолжаем проверку
    } else {
      return false; // возвращаем false, полученное сообщение не является числом
    }
  }
  return true; // возвращаем true, полученное сообщение является числом
}

// Проверка chat id


bool isChatIDMatches() {
  for (int i = 0; i <= 10; i ++) { 
    // Chat id пользователя найден в списке
    if (chatID == yourChatIDs[i]) { 
      return true; // Сhat id Telegram найден
    } else {
      continue; 
    }
  }
  return false; // Сhat id Telegram не найден
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // отключаем детектор отключения питания
  Serial.begin(115200); // настраиваем скорость СОМ порта

  pinMode(flashPin, OUTPUT); // настраиваем пин, к которому подключена вспышка, как выход
  pinMode(pirSensorPin, INPUT_PULLUP); // настраиваем пин, к которому подключён датчик движения, как вход

  videoBufferSize = 3000 * 1024;   //
  idxBufferSize = 2020;
  psramVideoBuffer = (uint8_t*)ps_malloc(videoBufferSize);
  if (psramVideoBuffer == 0) Serial.println("Не удалось выделить место для видео в PSRAM :(");
  psramIdxBuffer = (uint8_t*)ps_malloc(idxBufferSize);
  if (psramIdxBuffer == 0) Serial.println("Не удалось выделить место для idx в PSRAM :(");

  if (setupCamera()) { // если камера на плате ESP32-CAM успешно настроена и запущена
    Serial.println("Камера успешно запущена!");
  } else {
    Serial.println("Не удалось запустить камеру :(");
    ESP.restart(); // перезагружаем плату ESP32-CAM
  }

  if (SPIFFS.begin(true)) { // если SPIFFS память успешно запущена
    Serial.println("Монтирование SPIFFS прошло успешно!");
  } else {
    Serial.println("Не удалось монтировать SPIFFS :(");
    return;
  }

  if (SPIFFS.exists("/Settings.txt")) { // если файл с параметрами изображения, параметрами сигнализации и ускорением видео существует в SPIFFS памяти
    File settingsFile = SPIFFS.open("/Settings.txt"); // открываем файл с параметрами изображения, параметрами сигнализации и ускорением видео для чтения
    if (settingsFile) { // если файл с параметрами изображения, параметрами сигнализации и ускорением видео успешно открыт для чтения
      Serial.println("Файл с настройками успешно открыт для чтения!");
    } else {
      Serial.println("Не удалось открыть файл с настройками для чтения :(");
      return;
    }

    while (settingsFile.available()) {
      settings = settingsFile.readString(); // получаем содержимое файла
      Serial.println("Содержимое файла: " + String(settings));

      firstHashIndex   = settings.indexOf('#');                      // получаем индекс первого разделительного хештега
      secondHashIndex  = settings.indexOf('#', firstHashIndex + 1);  // получаем индекс второго разделительного хештега
      thirdHashIndex   = settings.indexOf('#', secondHashIndex + 1); // получаем индекс третьего разделительного хештега
      fourthHashIndex  = settings.indexOf('#', thirdHashIndex + 1);  // получаем индекс четвёртого разделительного хештега
      fifthHashIndex   = settings.indexOf('#', fourthHashIndex + 1); // получаем индекс пятого разделительного хештега
      sixthHashIndex   = settings.indexOf('#', fifthHashIndex + 1);  // получаем индекс шестого разделительного хештега
      seventhHashIndex = settings.indexOf('#', sixthHashIndex + 1);  // получаем индекс седьмого разделительного хештега
      eighthHashIndex = settings.indexOf('#', seventhHashIndex + 1); // получаем индекс восьмого разделительного хештега
      ninthHashIndex = settings.indexOf('#', eighthHashIndex + 1);   // получаем индекс девятого разделительного хештега

      framesize = settings.substring(0, firstHashIndex); // выделяем из всех параметров разрешение изображения
      brightness = settings.substring(firstHashIndex + 1, secondHashIndex); // выделяем из всех параметров яркость изображения
      specialEffect = settings.substring(secondHashIndex + 1, thirdHashIndex); // выделяем из всех параметров спец эффект изображения
      whiteBalanceMode = settings.substring(thirdHashIndex + 1, fourthHashIndex); // выделяем из всех параметров режим баланса белого изображения
      flipImage = settings.substring(fourthHashIndex + 1, fifthHashIndex); // выделяем из всех параметров то, будет ли переворачиваться изображение
      turnOnFlash = settings.substring(fifthHashIndex + 1, sixthHashIndex); // выделяем из всех параметров то, будет ли включаться вспышка для фото
      videoSpeedString = settings.substring(sixthHashIndex + 1, seventhHashIndex); // выделяем из всех параметров ускорение видео
      alarmMessageType = settings.substring(eighthHashIndex + 1, ninthHashIndex); // выделяем из всех параметров тип сообщения сигнализации движения
      alarmRunning = settings.substring(ninthHashIndex + 1, settings.length()); // выделяем из всех параметров то, запущена ли сигнализация движения
      pirSensorDelay = (settings.substring(seventhHashIndex + 1, eighthHashIndex)).toInt(); // выделяем из всех параметров задержку PIR датчика
    }

    settingsFile.close(); // закрываем файл с параметрами изображения, параметрами сигнализации и ускорением видео

    setImageFramesize(); // устанавливаем разрешение изображение
    setImageFlip(); // устанавливаем то, будет ли переворачиваться изображение
    setImageBrightness(); // устанавливаем яркость изображения
    setImageSpecialEffect(); // устанавливаем спец эффект изображения
    setImageWhiteBalanceMode(); // устанавливаем режим баланса белого изображения
    setVideoSpeed(); // устанавливаем ускорение видео

    if (alarmRunning == "Да") { // если сигнализация движения запущена
      isAlarmRunning = true; // записываем то, что сигнализация движения запущена
    } 
  }

  Serial.println("Подключаемся к " + String(WiFiSSID));
  WiFi.begin(WiFiSSID, WiFiPASS); // подключаемся к вашей WiFi сети

  if (WiFi.waitForConnectResult() == WL_CONNECTED) { // если плата ESP32-CAM успешно подключилась к вашей WiFi сети
    Serial.print("Подключились к WiFi сети! Локальный IP адрес: "); Serial.println(WiFi.localIP());

    esp_err_t setPowerSave = esp_wifi_set_ps(WIFI_PS_NONE); // отключаем экономию энергии для WiFi
    client.setInsecure(); // отключаем HTTPS сертификаты для подключения к вашему Telegram боту
    bot.longPoll = 3; // устанавливаем как долго Telegram бот будет ждать перед тем как возвращать "сообщения сейчас" в секундах
    bot.setMyCommands(botCommands); // устанавливаем команды Telegram бота

    configTime(timezone*3600, 3600, "pool.ntp.org"); // настраиваем часовой пояс и адрес NTP сервера для получения времени
    Serial.println("Получаем время из Интернета ...");

    while(!time(nullptr)) { // ждём пока время от NTP сервера не будет получено
      Serial.print("*");
      delay(500);
    }
    Serial.println("Получен отклик!");
  } else if (WiFi.waitForConnectResult() == WL_CONNECT_FAILED) { // если плате ESP32-CAM не удалось подключиться к вашей WiFi сети
    Serial.println("Указан неверный пароль WiFi сети :(");
  }
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) { // Проверка подключения к WiFi сети

      do { // делаем
        time_t now = time(nullptr); // Получаем время от NTP сервера
        struct tm* p_tm = localtime(&now); // Записываем время в структуру

        hours = String(p_tm->tm_hour + 1); // Записываем часы
        minutes = String(p_tm->tm_min);    // Записываем минуты
        days = String(p_tm->tm_mday);      // Записываем дни с начала месяца
        month = String(p_tm->tm_mon + 1);  // Записываем номер месяца
        year = p_tm->tm_year + 1900;       // Записываем год
      } while (year == 1900 or year == 1970); // Пока время не будет получено правильно

      if (hours.toInt() < 10) { // Если часы меньше 10
        hours = "0" + hours; // Добавляем ноль к часам
      }
      if (minutes.toInt() < 10) { // Если минуты меньше 10
        minutes = "0" + minutes; // Добавляем ноль к минутам
      }
      if (days.toInt() < 10) { // Если дни с начала месяца меньше 10
        days = "0" + days; // Добавляем ноль к дням с начала месяца
      }
      if (month.toInt() < 10) { // Если номер месяца меньше 10
        month = "0" + month; // Добавляем ноль к номеру месяца
      }

      // Запись названия видео
      videoFileName = hours + "." + minutes + " " + days + "-" + month + "-" + String(year) + ".avi"; 

      Serial.println("Отправляем видео на Telegram ...");
      if (motionDetected) { // если обнаружено движение
        // отправляем Telegram пользователю, который запустил сигнализацию, действие в чате о том, что видео отправляется на Telegram
        bot.sendChatAction(alarmChatID, "upload_video");
        // отправляем файл с видео Telegram пользователю, который запустил сигнализацию
        bot.sendMultipartFormDataToTelegramWithCaption("sendDocument", "document", videoFileName, "image/jpeg",
        "Обнаружено движение!", alarmChatID, psramVideoPtr - psramVideoBuffer, videoMore, videoNext, nullptr, nullptr);
      } else {
        // Отправляем пользователю действие в чате о том, что видео отправляется на Telegram
        bot.sendChatAction(videoChatID, "upload_video");
        // Отправляем файл с видео Telegram пользователю, который отправил команду 
        bot.sendMultipartFormDataToTelegram("sendDocument", "document", videoFileName, "image/jpeg", 
        videoChatID, psramVideoPtr - psramVideoBuffer, videoMore, videoNext, nullptr, nullptr);
      }
      motionDetected = false; // сбрасываем то, что обнаружено движение
    }
    if (isAlarmRunning) { // Если сигнализация движения запущена
      if (millis() - pirSensorDelayMillis > pirSensorDelay * 1000) { // Проверка на обнаружение движения
        pirSensorDelayMillis = millis(); // записываем время проверки того, обнаружено ли движение

        if (digitalRead(pirSensorPin)) { // PIR датчик обнаружил движение
          motionDetected = true; //Обнаружено движение
          sendVideoToTelegram(); // Отправка видео на Telegram
          }
        }
      }
    }

    if (millis() > lastMessageMillis + getMessageInterval) { // Делаем интервал получения сообщений
      int newMessageCount = bot.getUpdates(bot.last_message_received + 1); // Запись количества непрочитанных сообщений
      while (newMessageCount) {
        for (int i = 0; i < newMessageCount; i++) {
          Serial.println("Получен ответ от Telegram пользователя!");
          // Запись chat id Telegram пользователя, который отправил сообщение
          chatID = String(bot.messages[i].chat_id); 

          if (isChatIDMatches()) { // Проверка chat id пользователя 
            message = bot.messages[i].text; // Запись сообщения
            if (message == "/alarm") { 
              isAlarmRunning = !isAlarmRunning; // Изменение состояния сигнализации движения
              if (isAlarmRunning) { // Если сигнализация движения запущена
                alarmRunning = "Да"; // Запись, что сигнализация движения запущена
                alarmChatID = chatID; // Запись chat id Telegram пользователя, который отправил команду
                saveSettingsToSPIFFS(); // Запись параметров в SPIFFS память
                bot.sendMessage(chatID, "Сигнализация запущена!", "Markdown");
              } else {
                alarmRunning = "Нет"; // Запись, что сигнализация движения остановлена
                saveSettingsToSPIFFS(); // Запись параметров в SPIFFS память
                bot.sendMessage(chatID, "Сигнализация остановлена!", "Markdown");
              }
            }
          } else {
            bot.sendMessage(chatID, "Ваш chat id не совпадает!", "Markdown");
          }
        }
        newMessageCount = bot.getUpdates(bot.last_message_received + 1); // обновляем количество непрочитанных сообщений
      }
      lastMessageMillis = millis(); // записываем время получения последнего сообщения от Telegram пользователя
    }
  } else {
    WiFi.reconnect(); // переподключаемся к вашей WiFi сети

    client.setInsecure(); // отключаем HTTPS сертификаты для подключения к вашему Telegram боту
    bot.longPoll = 3; // устанавливаем как долго Telegram бот будет ждать перед тем как возвращать "сообщения сейчас" в секундах
    bot.setMyCommands(botCommands); // устанавливаем команды Telegram бота

    configTime(timezone*3600, 3600, "pool.ntp.org"); // настраиваем часовой пояс и адрес NTP сервера для получения времени
    Serial.println("Получаем время из Интернета ...");

    while(!time(nullptr)) { // ждём пока время от NTP сервера не будет получено
      Serial.print("*");
      delay(500);
    }
    Serial.println("Получен отклик!");
  }
}

bool isMoreDataAvailable() {
  return (fbLength - currentByte);
}

uint8_t getNextByte() {
  currentByte++;
  return (fbBuffer[currentByte - 1]);
}

bool videoMore() {
  return (videoLength - videoPtr);
}

uint8_t videoNext() {
  videoPtr++;
  return (videoBuffer[videoPtr - 1]);
}

struct frameSizeStruct { // структура для хранения размеров разрешений изображения
  uint8_t frameWidth[2];  // ширина изображения с определённым разрешением
  uint8_t frameHeight[2]; // высота изображения с определённым разрешением
};

static const frameSizeStruct frameSizeData[] = { // структура для хранения буфера разрешений изображения
  {{0x40, 0x01}, {0xF0, 0x00}}, // разрешение QVGA, 320x240
  {{0x90, 0x01}, {0x28, 0x01}}, // разрешение CIF,  400x296
  {{0x80, 0x02}, {0xE0, 0x01}}, // разрешение VGA,  640x480
  {{0x20, 0x03}, {0x58, 0x02}}, // разрешение SVGA, 800x600
  {{0x00, 0x04}, {0x00, 0x03}}  // разрешение XGA,  1024x768
};

uint8_t buf[240] = {
  0x52, 0x49, 0x46, 0x46, 0xD8, 0x01, 0x0E, 0x00, 0x41, 0x56, 0x49, 0x20, 0x4C, 0x49, 0x53, 0x54,
  0xD0, 0x00, 0x00, 0x00, 0x68, 0x64, 0x72, 0x6C, 0x61, 0x76, 0x69, 0x68, 0x38, 0x00, 0x00, 0x00,
  0xA0, 0x86, 0x01, 0x00, 0x80, 0x66, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
  0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x02, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x49, 0x53, 0x54, 0x84, 0x00, 0x00, 0x00,
  0x73, 0x74, 0x72, 0x6C, 0x73, 0x74, 0x72, 0x68, 0x30, 0x00, 0x00, 0x00, 0x76, 0x69, 0x64, 0x73,
  0x4D, 0x4A, 0x50, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x74, 0x72, 0x66,
  0x28, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x80, 0x02, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00,
  0x01, 0x00, 0x18, 0x00, 0x4D, 0x4A, 0x50, 0x47, 0x00, 0x84, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x4E, 0x46, 0x4F,
  0x10, 0x00, 0x00, 0x00, 0x6A, 0x61, 0x6D, 0x65, 0x73, 0x7A, 0x61, 0x68, 0x61, 0x72, 0x79, 0x20,
  0x76, 0x38, 0x38, 0x20, 0x4C, 0x49, 0x53, 0x54, 0x00, 0x01, 0x0E, 0x00, 0x6D, 0x6F, 0x76, 0x69,
};

static void inline printQuartet(unsigned long i, uint8_t * quartetFb) {
  uint8_t y[4];
  y[0] = i % 0x100;
  y[1] = (i >> 8) % 0x100;
  y[2] = (i >> 16) % 0x100;
  y[3] = (i >> 24) % 0x100;
  memcpy(quartetFb, y, 4);
}

static void inline printOctet(unsigned long i, unsigned long j, uint8_t * octetFb) {
  uint8_t y[8];
  y[0] = i % 0x100;
  y[1] = (i >> 8) % 0x100;
  y[2] = (i >> 16) % 0x100;
  y[3] = (i >> 24) % 0x100;
  y[4] = j % 0x100;
  y[5] = (j >> 8) % 0x100;
  y[6] = (j >> 16) % 0x100;
  y[7] = (j >> 24) % 0x100;
  memcpy(octetFb, y, 8);
}

// функция freeRTOS задачи для записи видео

void videoRecordingTask(void* taskParameters) {
  videoFb = getGoodPicture(); // Получение первого кадры для видео
  if (!videoFb) { // Не удалось получить кадр
    Serial.println("Не удалось получить кадр для видео!");
    return;
  } else {
    frameCount = 0; // Сброс количества кадров
    videoStartMillis = millis(); // Запись времени начала записи
    Serial.println("Начинаем запись видео!");
    
    nextFb = getGoodPicture(); // Получаем и записываем следующий кадр 
    lastFrameMillis = millis(); // Запись времени последнего кадра
    startVideoRecording(); // Запись видео

    for (int j = 0; j <= 148 ; j++) { // Запись видео пока количество кадров меньше 149
      currentFrameMillis = millis(); // Запись время полумения текущего кадра
      // Если интервал между текущим и предыдущим кадром меньше установленного интервала
      // делаем задержку, равную разнице между интервалом между текущим 
      // и предыдущим кадром и установленным интервалом кадров
      if (currentFrameMillis - lastFrameMillis < frameInterval) { 
        delay(frameInterval - (currentFrameMillis - lastFrameMillis));
      }

      lastFrameMillis = millis(); // Запись времени получения последнего кадра
      frameCount++; // Обновление количества кадров
      // Если получен хотя бы один кадр, сохраняем текущий кадр для видео в PSRAM
      if (frameCount != 1) esp_camera_fb_return(currentFb); 
      currentFb = nextFb; // Сохраняем следующий кадр для видео в текущий
      writeFrameToVideo(currentFb); // Запись текущего кадра
      nextFb = getGoodPicture(); // Получение и запись следующиего кадра

      if (videoSize > videoBufferSize * 0.95) break;
    }

    Serial.println("Завершаем запись видео!");

    esp_camera_fb_return(currentFb); // Сохранение текущего кадра
    frameCount++; // Обновление количества кадров

    currentFb = nextFb; // Сохранение следующего кадра в текущий
    nextFb = NULL; // Сбрасывание следующего кадра
    writeFrameToVideo(currentFb); // Запись текущего кадра
    esp_camera_fb_return(currentFb); // Сохранение текущего кадра

    currentFb = NULL; // Сбрасывание текущего кадра
    endVideoRecording(); // Завершение записи
    videoEndMillis = millis(); // Запись времени завершения записи

    // Запись fps видео
    fps = 1.0 * frameCount / ((videoEndMillis - videoStartMillis) / 1000); 
    frameCount = 0; // Сброс количества кадров
    videoRecorded = true; // Запись видео завершена
    delay(500);
    vTaskDelete(NULL); // Удаление freeRTOS задачи
  }
}

// функция для старта записи видео

static esp_err_t startVideoRecording() {
  psramVideoPtr = 0;
  psramIdxPtr = 0;

  memcpy(buf + 0x40, frameSizeData[framesizeInt].frameWidth,  2);
  memcpy(buf + 0xA8, frameSizeData[framesizeInt].frameWidth,  2);
  memcpy(buf + 0x44, frameSizeData[framesizeInt].frameHeight, 2);
  memcpy(buf + 0xAC, frameSizeData[framesizeInt].frameHeight, 2);

  psramVideoPtr = psramVideoBuffer;
  psramIdxPtr = psramIdxBuffer;

  memcpy(psramVideoPtr, buf, 240);
  psramVideoPtr += 240;
  startMillis = millis(); // записываем время старта записи видео

  jpegSize = 0;
  videoSize = 0;
  videoLength = 0;
  idxOffset = 4;
}

// функция для завершения записи видео
static esp_err_t endVideoRecording() {
  if (frameCount < 5) { // если количество кадров в видео меньше 5
    Serial.println("Не удалось записать видео, так-как получено меньше 5 кадров :(");
  } else {
    videoDuration = millis() - startMillis; // записываем длительность записи видео в миллисекундах

    realFPS = (1000.0f * (float)frameCount) / ((float)videoDuration) * videoSpeedFloat; // записываем реальный fps видео
    microsecondsPerFrame = 1000000.0f / realFPS; // записываем длительность кадра в видео в микросекундах
    attainedFPS = round(realFPS); // округляем и записываем fps видео
    attainedPerFrame = round(microsecondsPerFrame); // округляем и записываем длительность кадра в видео в микросекундах

    printQuartet(videoSize + 240 + frameCount * 16 + 8 * frameCount, psramVideoBuffer + 4);
    printQuartet(attainedPerFrame, psramVideoBuffer + 0x20);

    unsigned long maxBytesPerSecond = (1.0f * videoSize * attainedFPS) / frameCount;
    printQuartet(maxBytesPerSecond, psramVideoBuffer + 0x24);
    printQuartet(frameCount, psramVideoBuffer + 0x30);
    printQuartet(frameCount, psramVideoBuffer + 0x8c);
    printQuartet((int)attainedFPS, psramVideoBuffer + 0x84);
    printQuartet(videoSize + frameCount * 8 + 4, psramVideoBuffer + 0xe8);

    memcpy (psramVideoPtr, idx1Buffer, 4);
    psramVideoPtr += 4;

    printQuartet(frameCount * 16, psramVideoPtr);
    psramVideoPtr += 4;
    psramIdxPtr = psramIdxBuffer;

    for (int i = 0; i < frameCount; i++) {
      memcpy (psramVideoPtr, dcBuffer, 4);
      psramVideoPtr += 4;
      memcpy (psramVideoPtr, zeroBuffer, 4);
      psramVideoPtr += 4;

      memcpy (psramVideoPtr, psramIdxPtr, 8);
      psramVideoPtr += 8;
      psramIdxPtr += 8;
    }
  }
}

// функция для записи кадра в видео
static esp_err_t writeFrameToVideo(camera_fb_t * fb) {
  int fbLength; // переменная для хранения размера кадра в видео
  fbLength = fb->len; // записываем размер кадра в видео
  jpegSize = fbLength; // записываем размер jpeg
  remnant = (4 - (jpegSize & 0x00000003)) & 0x00000003;

  fbBlockMillis = millis(); // записываем время записи буфера кадра в видео
  frameWriteMillis = millis(); // записываем время записи кадра в видео
  memcpy(psramVideoPtr, dcBuffer, 4);
  jpegSizeRemnant = jpegSize + remnant;

  printQuartet(jpegSizeRemnant, psramVideoPtr + 4);
  fbBlockStart = fb->buf; // записываем буфер кадра в видео
  if (fbLength > 8 * 1024 - 8 ) {
    fbBlockLength = 8 * 1024;
    fbLength = fbLength - (8 * 1024 - 8);
    memcpy(psramVideoPtr + 8, fbBlockStart, fbBlockLength - 8);
    fbBlockStart = fbBlockStart + fbBlockLength - 8;
  } else {
    fbBlockLength = fbLength + 8 + remnant;
    memcpy(psramVideoPtr + 8, fbBlockStart, fbBlockLength - 8);
    fbLength = 0; // обнуляем размер кадра в видео
  }

  psramVideoPtr += fbBlockLength;
  while (fbLength > 0) {
    if (fbLength > 8 * 1024) {
      fbBlockLength = 8 * 1024;
      fbLength = fbLength - fbBlockLength;
    } else {
      fbBlockLength = fbLength  + remnant;
      fbLength = 0;
    }

    memcpy(psramVideoPtr, fbBlockStart, fbBlockLength);
    psramVideoPtr += fbBlockLength;
    fbBlockStart = fbBlockStart + fbBlockLength;
  }

  videoSize += jpegSize;
  videoLength += jpegSize;

  printOctet(idxOffset, jpegSize, psramIdxPtr);
  psramIdxPtr += 8;
  idxOffset = idxOffset + jpegSize + remnant + 8;
  videoSize += remnant;
}

// функция для получения хорошего фото
camera_fb_t * getGoodPicture() {
  camera_fb_t * fb; // создаём объект для хранения фото
  do { // делаем
    int fbLength; // переменная для хранения размера фото
    int foundFfd9;
    
    fb = esp_camera_fb_get(); // получаем и записываем фото
    if (!fb) { // если не удалось получить фото 
      Serial.println("Не удалось получить кадр для видео :(");
      getGoodPictureFailures++; // увеличиваем количество неудачных попыток получить хорошее фото
    } else {
      fbLength = fb->len; // записываем размер фото

      for (int j = 1; j <= 1025; j++) {
        if (fb->buf[fbLength - j] != 0xD9) {
        } else {
          if (fb->buf[fbLength - j - 1] == 0xFF ) {
            foundFfd9 = 1;
            break;
          }
        }
      }

      if (!foundFfd9) {
        Serial.printf("Получен плохой кадр, Номер кадра %d, Размер %d байт \n", frameCount, fbLength);
        esp_camera_fb_return(fb);
        getGoodPictureFailures++;
      } else {
        break;
      }
    }
  } while (getGoodPictureFailures < 5); // пока количество неудачных попыток получить хорошее фото меньше 5
  if (getGoodPictureFailures == 5) { // если количество неудачных попыток получить хорошее фото равно 5
    Serial.printf("Количество неудачных попыток получить хорошее фото достигло 5!");
    camera->set_quality(camera, camera->status.quality + 4); // уменьшаем качество изображения
    delay(500);
  }
  return fb; // возвращаем хорошее фото
}
