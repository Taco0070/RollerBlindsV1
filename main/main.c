#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_task.h"
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "esp_attr.h"
#include "esp_log.h"	
#include "bdc_motor.h"
#include "bdc_motor_interface.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"

#pragma region Definiciones

#pragma region Definicion de gpios
//Salidas de PWM	
#define GPIO_PWM1 GPIO_NUM_18   //Set GPIO 18 as PWM0A
#define GPIO_PWM2 GPIO_NUM_19  //Set GPIO 19 as PWM0B

//Entradas de botones
#define BOTON_SUBIR  GPIO_NUM_13 //Establece dirección del motor hacia arriba
#define BOTON_ENTER GPIO_NUM_27  //Botón de parada y funciones múltiples
#define BOTON_BAJAR  GPIO_NUM_14 //Establece dirección del motor hacia abajo

//Salidas LED
#define LED_AMARILLO  GPIO_NUM_33 //Indica que la persiana está en posición intermedia
#define LED_VERDE  GPIO_NUM_25 //Indica que la persiana está en movimiento
#define LED_ROJO  GPIO_NUM_26 //Indica que la persiana está detenida

//Entradas de encoder
#define ENCODER_GPIO_A GPIO_NUM_32
#define ENCODER_GPIO_B GPIO_NUM_35

#pragma endregion

#pragma region Motor-Macross
//Definiciones variables para el motor
#define MOTOR_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define MOTOR_FREQUENCY_HZ 25000 // 25kHz PWM
#define MOTOR_MAXIMUM_DUTY_CYCLE (MOTOR_TIMER_RESOLUTION_HZ / MOTOR_FREQUENCY_HZ) // 400 ticks
#define MOTOR_GPIO_1 GPIO_PWM1 // GPIO 18
#define MOTOR_GPIO_2 GPIO_PWM2 // GPIO 19
#define MOTOR_DEFAULT_SPEED 100 // 100% duty cycle (velocidad máxima)
#pragma endregion

#pragma region Encoder-Macross
//Minimum and maximum encoder values
//PPR (Pulses Per Revolution) del encoder es 11
#define ENCODER_MIN -2750
#define ENCODER_MAX 2750
#pragma endregion

#pragma region Definicion de variables
//Variables globales
int button_pressed = 0; //Estado del botón. 0 = no presionado, 1 = presionado 
int red_led_status = 0;
int green_led_status = 0;
int yellow_led_status = 0;

int forward_press_counter = 0;
int reverse_press_counter = 0;
int contador_pulso = 0;

//Declaracion del motor que se va a usar
bdc_motor_handle_t motor1 = NULL;

//Declaración de los manejadores de pcnt
pcnt_unit_handle_t pcnt_enconder_unit = NULL;
pcnt_channel_handle_t pcnt_enconder_channel = NULL;
pcnt_channel_handle_t pcnt_enconder_channel2 = NULL;

//Declaración de oneshot timers
esp_timer_handle_t oneshot_long_press_timer;
esp_timer_handle_t oneshot_doublepress_timer;

//Cola de eventos
static QueueHandle_t gpio_evt_queue = NULL;
#pragma endregion

#pragma region Definicion de funciones
//Declaración de funciones
void motor_buttons_intr_disable(void); //Deshabilita las interrupciones de los botones
void motor_buttons_intr_enable(void); //Habilita las interrupciones de los botones
void LED_init(void); //Inicializa los LEDs
void motor_init(void);
void long_press_timer_init(void *args);
void double_press_timer_init(void *args);
//static void read_encoder(void *arg);
//static void button_long_press(void);
#pragma endregion

#pragma endregion

static void oneshot_long_press_callback(void *args)
{
  while(1){
  ESP_LOGE("LONG PRESS", "El botón enter ha sido presionado por mucho tiempo");
  vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

static void oneshot_doublepress_timer_callback(void *args)
{
 if(forward_press_counter >= 2)
  {
    while(1)
    {
      ESP_ERROR_CHECK(bdc_motor_forward(motor1));
      gpio_set_level(LED_AMARILLO, 1);
    
      if((gpio_get_level(BOTON_BAJAR) == 0) | (gpio_get_level(BOTON_SUBIR) == 0) | (gpio_get_level(BOTON_ENTER) == 0))
      {
        ESP_ERROR_CHECK(bdc_motor_brake(motor1));
        break;
      }
      vTaskDelay(10 / portTICK_PERIOD_MS); //intenta evitar que se dispare el task watchdog
    }
  }

  if(reverse_press_counter >= 2)
  {
    while(1)
    {
      ESP_ERROR_CHECK(bdc_motor_reverse(motor1));
      gpio_set_level(LED_ROJO, 1);
    
      if((gpio_get_level(BOTON_BAJAR) == 0) | (gpio_get_level(BOTON_SUBIR) == 0) | (gpio_get_level(BOTON_ENTER) == 0))
      {
        ESP_ERROR_CHECK(bdc_motor_brake(motor1));
        break;
      }
      vTaskDelay(10 / portTICK_PERIOD_MS); //intenta evitar que se dispare el task watchdog
    }
  }

  forward_press_counter = 0;
  reverse_press_counter = 0;
  gpio_set_level(LED_ROJO, 0);
  gpio_set_level(LED_AMARILLO, 0);
  esp_timer_stop(oneshot_doublepress_timer);
  esp_timer_delete(oneshot_doublepress_timer);
  ESP_LOGE("TIMER", "se borro el timer double click");
  ESP_ERROR_CHECK(bdc_motor_brake(motor1));
  
}

static void double_press(void)
{
  
  if (forward_press_counter == 1 || reverse_press_counter == 1)
  {
    double_press_timer_init(&oneshot_doublepress_timer);
  }
  
  while (gpio_get_level(BOTON_SUBIR) == 0)
  {
    // button_pressed = 1;
    // yellow_led_status = button_pressed;
    gpio_set_level(LED_AMARILLO, 1);
    ESP_ERROR_CHECK(bdc_motor_forward(motor1));
    //read_encoder(pcnt_enconder_unit);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  while (gpio_get_level(BOTON_BAJAR) == 0)
  {
    // button_pressed = 1;
    // red_led_status = button_pressed;
    gpio_set_level(LED_ROJO, 1);
    ESP_ERROR_CHECK(bdc_motor_reverse(motor1));
    //read_encoder(pcnt_enconder_unit);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  ESP_ERROR_CHECK(bdc_motor_brake(motor1));
  gpio_set_level(LED_AMARILLO, 0);
  gpio_set_level(LED_ROJO, 0);
}

static void button_long_press(void)
{
  long_press_timer_init(&oneshot_long_press_timer);
  ESP_LOGE("TIMER", "Se ha activado el long press timer");
  
  while(1)
  {
    vTaskDelay(50 / portTICK_PERIOD_MS);
     if (gpio_get_level(BOTON_ENTER) == 0)
        {
          button_pressed = 1;
          green_led_status = button_pressed;
          gpio_set_level(LED_VERDE, green_led_status);
          ESP_LOGE("Boton", "El botón de enter ha sido presionado");
          ESP_ERROR_CHECK(bdc_motor_coast(motor1));

        } else {
          vTaskDelay(50 / portTICK_PERIOD_MS);
          button_pressed = 0;
          green_led_status = 0;
          gpio_set_level(LED_VERDE, green_led_status);
          break;
        }
  }
  esp_timer_stop(oneshot_long_press_timer);
  esp_timer_delete(oneshot_long_press_timer);
  ESP_LOGI("TIMER", "El timer long press ha sido parado y borrado");
 
}

void long_press_timer_init(void *args)
{
    const esp_timer_create_args_t oneshot_long_press_timer_args = 
    {
        .callback = &oneshot_long_press_callback,
        .name = "oneshot_long_press_timer",
    };

    ESP_ERROR_CHECK(esp_timer_create(&oneshot_long_press_timer_args, &oneshot_long_press_timer));
    ESP_LOGW("TIMER", "Se ha creado el timer long press");

    ESP_ERROR_CHECK(esp_timer_start_once(oneshot_long_press_timer, 5000000));
    ESP_LOGW("TIMER", "Se ha iniciado el timer long press");

}

void double_press_timer_init(void *args)
{
  const esp_timer_create_args_t oneshot_presscounter_timer_args = 
    {
        .callback = &oneshot_doublepress_timer_callback,
        .name = "oneshot_doublepress_timer",
    };

    ESP_ERROR_CHECK(esp_timer_create(&oneshot_presscounter_timer_args, &oneshot_doublepress_timer));
    ESP_LOGW("TIMER", "Se ha creado el double click timer");

    ESP_ERROR_CHECK(esp_timer_start_once(oneshot_doublepress_timer, 450000));
    ESP_LOGW("TIMER", "Se ha iniciado el double click timer");
    
}

/*
* @brief Funcion que obtiene el registro del botón que ha sido presionado, ya que, la
interrupción para los botones es compartida
*/
uint64_t getIRQStatus() 
{
  return *(int*)GPIO_STATUS_REG; //Obtiene el registro de estado de los botones
}

static void IRAM_ATTR ButtonISR(void* arg) 
{
  uint32_t boton_gpio_num = getIRQStatus(); //Asigna el registro del boton presionado a boton_gpio_num
  xQueueSendFromISR(gpio_evt_queue, &boton_gpio_num, NULL); //Envía el evento a la cola de eventos
}

void LED_init(void) 
{
  gpio_reset_pin(LED_AMARILLO); //Resetea el pin del LED amarillo
  gpio_set_direction(LED_AMARILLO, GPIO_MODE_OUTPUT); //Establece la dirección del pin del LED amarillo como salida

  gpio_reset_pin(LED_VERDE); //Resetea el pin del LED verde
  gpio_set_direction(LED_VERDE, GPIO_MODE_OUTPUT); //Establece la dirección del pin del LED verde como salida

  gpio_reset_pin(LED_ROJO); //Resetea el pin del LED rojo
  gpio_set_direction(LED_ROJO, GPIO_MODE_OUTPUT); //Establece la dirección del pin del LED rojo como salida
}

void motor_init (void)
{
  ESP_LOGE("Motor", "Creando el motor");
  bdc_motor_config_t motor_config = {
    .pwm_freq_hz = MOTOR_FREQUENCY_HZ,
    .pwma_gpio_num = GPIO_PWM1,
    .pwmb_gpio_num = GPIO_PWM2,
  }; 

   bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = MOTOR_TIMER_RESOLUTION_HZ,
    };    
    
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor1));
    ESP_LOGE("Motor", "Enable motor");
    ESP_ERROR_CHECK(bdc_motor_enable(motor1));
    bdc_motor_set_speed(motor1, MOTOR_DEFAULT_SPEED); // max 400
    ESP_LOGE("Motor", "El motor ha sido inicializado correctamente");
}

static void pcnt_encoder_init(void)
{
  ESP_LOGI("ENCODER", "Inicializando el encoder");
  pcnt_unit_config_t unit_config = {
    .high_limit = ENCODER_MAX,
    .low_limit = ENCODER_MIN,
    .flags.accum_count = 1,
  };

  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_enconder_unit));

  ESP_LOGI("ENCODER", "Estableciendo el filtro de glitch");
  pcnt_glitch_filter_config_t filter_config = {
    .max_glitch_ns = 10000,
  };

  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_enconder_unit, &filter_config));

  ESP_LOGI("ENCODER", "Instalando los canales pcnt");

  pcnt_chan_config_t channel_config = {
    .edge_gpio_num = ENCODER_GPIO_A,
    .level_gpio_num = ENCODER_GPIO_B,
  };

  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_enconder_unit, &channel_config, &pcnt_enconder_channel));

  pcnt_chan_config_t channel2_config = {
    .edge_gpio_num = ENCODER_GPIO_B,
    .level_gpio_num = ENCODER_GPIO_A,
  };

  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_enconder_unit, &channel2_config, &pcnt_enconder_channel2));

  ESP_LOGI("ENCODER", "Estableciendo las acciones del nivel y del borde para los canales pcnt ");
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_enconder_channel, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_enconder_channel, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_enconder_channel2, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_enconder_channel2, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  ESP_LOGI("ENCODER", "Enable pcnt unit");
  ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_enconder_unit));
  ESP_LOGI("ENCODER", "Aclarando la unidad pcnt");
  ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_enconder_unit));
  ESP_LOGI("ENCODER", "El encoder ha sido inicializado");
  ESP_ERROR_CHECK(pcnt_unit_start(pcnt_enconder_unit));

}

#pragma region Funciones de inicializacion y tratamiento botones

static void task_tratamiento_botones(void *arg) 
{
  int boton_gpio_num;

  while(1)
  {
    if (xQueueReceive(gpio_evt_queue, &boton_gpio_num, portMAX_DELAY))
    {
      motor_buttons_intr_disable(); // Deshabilita las interrupciones de los botones
      ESP_LOGI("BOTON", "Se ha deshabilitado las interrupciones de los botones desde el tratamiento");
      //pcnt_unit_enable(pcnt_enconder_unit); // Habilita el encoder
      //pcnt_unit_start(pcnt_enconder_unit); // Inicia el encoder
      //ESP_LOGI("ENCODER", "Se ha habilitado el encoder");
      
      uint64_t boton_subir_mask = (1ULL << BOTON_SUBIR); // Máscara para convertir a binario al botón de subir
      uint64_t boton_enter_mask = (1ULL << BOTON_ENTER); // Máscara para convertir a binario al botón de enter
      uint64_t boton_bajar_mask = (1ULL << BOTON_BAJAR); // Máscara para convertir a binario al botón de bajar

      //pcnt_encoder_init(); //Inicializa el encoder
      //xTaskCreate(read_encoder, "leyendo encoder", 2048, NULL, 10, NULL);

      if(boton_gpio_num & boton_subir_mask)
      {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        forward_press_counter++;
        double_press();
      }
     
      if(boton_gpio_num & boton_enter_mask)
      {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        button_long_press();
      }
      
      if(boton_gpio_num & boton_bajar_mask)
      {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        reverse_press_counter++;
        double_press();
      }
      
      motor_buttons_intr_enable(); // Habilita las interrupciones de los botones
      //pcnt_unit_stop(pcnt_enconder_unit); // Detiene el encoder
      //pcnt_unit_disable(pcnt_enconder_unit); // Deshabilita el encoder
      ESP_LOGI("BOTON", "Se ha deshabilitado las interrupciones de los botones desde el tratamiento");
      ESP_LOGI("ENCODER", "Se ha deshabilitado el encoder");
    }
  }
}

void motor_buttons_intr_disable(void)
{
    gpio_intr_disable(BOTON_SUBIR);
    gpio_intr_disable(BOTON_ENTER);
    gpio_intr_disable(BOTON_BAJAR);
}

void motor_buttons_intr_enable(void)
{
    gpio_intr_enable(BOTON_SUBIR);
    gpio_intr_enable(BOTON_ENTER);
    gpio_intr_enable(BOTON_BAJAR);
}

void motor_buttons_init(void)
{
/*
  gpio_config_t motor_buttons_config = 
  {
    .mode = GPIO_MODE_INPUT,
    .pin_bit_mask = (1ULL << BOTON_SUBIR) |
                    (1ULL << BOTON_ENTER) |
                    (1ULL << BOTON_BAJAR),
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .intr_type = GPIO_INTR_NEGEDGE
  };

*/
    gpio_reset_pin(BOTON_SUBIR);
    gpio_set_direction(BOTON_SUBIR, GPIO_MODE_INPUT);
    gpio_set_intr_type(BOTON_SUBIR, GPIO_INTR_NEGEDGE);

    gpio_reset_pin(BOTON_ENTER);
    gpio_set_direction(BOTON_ENTER, GPIO_MODE_INPUT);
    gpio_set_intr_type(BOTON_ENTER, GPIO_INTR_NEGEDGE);

    gpio_reset_pin(BOTON_BAJAR);
    gpio_set_direction(BOTON_BAJAR, GPIO_MODE_INPUT);
    gpio_set_intr_type(BOTON_BAJAR, GPIO_INTR_NEGEDGE);

}

#pragma endregion

static void read_encoder(void *arg)
{
  while(1)
  {
    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_enconder_unit, &contador_pulso));
    ESP_LOGI("ENCODER", "El valor del encoder es: %d", (contador_pulso));
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}



void app_main(void)
{
  motor_buttons_intr_disable();
  motor_buttons_init(); //Configura a los botones que controlarán al motor estableciendolos como salidas y declarando sus pines
  motor_init(); //Configura al motor con las macross declaradas al inicio
  LED_init(); //Inicializa los LEDs
  pcnt_encoder_init(); //Inicializa el encoder
  
  gpio_evt_queue = xQueueCreate(10, sizeof(uint64_t));

  xTaskCreate(task_tratamiento_botones, "tratando botones", 2048, NULL, 10, NULL);
  xTaskCreate(read_encoder, "leyendo encoder", 2048, NULL, 10, NULL);
  gpio_isr_register(ButtonISR, NULL, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_EDGE, NULL);

  ESP_LOGW("MAIN", "aqui termina la aplicacion");

  motor_buttons_intr_enable();
  
}