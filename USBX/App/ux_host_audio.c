/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_host_audio.c
  * @author  MCD Application Team
  * @brief   USBX Host Audio applicative source file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* Includes ------------------------------------------------------------------*/
#include "ux_host_audio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_usbx_host.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  USB_AUDIO_HOST_STATE_IDLE,
  USB_AUDIO_HOST_STATE_STARTING,
  USB_AUDIO_HOST_STATE_RUNNING,
  USB_AUDIO_HOST_STATE_STOPPING,
  USB_AUDIO_HOST_STATE_ERROR,
} usb_audio_host_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define USB_AUDIO_HOST_OUT_THREAD_STACKSIZE   5 * 1024
#define USB_AUDIO_HOST_OUT_THREAD_PRIORITY    7
#define USB_AUDIO_HOST_OUT_THREAD_NAME        "usb_host_audio_out_task"

#define USB_AUDIO_HOST_OUT_SAMPLE_RATE_HZ     (48000)
#define USB_AUDIO_HOST_OUT_NUM_CHANNELS       (2)
#define USB_AUDIO_HOST_OUT_RESOLUTION_BITS    (16)

#define SINE_GENERATOR_AMPLITUDE              (0.5)
#define SINE_GENERATOR_FREQUENCY              (600)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

const SHORT sine_lookup_int16[] = {
  0, 402, 804, 1206, 1607, 2009, 2410, 2811, 3211, 3611,
  4011, 4409, 4807, 5205, 5601, 5997, 6392, 6786, 7179, 7571,
  7961, 8351, 8739, 9126, 9511, 9895, 10278, 10659, 11039, 11416,
  11792, 12167, 12539, 12910, 13278, 13645, 14009, 14372, 14732, 15090,
  15446, 15799, 16151, 16499, 16845, 17189, 17530, 17868, 18204, 18537,
  18867, 19195, 19519, 19841, 20159, 20475, 20787, 21096, 21402, 21705,
  22005, 22301, 22594, 22884, 23170, 23452, 23731, 24007, 24279, 24547,
  24811, 25072, 25329, 25582, 25832, 26077, 26319, 26556, 26790, 27019,
  27245, 27466, 27683, 27896, 28105, 28310, 28510, 28706, 28898, 29085,
  29268, 29447, 29621, 29791, 29956, 30117, 30273, 30424, 30571, 30714,
  30852, 30985, 31113, 31237, 31356, 31470, 31580, 31685, 31785, 31880,
  31971, 32057, 32137, 32213, 32285, 32351, 32412, 32469, 32521, 32567,
  32609, 32646, 32678, 32705, 32728, 32745, 32757, 32765, 32767, 32765,
  32757, 32745, 32728, 32705, 32678, 32646, 32609, 32567, 32521, 32469,
  32412, 32351, 32285, 32213, 32137, 32057, 31971, 31880, 31785, 31685,
  31580, 31470, 31356, 31237, 31113, 30985, 30852, 30714, 30571, 30424,
  30273, 30117, 29956, 29791, 29621, 29447, 29268, 29085, 28898, 28706,
  28510, 28310, 28105, 27896, 27683, 27466, 27245, 27019, 26790, 26556,
  26319, 26077, 25832, 25582, 25329, 25072, 24811, 24547, 24279, 24007,
  23731, 23452, 23170, 22884, 22594, 22301, 22005, 21705, 21402, 21096,
  20787, 20475, 20159, 19841, 19519, 19195, 18867, 18537, 18204, 17868,
  17530, 17189, 16845, 16499, 16151, 15799, 15446, 15090, 14732, 14372,
  14009, 13645, 13278, 12910, 12539, 12167, 11792, 11416, 11039, 10659,
  10278, 9895, 9511, 9126, 8739, 8351, 7961, 7571, 7179, 6786,
  6392, 5997, 5601, 5205, 4807, 4409, 4011, 3611, 3211, 2811,
  2410, 2009, 1607, 1206, 804, 402, 0, -403, -805, -1207,
  -1608, -2010, -2411, -2812, -3212, -3612, -4012, -4410, -4808, -5206,
  -5602, -5998, -6393, -6787, -7180, -7572, -7962, -8352, -8740, -9127,
  -9512, -9896, -10279, -10660, -11040, -11417, -11793, -12168, -12540, -12911,
  -13279, -13646, -14010, -14373, -14733, -15091, -15447, -15800, -16152, -16500,
  -16846, -17190, -17531, -17869, -18205, -18538, -18868, -19196, -19520, -19842,
  -20160, -20476, -20788, -21097, -21403, -21706, -22006, -22302, -22595, -22885,
  -23171, -23453, -23732, -24008, -24280, -24548, -24812, -25073, -25330, -25583,
  -25833, -26078, -26320, -26557, -26791, -27020, -27246, -27467, -27684, -27897,
  -28106, -28311, -28511, -28707, -28899, -29086, -29269, -29448, -29622, -29792,
  -29957, -30118, -30274, -30425, -30572, -30715, -30853, -30986, -31114, -31238,
  -31357, -31471, -31581, -31686, -31786, -31881, -31972, -32058, -32138, -32214,
  -32286, -32352, -32413, -32470, -32522, -32568, -32610, -32647, -32679, -32706,
  -32729, -32746, -32758, -32766, -32768, -32766, -32758, -32746, -32729, -32706,
  -32679, -32647, -32610, -32568, -32522, -32470, -32413, -32352, -32286, -32214,
  -32138, -32058, -31972, -31881, -31786, -31686, -31581, -31471, -31357, -31238,
  -31114, -30986, -30853, -30715, -30572, -30425, -30274, -30118, -29957, -29792,
  -29622, -29448, -29269, -29086, -28899, -28707, -28511, -28311, -28106, -27897,
  -27684, -27467, -27246, -27020, -26791, -26557, -26320, -26078, -25833, -25583,
  -25330, -25073, -24812, -24548, -24280, -24008, -23732, -23453, -23171, -22885,
  -22595, -22302, -22006, -21706, -21403, -21097, -20788, -20476, -20160, -19842,
  -19520, -19196, -18868, -18538, -18205, -17869, -17531, -17190, -16846, -16500,
  -16152, -15800, -15447, -15091, -14733, -14373, -14010, -13646, -13279, -12911,
  -12540, -12168, -11793, -11417, -11040, -10660, -10279, -9896, -9512, -9127,
  -8740, -8352, -7962, -7572, -7180, -6787, -6393, -5998, -5602, -5206,
  -4808, -4410, -4012, -3612, -3212, -2812, -2411, -2010, -1608, -1207,
  -805, -403 };

#define SINE_LOOKUP_INT16_SIZE  (sizeof(sine_lookup_int16) / sizeof(sine_lookup_int16[0]))

struct {
  float phase;
  float amplitude;
  UINT frequency_hz;
  UINT sample_rate_hz;
  UCHAR n_ch;
} sine_generator;

struct {
  usb_audio_host_state_t out_state;
  TX_THREAD out_thread;
  TX_SEMAPHORE out_event_sem;
  UX_HOST_CLASS_AUDIO *out_audio;
  ULONG xfer_count;
} usb_audio_host;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void fill_buf(SHORT *buf, size_t num_samples);
static void usb_audio_host_out_audio_entry(ULONG thread_input);
static void usb_audio_host_out_transfer_request_completion_cb(UX_HOST_CLASS_AUDIO_TRANSFER_REQUEST *transfer_request);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void fill_buf(SHORT *buf, size_t num_samples)
{
  /** Phase offset for each iteration to achieve desired frequency */
  const float delta_phi =  (float) sine_generator.frequency_hz / sine_generator.sample_rate_hz * SINE_LOOKUP_INT16_SIZE;

  /** Fill buffer with sine wave */
  for (size_t i = 0; i < num_samples; i++) {
    size_t phase_i = (size_t) sine_generator.phase;
    for (size_t ch_num = 0; ch_num < sine_generator.n_ch; ch_num++) {
      buf[i + ch_num] = sine_generator.amplitude * sine_lookup_int16[phase_i];
    }

    sine_generator.phase += delta_phi;
    if (sine_generator.phase >= SINE_LOOKUP_INT16_SIZE) {
      sine_generator.phase -= SINE_LOOKUP_INT16_SIZE;
    }
  }
}

static void usb_audio_host_out_audio_entry(ULONG thread_input)
{
  (void) thread_input;

  UINT status;
  ULONG actual_flags;

  UX_HOST_CLASS_AUDIO_SAMPLING audio_sampling;
  UINT subframe_size, frame_size;
  ULONG nominal_packet_size;

  UCHAR *buf1, *buf2, *p_data;

  UX_HOST_CLASS_AUDIO_TRANSFER_REQUEST transfer_request1, transfer_request2;
  UX_HOST_CLASS_AUDIO_TRANSFER_REQUEST *current_transfer_request;

  UINT buf_index;
  

  while (1)
  {
    switch (usb_audio_host.out_state)
    {
    case USB_AUDIO_HOST_STATE_IDLE: {
      /* Wait until device is connected */
      tx_event_flags_get(&usb_host_events, USB_HOST_EVENT_FLAG_CONNECTED, TX_OR, &actual_flags, TX_WAIT_FOREVER);
      usb_audio_host.out_state = USB_AUDIO_HOST_STATE_STARTING;
      break;
    }

    case USB_AUDIO_HOST_STATE_STARTING: {
      /** Allocate memory etc */
      if (usb_audio_host.out_audio == NULL) {
        usb_audio_host.out_state = USB_AUDIO_HOST_STATE_ERROR;
        break;
      }


      /** Set these to static values for now, but would ideally support more devices */
      audio_sampling.ux_host_class_audio_sampling_channels = USB_AUDIO_HOST_OUT_NUM_CHANNELS;
      audio_sampling.ux_host_class_audio_sampling_frequency = USB_AUDIO_HOST_OUT_SAMPLE_RATE_HZ;
      audio_sampling.ux_host_class_audio_sampling_resolution = USB_AUDIO_HOST_OUT_RESOLUTION_BITS;
      
      status = ux_host_class_audio_streaming_sampling_set(usb_audio_host.out_audio, &audio_sampling);
      if (status != UX_SUCCESS) {
        /** Do not support this device, wait until disconnected and retry */
        usb_audio_host.out_state = USB_AUDIO_HOST_STATE_ERROR;
      }

      /** Determine subframe size, framesize and packet size. Assume full-speed frequency. */
      subframe_size = audio_sampling.ux_host_class_audio_sampling_resolution / 8;
      frame_size = subframe_size * audio_sampling.ux_host_class_audio_sampling_channels;
      nominal_packet_size = (audio_sampling.ux_host_class_audio_sampling_frequency / 1000) * frame_size;

      /** Allocate buffer 1 */
      buf1 = ux_utility_memory_allocate(UX_ALIGN_32, UX_CACHE_SAFE_MEMORY, nominal_packet_size);
      if (buf1 == NULL) {
        /** Failed to allocate memory for buffer */
        usb_audio_host.out_state = USB_AUDIO_HOST_STATE_ERROR;
      }

      /** Allocate buffer 2 */
      buf2 = ux_utility_memory_allocate(UX_ALIGN_32, UX_CACHE_SAFE_MEMORY, nominal_packet_size);
      if (buf2 == NULL) {
        /** Failed to allocate memory for buffer */
        usb_audio_host.out_state = USB_AUDIO_HOST_STATE_ERROR;
      }

      /** Setup sine generator for filling buffers with dummy values */
      sine_generator.amplitude = SINE_GENERATOR_AMPLITUDE;
      sine_generator.frequency_hz = SINE_GENERATOR_FREQUENCY;
      sine_generator.sample_rate_hz = audio_sampling.ux_host_class_audio_sampling_frequency;
      sine_generator.n_ch = audio_sampling.ux_host_class_audio_sampling_channels;

      /** Fill buf1 and buf2 with sine wave */
      fill_buf((SHORT*) buf1, nominal_packet_size / subframe_size);
      fill_buf((SHORT*) buf2, nominal_packet_size / subframe_size);

      /** Prepare transfer request 1 */
      transfer_request1.ux_host_class_audio_transfer_request_completion_function = &usb_audio_host_out_transfer_request_completion_cb;
      transfer_request1.ux_host_class_audio_transfer_request_data_pointer = buf1;
      transfer_request1.ux_host_class_audio_transfer_request_class_instance = usb_audio_host.out_audio;
      transfer_request1.ux_host_class_audio_transfer_request_requested_length = nominal_packet_size;
      transfer_request1.ux_host_class_audio_transfer_request_packet_size = nominal_packet_size;
      transfer_request1.ux_host_class_audio_transfer_request_actual_length = nominal_packet_size;

      /** Prepare transfer request 2 */
      transfer_request2.ux_host_class_audio_transfer_request_completion_function = &usb_audio_host_out_transfer_request_completion_cb;
      transfer_request2.ux_host_class_audio_transfer_request_data_pointer = buf2;
      transfer_request2.ux_host_class_audio_transfer_request_class_instance = usb_audio_host.out_audio;
      transfer_request2.ux_host_class_audio_transfer_request_requested_length = nominal_packet_size;
      transfer_request2.ux_host_class_audio_transfer_request_packet_size = nominal_packet_size;
      transfer_request2.ux_host_class_audio_transfer_request_actual_length = nominal_packet_size;

      /** Queue requests */
      status = ux_host_class_audio_write(usb_audio_host.out_audio, &transfer_request1);
      if (status != UX_SUCCESS) {
        usb_audio_host.out_state = USB_AUDIO_HOST_STATE_ERROR;
      }

      status = ux_host_class_audio_write(usb_audio_host.out_audio, &transfer_request2);
      if (status != UX_SUCCESS) {
        usb_audio_host.out_state = USB_AUDIO_HOST_STATE_ERROR;
      }

      buf_index = 0;

      usb_audio_host.out_state = USB_AUDIO_HOST_STATE_RUNNING;

      break;
    }

    case USB_AUDIO_HOST_STATE_RUNNING: {
      /** Read and transmit */
      status = tx_semaphore_get(&usb_audio_host.out_event_sem, TX_WAIT_FOREVER);

      if (usb_audio_host.out_audio == NULL) {
        /** Event sem was given because device has disconnected */
        usb_audio_host.out_state = USB_AUDIO_HOST_STATE_STOPPING;
      }

      /** Determine which transfer just completed and which to send next */
      if (buf_index == 0) {
        current_transfer_request = &transfer_request1;
        buf_index = 1;
      } 
      else {
        current_transfer_request = &transfer_request2;
        buf_index = 0;
      } 

      p_data = current_transfer_request->ux_host_class_audio_transfer_request_data_pointer;

      memset(p_data, 0x00, nominal_packet_size);

      /** Fill data with next bytes */
      fill_buf((SHORT*) p_data, nominal_packet_size / subframe_size);

      /** Start next transfer request */
      current_transfer_request->ux_host_class_audio_transfer_request_completion_function = &usb_audio_host_out_transfer_request_completion_cb;
      current_transfer_request->ux_host_class_audio_transfer_request_requested_length = nominal_packet_size;
      current_transfer_request->ux_host_class_audio_transfer_request_actual_length = nominal_packet_size;
      current_transfer_request->ux_host_class_audio_transfer_request_packet_size = nominal_packet_size;

      ux_host_class_audio_write(usb_audio_host.out_audio, current_transfer_request);
      break;
    }

    case USB_AUDIO_HOST_STATE_STOPPING: {
            
      /** Wait until disconnection then return to idle state */
      tx_event_flags_get(&usb_host_events, USB_HOST_EVENT_FLAG_DISCONNECTED, TX_OR, &actual_flags, TX_WAIT_FOREVER);
      usb_audio_host.out_state = USB_AUDIO_HOST_STATE_IDLE;

      /** Release memory etc. */
      if (transfer_request1.ux_host_class_audio_transfer_request_data_pointer != NULL) {
        ux_utility_memory_free(buf1);
        buf1 = NULL;
      }

      if (transfer_request2.ux_host_class_audio_transfer_request_data_pointer != NULL) {
        ux_utility_memory_free(buf2);
        buf2 = NULL;
      }

      usb_audio_host.out_state = USB_AUDIO_HOST_STATE_IDLE;
      break;
    }

    case USB_AUDIO_HOST_STATE_ERROR: {
      /** Wait until disconnection then return to idle state */
      tx_event_flags_get(&usb_host_events, USB_HOST_EVENT_FLAG_DISCONNECTED, TX_OR, &actual_flags, TX_WAIT_FOREVER);
      usb_audio_host.out_state = USB_AUDIO_HOST_STATE_IDLE;
      break;
    }

    default:
      break;
    }
    /* code */
  }
}

static void usb_audio_host_out_transfer_request_completion_cb(UX_HOST_CLASS_AUDIO_TRANSFER_REQUEST *transfer_request)
{
  (void) transfer_request;

  /* Release the semaphore */
  usb_audio_host.xfer_count += 1;
  tx_semaphore_put(&usb_audio_host.out_event_sem);
}
/* USER CODE END 0 */

/* USER CODE BEGIN 2 */
UINT usb_audio_host_init(VOID *memory_ptr)
{
  UINT status;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;
  UCHAR *pointer;

  /** Allocate thread stack */
  status = tx_byte_allocate(byte_pool, (VOID**) &pointer, USB_AUDIO_HOST_OUT_THREAD_STACKSIZE, TX_NO_WAIT);
  if (status != TX_SUCCESS) {
    return status;
  }

  /** Create out audio thread */
  status = tx_thread_create(&usb_audio_host.out_thread, USB_AUDIO_HOST_OUT_THREAD_NAME, 
      &usb_audio_host_out_audio_entry, 0, pointer, USB_AUDIO_HOST_OUT_THREAD_STACKSIZE, 
      USB_AUDIO_HOST_OUT_THREAD_PRIORITY, USB_AUDIO_HOST_OUT_THREAD_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);

  if (status != TX_SUCCESS) {
    return status;
  }

  status = tx_semaphore_create(&usb_audio_host.out_event_sem, "usb_host_audio_out_event_sem", 0);
  if (status != TX_SUCCESS) {
    return status;
  }

  return UX_SUCCESS;
}

VOID usb_audio_host_inserted(UX_HOST_CLASS_AUDIO *audio)
{
  if (ux_host_class_audio_subclass_get(audio) == UX_HOST_CLASS_AUDIO_SUBCLASS_STREAMING && 
    ux_host_class_audio_type_get(audio) == UX_HOST_CLASS_AUDIO_OUTPUT) {
    /** Save pointer to audio struct */
    usb_audio_host.out_audio = audio;
  }
}

VOID usb_audio_host_removed(UX_HOST_CLASS_AUDIO *audio)
{
  if (audio == usb_audio_host.out_audio) {
    usb_audio_host.out_audio = NULL;
    tx_semaphore_put(&usb_audio_host.out_event_sem);
  }
}
/* USER CODE END 2 */
