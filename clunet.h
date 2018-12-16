#ifndef _CLUNET_H_
#define _CLUNET_H_

#define CLASS_NAME  "clunet"
#define DEVICE_NAME_BUS "clunet_bus"
#define DEVICE_NAME_BUS_FILE "clunet"
#define DEVICE_NAME_CUSTOM "clunet_device"

#define CLUNET_READ_BUFFER_SIZE 256
#define CLUNET_SEND_BUFFER_SIZE 256

#define CLUNET_READING !gpio_get_value(receive_pin)

#define CLUNET_T clunet_t
#define CLUNET_0_T (CLUNET_T)
#define CLUNET_1_T (3*CLUNET_T)
#define CLUNET_INIT_T (10*CLUNET_T)
#define CLUNET_IDLE_TIMEOUT_T (10*CLUNET_T)

#define CLUNET_PRIORITY_NOTICE 1
/* Приоритет пакета 1 - неважное уведомление, которое вообще может быть потеряно без последствий */
#define CLUNET_PRIORITY_INFO 2
/* Приоритет пакета 2 - какая-то информация, не очень важная */
#define CLUNET_PRIORITY_MESSAGE 3
/* Приоритет пакета 3 - сообщение с какой-то важной информацией */
#define CLUNET_PRIORITY_COMMAND 4
/* Приоритет пакета 4 - команда, на которую нужно сразу отреагировать */

#define CLUNET_SENDING_STATE_IDLE 0
#define CLUNET_SENDING_STATE_INIT 1
#define CLUNET_SENDING_STATE_PRIO1 2
#define CLUNET_SENDING_STATE_PRIO2 3
#define CLUNET_SENDING_STATE_DATA 4
#define CLUNET_SENDING_STATE_DONE 5
#define CLUNET_SENDING_STATE_WAITING_LINE 8

#define CLUNET_READING_STATE_IDLE 0
#define CLUNET_READING_STATE_INIT 1
#define CLUNET_READING_STATE_PRIO1 2
#define CLUNET_READING_STATE_PRIO2 3
#define CLUNET_READING_STATE_DATA 4

#define CLUNET_OFFSET_SRC_ADDRESS 0
#define CLUNET_OFFSET_DST_ADDRESS 1
#define CLUNET_OFFSET_COMMAND 2
#define CLUNET_OFFSET_SIZE 3
#define CLUNET_OFFSET_DATA 4

#define CLUNET_BROADCAST_ADDRESS 255

#define MAX_OPENED_FILES 64
#define RECEIVER_BUFFER_SIZE 4096
#define TRANSMITTER_BUFFER_SIZE 512

struct cfile_t {
    int id;
    char receiver_buffer[RECEIVER_BUFFER_SIZE];
    u64 receiver_write_pos;
    char transmitter_buffer[TRANSMITTER_BUFFER_SIZE];
    u16 transmitter_write_pos;
    int device_address;
};

static int clunet_init(void);
static void clunet_free(void);

#endif