#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/timekeeping.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <asm/div64.h>
#include "clunet.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cluster");
MODULE_DESCRIPTION("CLUNET driver");

static u8 receive_pin = 2;
static u8 transmit_pin = 3;
static u8 transmit_value = 1;
static u8 clunet_t = 64;
module_param(receive_pin, byte, 0);
MODULE_PARM_DESC(receive_pin,"CLUNET receiver pin number (default 2)");
module_param(transmit_pin, byte, 0);
MODULE_PARM_DESC(transmit_pin,"CLUNET transmitter pin number (default 3)");
module_param(transmit_value, byte, 0);
MODULE_PARM_DESC(transmit_value,"0 = direct send logic, 1 = inverse send logic (default 1)");
module_param(clunet_t, byte, 0);
MODULE_PARM_DESC(clunet_t,"CLUNET bit 0 length (default 64us)");
static unsigned int irqNumber = 0xffff;
static struct class* clunet_class  = NULL;
//static struct device* clunet_device = NULL;
static int clunet_bus_major = 0;
static u64 last_rising_time = 0;
static u64 last_falling_time = 0;
static u8 clunetReadingState = CLUNET_READING_STATE_IDLE;
static u8 clunetReadingCurrentByte;
static u8 clunetReadingCurrentBit;
volatile char dataToRead[CLUNET_READ_BUFFER_SIZE];

static u8 check_crc(const char* data, const uint8_t size)
{
      u8 crc = 0;
      u8 i, j;
      for (i = 0; i < size; i++)
      {
            uint8_t inbyte = data[i];
            for (j = 0 ; j < 8 ; j++)
            {
                  uint8_t mix = (crc ^ inbyte) & 1;
                  crc >>= 1;
                  if (mix) crc ^= 0x8C;
                  inbyte >>= 1;
            }
      }
      return crc;
}

static void clunet_data_received(const uint8_t src_address, const uint8_t dst_address, const uint8_t command, char* data, const uint8_t size)
{
	u8 i;
	char buffer[512];
	buffer[0] = 0;
	if (size > 0) strcat(buffer, ":");
	for (i = 0; i < size && i < 128; i++)
		sprintf(1 + buffer + i*3, " %02X", data[i]);
	printk(KERN_DEBUG "CLUNET received from %02X to %02X cmd %02X size %d%s\n", src_address, dst_address, command, size, buffer);
}

static irq_handler_t clunet_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs)
{
	u64 now = ktime_to_us(ktime_get_boottime());
	u8 value = gpio_get_value(receive_pin);
	if (value && last_falling_time > 0) // high
	{
		/* Линия свободна, пробуем запланировать отправку */
		//if (clunetSendingState == CLUNET_SENDING_STATE_WAITING_LINE)
		//	clunet_start_send();

		uint64_t ticks = now - last_falling_time;	// Вычислим время сигнала

		/* Если кто-то долго жмёт линию (время >= 6.5Т) - это инициализация */
		if (ticks >= (CLUNET_INIT_T + CLUNET_1_T) / 2)
			clunetReadingState = CLUNET_READING_STATE_PRIO1;

		/* Иначе если недолго, то смотрим на этап */
		else
			switch (clunetReadingState)
			{

			/* Чтение данных */
			case CLUNET_READING_STATE_DATA:

				/* Если бит значащий (время > 2Т), то установим его в приемном буфере */
				if (ticks > (CLUNET_0_T + CLUNET_1_T) / 2)
					dataToRead[clunetReadingCurrentByte] |= (1 << clunetReadingCurrentBit);

				/* Инкрементируем указатель бита, и при полной обработке всех 8 бит в байте выполним: */
				if (++clunetReadingCurrentBit & 8)
				{

					/* Проверка на окончание чтения пакета */
					if ((++clunetReadingCurrentByte > CLUNET_OFFSET_SIZE) && (clunetReadingCurrentByte > dataToRead[CLUNET_OFFSET_SIZE] + CLUNET_OFFSET_DATA))
					{

						clunetReadingState = CLUNET_READING_STATE_IDLE;

						/* Проверяем CRC, при успехе начнем обработку принятого пакета */
						if (!check_crc((char*)dataToRead, clunetReadingCurrentByte))
							clunet_data_received (
								dataToRead[CLUNET_OFFSET_SRC_ADDRESS],
								dataToRead[CLUNET_OFFSET_DST_ADDRESS],
								dataToRead[CLUNET_OFFSET_COMMAND],
								(char*)(dataToRead + CLUNET_OFFSET_DATA),
								dataToRead[CLUNET_OFFSET_SIZE]
							);
						else
							printk(KERN_WARNING "CLUNET CRC error: from %02X to %02X cmd %02X size %d\n", 
								dataToRead[CLUNET_OFFSET_SRC_ADDRESS],
								dataToRead[CLUNET_OFFSET_DST_ADDRESS],
								dataToRead[CLUNET_OFFSET_COMMAND],
								dataToRead[CLUNET_OFFSET_SIZE]);
					}
					
					/* Иначе если пакет не прочитан и буфер не закончился - подготовимся к чтению следующего байта */
					else if (clunetReadingCurrentByte < CLUNET_READ_BUFFER_SIZE)
					{
						clunetReadingCurrentBit = 0;
						dataToRead[clunetReadingCurrentByte] = 0;
					}
					
					/* Иначе - нехватка приемного буфера -> игнорируем пакет */
					else
					{
						clunetReadingState = CLUNET_READING_STATE_IDLE;
						printk(KERN_ERR "CLUNET out of revc buffer\n");
					}
				}
				break;

			/* Получение приоритета (младший бит), клиенту он не нужен */
			case CLUNET_READING_STATE_PRIO2:
				clunetReadingCurrentByte = clunetReadingCurrentBit = 0;
				dataToRead[0] = 0;

			/* Получение приоритета (старший бит), клиенту он не нужен */
			case CLUNET_READING_STATE_PRIO1:
				clunetReadingState++;

			}
	} else {
		uint64_t ticks = now - last_rising_time;	// Idle time
		if (clunetReadingState != CLUNET_READING_STATE_IDLE 
			&& ticks >= CLUNET_IDLE_TIMEOUT_T) // Timeout
		{
			clunetReadingState = CLUNET_READING_STATE_IDLE;
			printk(KERN_WARNING "CLUNET recv timeout\n");
		}
	}

	if (value)
		last_rising_time = now;
	else
		last_falling_time = now;
	//printk(KERN_INFO "CLUNET: IRQ! %d\n", gpio_get_value(receive_pin));
    return (irq_handler_t) IRQ_HANDLED;      // Announce that the IRQ has been handled correctly
}

static int __init clunet_init(void)
{
    int r;
	dev_t dev;
    gpio_free(receive_pin);
    r = gpio_request(receive_pin, "clunet_reciver");
    if (r)
    {
        printk(KERN_ERR "CLUNET: receive_pin gpio_request error: %d\n", r);
        clunet_free();
        return -1;
    }
    gpio_direction_input(receive_pin);
    gpio_free(transmit_pin);
    r = gpio_request(transmit_pin, "clunet_transmitter");
    if (r)
    {
        printk(KERN_ERR "CLUNET: transmit_pin gpio_request error: %d\n", r);
        clunet_free();
        return -1;
    }
    gpio_direction_output(transmit_pin, !transmit_value);
    irqNumber = gpio_to_irq(receive_pin);
    r = request_irq(irqNumber,             // The interrupt number requested
                        (irq_handler_t) clunet_irq_handler, // The pointer to the handler function below
                        IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,   // Interrupt on falling edge
                        "clunet_handler",    // Used in /proc/interrupts to identify the owner
                        NULL);                 // The *dev_id for shared interrupt lines, NULL is okay    
    if (r)
    {
        printk(KERN_ERR "CLUNET: receive_pin request_irq error: %d\n", r);
        clunet_free();
        return -1;
    }
    // Register the device class
    clunet_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(clunet_class))
    {
        printk(KERN_ERR "clunet: failed to register device class\n");
        clunet_free();
        return -1;
    }
    r = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME_BUS);
	if (r < 0) {
	    printk(KERN_ERR "clunet: failed to allocate chrdev region\n");
	    return -1;
	}
    clunet_bus_major = MAJOR(dev);
    printk(KERN_INFO "CLUNET: started, major number %d\n", clunet_bus_major);
    return 0;
}

static void clunet_free(void)
{
    if (irqNumber != 0xffff)
        free_irq(irqNumber, NULL);
    gpio_free(receive_pin);
	gpio_free(transmit_pin);
    if (clunet_class)
    {
        class_unregister(clunet_class);                          // unregister the device class
        class_destroy(clunet_class);                             // remove the device class
    }
	if (clunet_bus_major)
		unregister_chrdev_region(MKDEV(clunet_bus_major, 0), 1);
}

static void __exit clunet_exit(void)
{
    clunet_free();
    printk(KERN_INFO "CLUNET: stopped\n");
}

module_init(clunet_init);
module_exit(clunet_exit);
