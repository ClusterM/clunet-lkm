#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cdev.h>
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
static struct device* clunet_device = NULL;
static struct cdev clunet_bus_cdev;
static int clunet_bus_major = 0;
static int clunet_bus_number_opens = 0;
static struct file* opened_files[MAX_OPENED_FILES];
static DECLARE_WAIT_QUEUE_HEAD(wq_data);

static u64 last_rising_time = 0;
static u64 last_falling_time = 0;
static u8 clunetReadingState = CLUNET_READING_STATE_IDLE;
static u8 clunetReadingCurrentByte = 0;
static u8 clunetReadingCurrentBit = 0;
static u8 dataToRead[CLUNET_READ_BUFFER_SIZE];
static u8 recvPriority = 0;

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

static void clunet_data_received(const uint8_t prio, const uint8_t src_address, const uint8_t dst_address, const uint8_t command, char* data, const uint8_t size)
{
    int i;
    char* buffer;
	char* b;
    buffer = kmalloc(2 + 3 + 3 + 3 + size * 2 + 1 + 1, GFP_ATOMIC); // 'P SR DS CM DATA\n0'
	if (!buffer)
	{
	    printk(KERN_ERR "CLUNET: can't allocatate memory");
		return;
	}
	sprintf(buffer, "%d %02X %02X %02X ", prio, src_address, dst_address, command);
    for (i = 0; i < size; i++)
        sprintf(buffer + 2 + 3 + 3 + 3 + i*2, "%02X", data[i]);
    strcat(buffer, "\n");
    printk(KERN_DEBUG "CLUNET received: %s", buffer);
    b = buffer;
    while (*b)
    {
        for (i = 0; i < clunet_bus_number_opens; i++)
        {
            ((struct cfile_t*)opened_files[i]->private_data)->receiver_buffer[
                ((struct cfile_t*)opened_files[i]->private_data)->receiver_write_pos++
			    % RECEIVER_BUFFER_SIZE] = *b;
        }
		b++;
    };
	kfree(buffer);
	wake_up_interruptible(&wq_data);
}

static irq_handler_t clunet_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs)
{
    u64 now = ktime_to_us(ktime_get_boottime());
    u8 value = gpio_get_value(receive_pin);
    if (value && last_falling_time > 0) // high
    {
        /* Линия свободна, пробуем запланировать отправку */
        //if (clunetSendingState == CLUNET_SENDING_STATE_WAITING_LINE)
        //    clunet_start_send();

        uint64_t ticks = now - last_falling_time;    // Вычислим время сигнала

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
                        if (!check_crc((char*)dataToRead + CLUNET_OFFSET_SRC_ADDRESS, clunetReadingCurrentByte - CLUNET_OFFSET_SRC_ADDRESS))
                            clunet_data_received (
                                recvPriority,
                                dataToRead[CLUNET_OFFSET_SRC_ADDRESS],
                                dataToRead[CLUNET_OFFSET_DST_ADDRESS],
                                dataToRead[CLUNET_OFFSET_COMMAND],
                                (char*)(dataToRead + CLUNET_OFFSET_DATA),
                                dataToRead[CLUNET_OFFSET_SIZE]
                            );
                        else
                            printk(KERN_WARNING "CLUNET CRC error: prio %d from %02X to %02X cmd %02X size %d\n", 
                                recvPriority,
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
                /* Если бит значащий (время > 2Т), то установим его в приемном буфере */
                clunetReadingState++;
                if (ticks > (CLUNET_0_T + CLUNET_1_T) / 2)
                    recvPriority |= 1;
                recvPriority++;
                clunetReadingCurrentByte = CLUNET_OFFSET_SRC_ADDRESS;
                clunetReadingCurrentBit = 0;
                dataToRead[CLUNET_OFFSET_SRC_ADDRESS] = 0;
                break;

            /* Получение приоритета (старший бит), клиенту он не нужен */
            case CLUNET_READING_STATE_PRIO1:
                clunetReadingState++;
                /* Если бит значащий (время > 2Т), то установим его в приемном буфере */
                if (ticks > (CLUNET_0_T + CLUNET_1_T) / 2)
                    recvPriority = 1 << 1;
                else
                    recvPriority = 0;
            }
    } else {
        uint64_t ticks = now - last_rising_time;    // Idle time
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

/** @brief The device open function that is called each time the device is opened
 *  This will only increment the clunet_bus_number_opens counter in this case.
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int clunet_dev_open(struct inode *inodep, struct file *filep)
{
    if (clunet_bus_number_opens >= MAX_OPENED_FILES)
        return -EMFILE;
    filep->private_data = kmalloc(sizeof(struct cfile_t), GFP_KERNEL);
    if (!filep->private_data)
        return -ENOMEM;
    memset(filep->private_data, 0, sizeof(struct cfile_t));
    ((struct cfile_t*)filep->private_data)->id = clunet_bus_number_opens;
    filep->f_pos = 0;
    opened_files[clunet_bus_number_opens] = filep;
    clunet_bus_number_opens++;
    return 0;
}

/** @brief This function is called whenever device is being read from user space i.e. data is
 *  being sent from the device to the user. In this case is uses the copy_to_user() function to
 *  send the buffer string to the user and captures any errors.
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 *  @param buffer The pointer to the buffer to which this function writes the data
 *  @param len The length of the b
 *  @param offset The offset if required
 */
static ssize_t clunet_dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
    ssize_t r;
    r = 0;
    while (len)
    {
        if (((struct cfile_t*)filep->private_data)->receiver_write_pos - (*offset) > RECEIVER_BUFFER_SIZE)
            return -ENOTRECOVERABLE;
        if (*offset == ((struct cfile_t*)filep->private_data)->receiver_write_pos)
        {
            if (r) return r;
            if (filep->f_flags & O_NONBLOCK)
                return -EAGAIN;
            if (wait_event_interruptible(wq_data, (*offset != ((struct cfile_t*)filep->private_data)->receiver_write_pos)))
                return -ERESTARTSYS;
        }
        if (*offset == ((struct cfile_t*)filep->private_data)->receiver_write_pos)
            break;
        if (put_user(((struct cfile_t*)filep->private_data)->receiver_buffer[*offset % RECEIVER_BUFFER_SIZE], buffer))
            return -EFAULT;
        buffer++;
        r++;
		(*offset)++;
        len--;
    }
    return r;
}
 
/** @brief This function is called whenever the device is being written to from user space i.e.
 *  data is sent to the device from the user. The data is copied to the message[] array in this
 *  LKM using the sprintf() function along with the length of the string.
 *  @param filep A pointer to a file object
 *  @param buffer The buffer to that contains the string to write to the device
 *  @param len The length of the array of data that is being passed in the const char buffer
 *  @param offset The offset if required
 */
static ssize_t clunet_dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
    return 0;
}

/** @brief The device release function that is called whenever the device is closed/released by
 *  the userspace program
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int clunet_dev_release(struct inode *inodep, struct file *filep)
{
    int id;
    clunet_bus_number_opens--;
    id = ((struct cfile_t*)filep->private_data)->id;
    opened_files[id] = opened_files[clunet_bus_number_opens];
    ((struct cfile_t*)opened_files[id]->private_data)->id = id;
    kfree(filep->private_data);
    return 0;
}

static struct file_operations fops =
{
   .open = clunet_dev_open,
   .read = clunet_dev_read,
   .write = clunet_dev_write,
   .release = clunet_dev_release,
};

static int __init clunet_init(void)
{
    int r;
    dev_t dev;
    memset(opened_files, 0, sizeof(opened_files));
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
    if (clunet_class == NULL)
    {
        printk(KERN_ERR "CLUNET: failed to register device class\n");
        clunet_free();
        return -1;
    }
    r = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME_BUS);
    if (r)
    {
        printk(KERN_ERR "CLUNET: failed to allocate chrdev region: %d\n", r);
        clunet_free();
        return -1;
    }
    clunet_bus_major = MAJOR(dev);
    clunet_device = device_create(clunet_class, NULL, dev, NULL, DEVICE_NAME_BUS);
    if (clunet_device == NULL)
    {
        printk(KERN_ERR "CLUNET: failed to create /dev/%s\n", DEVICE_NAME_BUS);
        clunet_free();
        return -1;
    }
    cdev_init(&clunet_bus_cdev, &fops);
    clunet_bus_cdev.owner = THIS_MODULE;
    clunet_bus_cdev.ops = &fops;
    r = cdev_add(&clunet_bus_cdev, dev, 1);
    if (r)
    {
        printk(KERN_ERR "CLUNET: failed to add chrdev: %d\n", r);
        clunet_free();
        return -1;
    }
    printk(KERN_INFO "CLUNET: started, major number %d\n", clunet_bus_major);
    return 0;
}

static void clunet_free(void)
{
    if (clunet_class && clunet_bus_major)
    {
        cdev_del(&clunet_bus_cdev);
        device_destroy(clunet_class, MKDEV(clunet_bus_major, 0));     // remove the device
    }
    if (clunet_bus_major)
    {
        unregister_chrdev_region(MKDEV(clunet_bus_major, 0), 1);
    }
    if (clunet_class)
    {
        class_unregister(clunet_class);                          // unregister the device class
        class_destroy(clunet_class);                             // remove the device class
    }
    if (irqNumber != 0xffff)
        free_irq(irqNumber, NULL);
    gpio_free(receive_pin);
    gpio_free(transmit_pin);
}

static void __exit clunet_exit(void)
{
    clunet_free();
    printk(KERN_INFO "CLUNET: stopped\n");
}

module_init(clunet_init);
module_exit(clunet_exit);
