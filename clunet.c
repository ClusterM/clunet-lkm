#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/timekeeping.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include "clunet.h"

/* Module parameters */
static u8 receive_pin = 2;
static u8 transmit_pin = 3;
static u8 transmit_value = 1;
static u8 clunet_t = 64;
static u8 address = 0;
static char* device_name = "Linux CLUNET driver";
module_param(receive_pin, byte, S_IRUGO);
MODULE_PARM_DESC(receive_pin,"CLUNET receiver pin number (default 2)");
module_param(transmit_pin, byte, S_IRUGO);
MODULE_PARM_DESC(transmit_pin,"CLUNET transmitter pin number (default 3)");
module_param(transmit_value, byte, S_IRUGO);
MODULE_PARM_DESC(transmit_value,"0 = direct send logic, 1 = inverse send logic (default 1)");
module_param(clunet_t, byte, S_IRUGO);
MODULE_PARM_DESC(clunet_t,"CLUNET bit 0 length (default 64us)");
module_param(address, byte, S_IRUGO);
MODULE_PARM_DESC(address,"Local address (default 0)");
module_param(device_name, charp, S_IRUGO);
MODULE_PARM_DESC(device_name,"Local device name");

static struct gpio_desc* receive_pin_desc = NULL;
static struct gpio_desc* transmit_pin_desc = NULL;
static unsigned int irq_number = 0xffff;
static struct class* clunet_class  = NULL;
static struct device* clunet_bus_device = NULL;
static struct cdev clunet_bus_cdev;
static struct cdev clunet_device_cdev;
static int clunet_bus_major = 0;
static int clunet_device_major = 0;
static int clunet_bus_number_opens = 0;
static struct file* opened_files[MAX_OPENED_FILES];
static DECLARE_WAIT_QUEUE_HEAD(wq_data);
static DEFINE_MUTEX(m_wait_for_line);
static struct hrtimer* data_send_timer = NULL;
static u64 last_rising_time = 0;
static u64 last_falling_time = 0;
static u8 clunet_sending = 0;
static u8 clunet_sending_state = CLUNET_SENDING_STATE_IDLE;
static u8 clunet_sending_priority;
static u8 clunet_sending_data_length;
static u8 clunet_sending_current_byte;
static u8 clunet_sending_current_bit;
static u8 out_buffer[CLUNET_SEND_BUFFER_SIZE];
static u8 clunet_reading_state = CLUNET_READING_STATE_IDLE;
static u8 clunet_reading_priority = 0;
static u8 clunet_reading_current_byte = 0;
static u8 clunet_reading_current_bit = 0;
static u8 in_buffer[CLUNET_READ_BUFFER_SIZE];

/* CRC check function */
static u8 check_crc(const char* data, const uint8_t size)
{
      u8 crc = 0;
      u8 i, j;
      for (i = 0; i < size; i++) {
            uint8_t inbyte = data[i];
            for (j = 0 ; j < 8 ; j++) {
                  uint8_t mix = (crc ^ inbyte) & 1;
                  crc >>= 1;
                  if (mix) crc ^= 0x8C;
                  inbyte >>= 1;
            }
      }
      return crc;
}

/* Function to schedule main timer */
static void set_send_timer(u16 t)
{
    if (data_send_timer) {
        hrtimer_try_to_cancel(data_send_timer);
        if (t)
            hrtimer_start(data_send_timer, ktime_set(0, t * 1000UL),
                HRTIMER_MODE_REL);
    }
}

/* Function to outpud data  */
static inline void clunet_set_line(u8 value)
{
    clunet_sending = value;
    gpiod_direction_output(transmit_pin_desc,
        value ? transmit_value : !transmit_value);
}

/* Callback for received data */
static void clunet_data_received(
    const uint8_t prio,
    const uint8_t src_address,
    const uint8_t dst_address,
    const uint8_t command,
    char* data,
    const uint8_t size)
{
    int i, p;
    char* buffer;
    int d_address;
    /* 'P SR DS CM DATA\n0' */
    buffer = kmalloc(2 + 3 + 3 + 3 + size * 2 + 1 + 1, GFP_ATOMIC);
    if (!buffer) {
        pr_err("CLUNET: can't allocatate memory");
        return;
    }
    sprintf(buffer, "%d %02X %02X %02X ",
        prio, src_address, dst_address, command);
    for (i = 0; i < size; i++)
        sprintf(buffer + 2 + 3 + 3 + 3 + i*2, "%02X", data[i]);
    strcat(buffer, "\n");
    pr_debug("CLUNET received: %s", buffer);
    p = 0;
    while (buffer[p]) {
        for (i = 0; i < clunet_bus_number_opens; i++) {
            if (!(opened_files[i]->f_mode & FMODE_READ)) continue;
            d_address = ((struct cfile_t*)opened_files[i]->private_data)->device_address;
            if ((d_address < 0) /* Bus devices */
                /* Source address matched, */
                || ((src_address == d_address)
                    /* destination address is my address, */
                    && ((dst_address == address)
                        /* or broadcast address */
                        || (dst_address == CLUNET_BROADCAST_ADDRESS))
                    && (p >= 2 + 3 + 3)) /* skip 'P SR DS ' */
            )
            ((struct cfile_t*)opened_files[i]->private_data)->receiver_buffer[
                ((struct cfile_t*)opened_files[i]->private_data)->receiver_write_pos++
                % RECEIVER_BUFFER_SIZE] = buffer[p];
        }
        p++;
    };
    kfree(buffer);

    /* Some reserved commands */
    if (((dst_address == address) /* Destination address is my address */
        /* or broadcast address */
        || (dst_address == CLUNET_BROADCAST_ADDRESS))
        /* Not busy */
        && ((clunet_sending_state == CLUNET_SENDING_STATE_IDLE)
        /* Or not very busy */
        || (clunet_sending_priority <= CLUNET_PRIORITY_MESSAGE)) 
       ) {
        /* Device discovery reply */
        if (command == CLUNET_COMMAND_DISCOVERY) {
            uint8_t len = 0; while(device_name[len]) len++;
            clunet_send(address, src_address, CLUNET_PRIORITY_MESSAGE,
                CLUNET_COMMAND_DISCOVERY_RESPONSE, device_name, len);
        }
        /* PING reply */
        else if (command == CLUNET_COMMAND_PING) {
            clunet_send(address, src_address, CLUNET_PRIORITY_COMMAND,
                CLUNET_COMMAND_PING_REPLY, data, size);
        }
    }

    wake_up_interruptible(&wq_data);
}

/* IRQ fired every rising/falling edge of receiver pin */
static irq_handler_t clunet_irq_handler(unsigned int irq,
    void *dev_id, struct pt_regs *regs)
{
    u64 now = ktime_to_us(ktime_get_boottime());
    u8 value = CLUNET_READING;
    uint64_t ticks;
    if (value && last_rising_time > 0) { /* Line is low */
        ticks = now - last_rising_time; /* Idle time */
        if (clunet_reading_state != CLUNET_READING_STATE_IDLE 
            && ticks >= CLUNET_IDLE_TIMEOUT_T
           ) { /* Timeout */
            clunet_reading_state = CLUNET_READING_STATE_IDLE;
            pr_warning("CLUNET recv timeout\n");
        }
        if (clunet_sending_state && !clunet_sending) { /* Collision */
            /* Stop transmission and wait for line */
            set_send_timer(0);
            clunet_sending_state = CLUNET_SENDING_STATE_WAITING_LINE;
        }
    }
    else if (!value && last_falling_time > 0) { /* Line is high */    
        /* Line is free trying to schedule transmission */
        if (clunet_sending_state == CLUNET_SENDING_STATE_WAITING_LINE) {
            clunet_sending_state = CLUNET_SENDING_STATE_INIT;
            set_send_timer(CLUNET_SEND_PAUSE_T);
        }

        ticks = now - last_falling_time; /* Signal time calculated */

        /* Is it initialization? (time >= 6.5T?) */
        if (ticks >= (CLUNET_INIT_T + CLUNET_1_T) / 2)
            clunet_reading_state = CLUNET_READING_STATE_PRIO1;
        /* Need to check stage otherwise */
        else
            switch (clunet_reading_state) {

            /* Reading data */
            case CLUNET_READING_STATE_DATA:
                /* If time > 2T, need to set bit */
                if (ticks > (CLUNET_0_T + CLUNET_1_T) / 2)
                    in_buffer[clunet_reading_current_byte] 
                        |= (1 << clunet_reading_current_bit);
                /* Next bit, end of byte check */
                if (++clunet_reading_current_bit & 8)
                {
                    /* Is it end of the frame? */
                    if ((++clunet_reading_current_byte > CLUNET_OFFSET_SIZE)
                        && (clunet_reading_current_byte > in_buffer[CLUNET_OFFSET_SIZE]
                        + CLUNET_OFFSET_DATA)) {
                        clunet_reading_state = CLUNET_READING_STATE_IDLE;
                        /* Lets check CRC */
                        if (!check_crc((char*)in_buffer 
                            + CLUNET_OFFSET_SRC_ADDRESS,
                            clunet_reading_current_byte 
                            - CLUNET_OFFSET_SRC_ADDRESS)) {
                            /* CRC is OK, lets proceed data */
                            clunet_data_received(
                                clunet_reading_priority,
                                in_buffer[CLUNET_OFFSET_SRC_ADDRESS],
                                in_buffer[CLUNET_OFFSET_DST_ADDRESS],
                                in_buffer[CLUNET_OFFSET_COMMAND],
                                (char*)(in_buffer + CLUNET_OFFSET_DATA),
                                in_buffer[CLUNET_OFFSET_SIZE]
                            );
                        } else { /* CRC check */
                            /* CRC error, lets print error to kernel log */
                            pr_warning("CLUNET CRC error: prio %d from %02X to %02X cmd %02X size %d\n",
                                clunet_reading_priority,
                                in_buffer[CLUNET_OFFSET_SRC_ADDRESS],
                                in_buffer[CLUNET_OFFSET_DST_ADDRESS],
                                in_buffer[CLUNET_OFFSET_COMMAND],
                                in_buffer[CLUNET_OFFSET_SIZE]);
                        }
                    }
                    /* This is not end of the packet, reading next byte */
                    else if (clunet_reading_current_byte 
                        < CLUNET_READ_BUFFER_SIZE) {
                        clunet_reading_current_bit = 0;
                        in_buffer[clunet_reading_current_byte] = 0;
                    }
                    /* Buffer overflow, discaring data */
                    else {
                        clunet_reading_state = CLUNET_READING_STATE_IDLE;
                        pr_err("CLUNET out of revc buffer\n");
                    }
                }
                break;

            /* Reading priority (lower bit) */
            case CLUNET_READING_STATE_PRIO2:
                clunet_reading_state++;                
                if (ticks > (CLUNET_0_T + CLUNET_1_T) / 2)
                    /* Logic 1 */
                    clunet_reading_priority |= 1;
                clunet_reading_priority++;
                clunet_reading_current_byte = CLUNET_OFFSET_SRC_ADDRESS;
                clunet_reading_current_bit = 0;
                in_buffer[CLUNET_OFFSET_SRC_ADDRESS] = 0;
                break;

            /* Reading priority (higher bit) */
            case CLUNET_READING_STATE_PRIO1:
                clunet_reading_state++;
                if (ticks > (CLUNET_0_T + CLUNET_1_T) / 2)
                    /* Login 1 */
                    clunet_reading_priority = 1 << 1;
                else
                    /* Logic 0 */
                    clunet_reading_priority = 0;
            }
    }

    /* Save current timestamp */
    if (!value)
        last_rising_time = now;
    else
        last_falling_time = now;
    /* Announce that the IRQ has been handled correctly */
    return (irq_handler_t) IRQ_HANDLED;
}

/* Timer */
static enum hrtimer_restart send_timer_callback(struct hrtimer *timer)
{
    /* If data sending is done, free transmitter */
    if (!clunet_sending_state
        || clunet_sending_state == CLUNET_SENDING_STATE_DONE) {
        /* Free transmitter */
        clunet_sending_state = CLUNET_SENDING_STATE_IDLE;
        /* Release line */
        clunet_set_line(0);
        /* Wake up, i'm ready for new data! */
        wake_up_interruptible(&wq_data);
    } else if (!clunet_sending && CLUNET_READING) {
        /* If transmitting data and line is already busy */
        /* Waiting for line */
        clunet_sending_state = CLUNET_SENDING_STATE_WAITING_LINE;
    } else {
        /* Continue transmission */
        clunet_set_line(!clunet_sending); /* Invert line value */

        if (!clunet_sending) /* If line is released */
        {
            /* Scedule timer for T */
            set_send_timer(CLUNET_T);
        } else if (clunet_sending_state == CLUNET_SENDING_STATE_DATA) {
            /* Transferring data otherwise */
            /* Timer value depends on next bit value */
            set_send_timer((out_buffer[clunet_sending_current_byte] &
                (1 << clunet_sending_current_bit)) ? CLUNET_1_T : CLUNET_0_T);
            /* Every 8 bits... */
            if (++clunet_sending_current_bit & 8)
            {
                /* If there are not more data */
                if (++clunet_sending_current_byte 
                    < clunet_sending_data_length)
                    /* Next byte */
                    clunet_sending_current_bit = 0;
                /* End of transmission */
                else
                    /* Next stage */
                    clunet_sending_state++;
            }
        } else { /* Other stages */
            switch (clunet_sending_state++)
            {
            /* Init (holding line for 10Т) */
            case CLUNET_SENDING_STATE_INIT:
                set_send_timer(CLUNET_INIT_T);
                break;
            /* Priority (higher bit) */
            case CLUNET_SENDING_STATE_PRIO1:
                set_send_timer((clunet_sending_priority > 2)
                    ? CLUNET_1_T
                    : CLUNET_0_T);
                break;
            /* Priority (lower bit) */
            case CLUNET_SENDING_STATE_PRIO2:
                set_send_timer((clunet_sending_priority & 1) 
                    ? CLUNET_0_T
                    : CLUNET_1_T);
                /* Reset counters */
                clunet_sending_current_byte
                    = clunet_sending_current_bit = 0;
                break;
            default:
                /* Something weird happen */
                clunet_set_line(0);
                pr_err("CLUNET unknown sending state: %d\n",
                    clunet_sending_state);
            }
        }
    }

    return HRTIMER_NORESTART;
}

/* Function to schedule frame transmission */
static void clunet_send(const uint8_t src_address,
    const uint8_t dst_address, const uint8_t prio,
    const uint8_t command,
    const char* data,
    const uint8_t size)
{
    u8 i;
    /* Check data size */
    if (size >= (CLUNET_SEND_BUFFER_SIZE - CLUNET_OFFSET_DATA))
        return;    

    /* Abort current transmission if any */
    if (clunet_sending_state) {
        if (data_send_timer)
            hrtimer_try_to_cancel(data_send_timer);
        clunet_set_line(0);
    }

    /* Fill the data */
    clunet_sending_priority = (prio > 4) ? 4 : prio ? : 1;
    out_buffer[CLUNET_OFFSET_SRC_ADDRESS] = src_address;
    out_buffer[CLUNET_OFFSET_DST_ADDRESS] = dst_address;
    out_buffer[CLUNET_OFFSET_COMMAND] = command;
    out_buffer[CLUNET_OFFSET_SIZE] = size;
    for (i = 0; i < size; i++)
        out_buffer[CLUNET_OFFSET_DATA + i] = data[i];
    /* Calculate checksum */
    out_buffer[CLUNET_OFFSET_DATA + size]
        = check_crc((char*)out_buffer, CLUNET_OFFSET_DATA + size);
    clunet_sending_data_length = size + (CLUNET_OFFSET_DATA + 1);

    /* Start transfer immidiatly if line is free */
    if (!CLUNET_READING) {
        clunet_sending_state = CLUNET_SENDING_STATE_INIT;
        set_send_timer(CLUNET_SEND_PAUSE_T);
    } else { /* Schedule it otherwise */
        clunet_sending_state = CLUNET_SENDING_STATE_WAITING_LINE;
    }
}

/* The device open function that is called each time the device is opened */
static int clunet_dev_open(struct inode *inodep, struct file *filep)
{
    int d_address = -1;
    if (clunet_bus_number_opens >= MAX_OPENED_FILES)
        return -EMFILE;
    filep->private_data = kzalloc(sizeof(struct cfile_t), GFP_KERNEL);
    if (!filep->private_data)
        return -ENOMEM;
    ((struct cfile_t*)filep->private_data)->id = clunet_bus_number_opens;
    if (imajor(inodep) == clunet_device_major)
        d_address = iminor(inodep);
    ((struct cfile_t*)filep->private_data)->device_address = d_address;
    filep->f_pos = 0;
    opened_files[clunet_bus_number_opens] = filep;
    clunet_bus_number_opens++;
    return 0;
}

/* This function is called whenever device is being read from user space i.e. data is
 * being sent from the device to the user. In this case is uses the copy_to_user() function to
 * send the buffer string to the user and captures any errors.
 */
static ssize_t clunet_dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
    ssize_t r;
    r = 0;
    while (len) {
        if (((struct cfile_t*)filep->private_data)->receiver_write_pos - (*offset) > RECEIVER_BUFFER_SIZE)
            return -ENOTRECOVERABLE;
        if (*offset == ((struct cfile_t*)filep->private_data)->receiver_write_pos) {
            if (r) return r;
            if (filep->f_flags & O_NONBLOCK)
                return -EAGAIN;
            if (wait_event_interruptible(wq_data, (*offset != ((struct cfile_t*)filep->private_data)->receiver_write_pos)))
                return -ERESTARTSYS;
        }
        if (*offset == ((struct cfile_t*)filep->private_data)->receiver_write_pos)
            break;
        if (put_user(((struct cfile_t*)filep->private_data)->receiver_buffer[*offset % RECEIVER_BUFFER_SIZE], buffer))
            return -EIO;
        buffer++;
        r++;
        (*offset)++;
        len--;
    }
    return r;
}
 
/* This function is called whenever the device is being written to from user space i.e.
 * data is sent to the device from the user. The data is copied to the message[] array in this
 * LKM using the sprintf() function along with the length of the string.
 */
static ssize_t clunet_dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
    ssize_t r;
    u16 i, p, l;
    u8 v;
    char ch;
    u8* decoded;
    int d_address;
    r = 0;
    d_address = ((struct cfile_t*)filep->private_data)->device_address;
    while (r < len) {
        if (((struct cfile_t*)filep->private_data)->transmitter_write_pos 
            >= TRANSMITTER_BUFFER_SIZE) {
            ((struct cfile_t*)filep->private_data)->transmitter_write_pos = 0;
            return -ENOBUFS;
        }
        if (get_user(ch, buffer))
            return -EIO;
        r++;
        buffer++;
        if (ch == '\n') { /* End of line */
            l = ((struct cfile_t*)filep->private_data)->transmitter_write_pos;
            if (!l) break; /* Empty line? Skip it */
            decoded = kmalloc(l / 2 + 1, GFP_KERNEL);
            if (!decoded)
                return -ENOMEM; /* Can't allocate memory */
            if (d_address < 0) { /* Bus? */
                p = 1;
                decoded[0] = 0;
            } else { /* Device? Skip priority, source address and dst address */
                p = 3 * 2;
                decoded[0] = 4; /* Always high priority */
                decoded[1] = address; /* Our address */
                decoded[2] = d_address; /* Device address */
            }
            for (i = 0; i < l; i++) {
                ch = ((struct cfile_t*)filep->private_data)->transmitter_buffer[i];
                if (ch >= '0' && ch <= '9')
                    v = ch - '0';
                else if (ch >= 'a' && ch <= 'f')
                    v = ch - 'a' + 10;
                else if (ch >= 'A' && ch <= 'F')
                    v = ch - 'A' + 10;
                else
                    continue;
                if (p & 1)
                    decoded[p/2] |= v;
                else
                    decoded[p/2] = v << 4;
                p++;
            }
            ((struct cfile_t*)filep->private_data)->transmitter_write_pos = 0;
            p >>= 1;
            if (p < 4 || decoded[0] == 0 || decoded[0] > 4) {
                /* Invalid command */
                kfree(decoded);
                return -EBADMSG;
            }
            /* Only one thread can really wait for free line */
            if (mutex_lock_interruptible(&m_wait_for_line)) {
                /* Interrupted, fine */
                kfree(decoded);
                return -ERESTARTSYS;
            }
            if (clunet_sending_state != CLUNET_SENDING_STATE_IDLE) {
                /* Line is busy, should i block? */
                if (filep->f_flags & O_NONBLOCK) {
                    /* Oh, i shouldn't wait for line, bye-bye */
                    kfree(decoded);
                    mutex_unlock(&m_wait_for_line);
                    return -EBUSY;
                }
                /* Blocking and sleeping until transmitter ready */
                if (wait_event_interruptible(
                    wq_data, clunet_sending_state
                    == CLUNET_SENDING_STATE_IDLE)) {
                    kfree(decoded);
                    mutex_unlock(&m_wait_for_line);
                    return -ERESTARTSYS;
                }
            }
            /* Ok, it's time to enqueue data */
            clunet_send(decoded[1], decoded[2], decoded[0], decoded[3], decoded+4, p-4);
            /* Cleanup */
            kfree(decoded);
            /* Unlocking mutex */
            mutex_unlock(&m_wait_for_line);
            /* Return to userspace */
            break;
        }
        ((struct cfile_t*)filep->private_data)->transmitter_buffer[
            ((struct cfile_t*)filep->private_data)->transmitter_write_pos++
        ] = ch;
    }
    return r;
}

/* The device release function that is called whenever the device is closed/released by
 * the userspace program
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

/* File operations */
static struct file_operations fops =
{
   .open = clunet_dev_open,
   .read = clunet_dev_read,
   .write = clunet_dev_write,
   .release = clunet_dev_release,
};

/* Main init function */
static int __init clunet_init(void)
{
    int r;
    dev_t dev;
    /* Allocate and prepare memory */
    memset(opened_files, 0, sizeof(opened_files));
    /* Allocate and init the timer */
    data_send_timer = kzalloc(sizeof(struct hrtimer), GFP_KERNEL);
    if (!data_send_timer) {
        pr_err("CLUNET: can't allocate memory for timer\n");
        clunet_free();
        return -1;
    }
    hrtimer_init(data_send_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    data_send_timer->function = send_timer_callback;
    /* Allocate the receiver pin */
    receive_pin_desc = gpio_to_desc(receive_pin);
    if (IS_ERR(receive_pin_desc)) {
        pr_err("CLUNET: receive_pin gpiod_request error: %ld\n", PTR_ERR(receive_pin_desc));
        clunet_free();
        return -1;
    }
    gpiod_direction_input(receive_pin_desc);
    /* Allocate the transmitter pin */
    transmit_pin_desc = gpio_to_desc(transmit_pin);
    if (IS_ERR(transmit_pin_desc)) {
        pr_err("CLUNET: transmit_pin gpiod_request error: %ld\n", PTR_ERR(transmit_pin_desc));
        clunet_free();
        return -1;
    }
    clunet_set_line(0);
    /* Prepare the IRQ for receiver */
    irq_number = gpiod_to_irq(receive_pin_desc);
    r = request_irq(
        /* The interrupt number requested */
        irq_number,
        /* The pointer to the handler function below */
        (irq_handler_t) clunet_irq_handler,
        /* Interrupt on falling edge */
        IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
        /* Used in /proc/interrupts to identify the owner */
        "clunet_handler",
        NULL);
    if (r) {
        pr_err("CLUNET: receive_pin request_irq error: %d\n", r);
        clunet_free();
        return -1;
    }
    /* Register the device class */
    clunet_class = class_create(THIS_MODULE, CLASS_NAME);
    if (clunet_class == NULL) {
        pr_err("CLUNET: failed to register device class\n");
        clunet_free();
        return -1;
    }
    /* Allocate character device region for bus (1 device) */
    r = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME_BUS);
    if (r) {
        pr_err("CLUNET: failed to allocate chrdev region: %d\n", r);
        clunet_free();
        return -1;
    }
    clunet_bus_major = MAJOR(dev);
    /* Create the bus device in /dev */
    clunet_bus_device = device_create(clunet_class, NULL, dev, NULL, DEVICE_NAME_BUS_FILE);
    if (clunet_bus_device == NULL) {
        pr_err("CLUNET: failed to create /dev/%s\n", DEVICE_NAME_BUS_FILE);
        clunet_free();
        return -1;
    }
    /* Attach it to file operations */
    cdev_init(&clunet_bus_cdev, &fops);
    clunet_bus_cdev.owner = THIS_MODULE;
    r = cdev_add(&clunet_bus_cdev, dev, 1);
    if (r) {
        pr_err("CLUNET: failed to add chrdev: %d\n", r);
        clunet_free();
        return -1;
    }
    /* Allocate character device region for CLUNET devices (256 devices) */
    r = alloc_chrdev_region(&dev, 0, 256, DEVICE_NAME_CUSTOM);
    if (r) {
        pr_err("CLUNET: failed to allocate chrdev region (device): %d\n", r);
        clunet_free();
        return -1;
    }
    /* Attach them to file operations */
    clunet_device_major = MAJOR(dev);
    cdev_init(&clunet_device_cdev, &fops);
    clunet_device_cdev.owner = THIS_MODULE;
    r = cdev_add(&clunet_device_cdev, dev, 256);
    if (r) {
        pr_err("CLUNET: failed to add chrdev (device): %d\n", r);
        clunet_free();
        return -1;
    }
    /* Send startup message */
    clunet_send (
        address,
        CLUNET_BROADCAST_ADDRESS,
        CLUNET_PRIORITY_MESSAGE,
        CLUNET_COMMAND_BOOT_COMPLETED,
        NULL,
        0
    );
    pr_info("CLUNET: started, major numbers: %d (bus), %d (devices), local address: 0x%02X, device name: %s\n",
        clunet_bus_major, clunet_device_major, address, device_name);
    return 0;
}

/* Function to release all handles */
static void clunet_free(void)
{
    /* Timer */
    if (data_send_timer) {
        hrtimer_try_to_cancel(data_send_timer);
        kfree(data_send_timer);
    }
    /* Devices */
    if (clunet_class && clunet_bus_major) {
        cdev_del(&clunet_bus_cdev);
        device_destroy(clunet_class, MKDEV(clunet_bus_major, 0));
    }
    /* chrdev regions */
    if (clunet_device_major) {
        cdev_del(&clunet_device_cdev);
        unregister_chrdev_region(MKDEV(clunet_device_major, 0), 256);
    }
    if (clunet_bus_major)
        unregister_chrdev_region(MKDEV(clunet_bus_major, 0), 1);
    /* Class */
    if (clunet_class) {
        /* Unregister the device class */
        class_unregister(clunet_class);
        /* Remove the device class */
        class_destroy(clunet_class);
    }
    /* IRQ */
    if (irq_number != 0xffff)
        free_irq(irq_number, NULL);
    /* GPIO pins */
    if (!IS_ERR_OR_NULL(receive_pin_desc))
        gpiod_put(receive_pin_desc);
    if (!IS_ERR_OR_NULL(transmit_pin_desc))
        gpiod_put(transmit_pin_desc);
}

/* Exit function */
static void __exit clunet_exit(void)
{
    clunet_free();
    pr_info("CLUNET: stopped\n");
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexey 'Cluster' Avdyukhin <clusterrr@clusterrr.com>");
MODULE_DESCRIPTION("CLUNET bus driver");

module_init(clunet_init);
module_exit(clunet_exit);
