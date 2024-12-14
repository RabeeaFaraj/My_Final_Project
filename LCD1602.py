import time
import smbus2 as smbus

# Initialize I2C bus for communication with the LCD
BUS = smbus.SMBus(1)

def write_word(addr, data):
    """
    Writes a word (byte) of data to the specified I2C address.

    Parameters:
    - addr (int): I2C address of the device (LCD).
    - data (int): Data byte to send to the LCD.
    """
    global BLEN
    temp = data
    # Control backlight by adjusting the data byte
    if BLEN == 1:
        temp |= 0x08  # Turn backlight on
    else:
        temp &= 0xF7  # Turn backlight off
    BUS.write_byte(addr, temp)

def send_command(comm):
    """
    Sends a command to the LCD.

    Parameters:
    - comm (int): Command byte to send to the LCD.
    """
    # Send higher nibble (bit7-4) first
    buf = comm & 0xF0
    buf |= 0x04  # RS = 0, RW = 0, EN = 1
    write_word(LCD_ADDR, buf)
    time.sleep(0.002)
    buf &= 0xFB  # Set EN = 0
    write_word(LCD_ADDR, buf)

    # Send lower nibble (bit3-0) second
    buf = (comm & 0x0F) << 4
    buf |= 0x04  # RS = 0, RW = 0, EN = 1
    write_word(LCD_ADDR, buf)
    time.sleep(0.002)
    buf &= 0xFB  # Set EN = 0
    write_word(LCD_ADDR, buf)

def send_data(data):
    """
    Sends data to be displayed on the LCD.

    Parameters:
    - data (int): Data byte to send to the LCD.
    """
    # Send higher nibble (bit7-4) first
    buf = data & 0xF0
    buf |= 0x05  # RS = 1, RW = 0, EN = 1
    write_word(LCD_ADDR, buf)
    time.sleep(0.002)
    buf &= 0xFB  # Set EN = 0
    write_word(LCD_ADDR, buf)

    # Send lower nibble (bit3-0) second
    buf = (data & 0x0F) << 4
    buf |= 0x05  # RS = 1, RW = 0, EN = 1
    write_word(LCD_ADDR, buf)
    time.sleep(0.002)
    buf &= 0xFB  # Set EN = 0
    write_word(LCD_ADDR, buf)

def init(addr, bl):
    """
    Initializes the LCD with the specified I2C address and backlight setting.

    Parameters:
    - addr (int): I2C address of the LCD.
    - bl (int): Backlight enable (1 for on, 0 for off).

    Returns:
    - bool: True if initialization is successful, False otherwise.
    """
    global LCD_ADDR
    global BLEN
    LCD_ADDR = addr
    BLEN = bl
    try:
        # Initialize to 8-bit mode, then switch to 4-bit mode
        send_command(0x33)
        time.sleep(0.005)
        send_command(0x32)
        time.sleep(0.005)
        # Set display to 2-line mode and 5x7 dot format
        send_command(0x28)
        time.sleep(0.005)
        # Enable display, disable cursor
        send_command(0x0C)
        time.sleep(0.005)
        # Clear the screen
        send_command(0x01)
        # Turn on the backlight
        BUS.write_byte(LCD_ADDR, 0x08)
    except:
        return False
    else:
        return True

def clear():
    """
    Clears the LCD screen.
    """
    send_command(0x01)  # Send clear screen command

def openlight():
    """
    Turns on the LCD backlight.
    """
    BUS.write_byte(0x27, 0x08)
    BUS.close()

def write(x, y, str):
    """
    Writes a string to a specified position on the LCD.

    Parameters:
    - x (int): Column position (0-15).
    - y (int): Row position (0-1).
    - str (str): Text to display on the LCD.
    """
    # Clamp x and y to valid LCD positions
    if x < 0:
        x = 0
    if x > 15:
        x = 15
    if y < 0:
        y = 0
    if y > 1:
        y = 1

    # Calculate cursor address based on position
    addr = 0x80 + 0x40 * y + x
    send_command(addr)

    # Write each character to the LCD
    for chr in str:
        send_data(ord(chr))  # Convert character to ASCII and send

# Example usage of LCD functions
if _name_ == '_main_':
    init(0x27, 1)  # Initialize LCD at address 0x27 with backlight on
    write(4, 0, 'Hello')  # Write "Hello" at row 0, column 4
    write(7, 1, 'world!')  # Write "world!" at row 1, column 7