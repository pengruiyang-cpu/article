{{FirstPerson}}
== Network Driver for Intel Ethernet Cards I217 and 82577LM ==

I am writing this Wiki as a demonstration of my own experience of getting a working driver for the Intel I217 and 82577LM network cards to work, on a real native bare metal hardware, namely Thinkpads W540 and W510. Linux uses the e1000e network driver for those cards. I have started from a working e1000 driver that I have developed for my OS and which is operational on Qemu, Bochs, and VirtualBox. The main objective of this Wiki is to try to highlight the differences and the addition needed on an operations e1000 driver to work handle network cards that work with the e1000e. So the provided knowledge in this wiki might be applicable on other Intel interfaces. For completion, I will present in this wiki my e1000 driver with the additions that made it work on those native NICs, I217 and 82577LM. I built my original e1000 driver based on information from OSDev and some hobby operating systems on github.

It is very important to highlight that this wiki does not utilize all the features in the above NICs, but it show how to configure the NICs for basic functionality such as initialization, read packets, and write packets. 

== Card Addresses and Data Structures ==

To start with, lets state some macro definitions that we are going to use in the code.
<source lang="c">

#define INTEL_VEND     0x8086  // Vendor ID for Intel 
#define E1000_DEV      0x100E  // Device ID for the e1000 Qemu, Bochs, and VirtualBox emmulated NICs
#define E1000_I217     0x153A  // Device ID for Intel I217
#define E1000_82577LM  0x10EA  // Device ID for Intel 82577LM


// I have gathered those from different Hobby online operating systems instead of getting them one by one from the manual

#define REG_CTRL        0x0000
#define REG_STATUS      0x0008
#define REG_EEPROM      0x0014
#define REG_CTRL_EXT    0x0018
#define REG_IMASK       0x00D0
#define REG_RCTRL       0x0100
#define REG_RXDESCLO    0x2800
#define REG_RXDESCHI    0x2804
#define REG_RXDESCLEN   0x2808
#define REG_RXDESCHEAD  0x2810
#define REG_RXDESCTAIL  0x2818

#define REG_TCTRL       0x0400
#define REG_TXDESCLO    0x3800
#define REG_TXDESCHI    0x3804
#define REG_TXDESCLEN   0x3808
#define REG_TXDESCHEAD  0x3810
#define REG_TXDESCTAIL  0x3818


#define REG_RDTR         0x2820 // RX Delay Timer Register
#define REG_RXDCTL       0x3828 // RX Descriptor Control
#define REG_RADV         0x282C // RX Int. Absolute Delay Timer
#define REG_RSRPD        0x2C00 // RX Small Packet Detect Interrupt



#define REG_TIPG         0x0410      // Transmit Inter Packet Gap
#define ECTRL_SLU        0x40        //set link up


#define RCTL_EN                         (1 << 1)    // Receiver Enable
#define RCTL_SBP                        (1 << 2)    // Store Bad Packets
#define RCTL_UPE                        (1 << 3)    // Unicast Promiscuous Enabled
#define RCTL_MPE                        (1 << 4)    // Multicast Promiscuous Enabled
#define RCTL_LPE                        (1 << 5)    // Long Packet Reception Enable
#define RCTL_LBM_NONE                   (0 << 6)    // No Loopback
#define RCTL_LBM_PHY                    (3 << 6)    // PHY or external SerDesc loopback
#define RTCL_RDMTS_HALF                 (0 << 8)    // Free Buffer Threshold is 1/2 of RDLEN
#define RTCL_RDMTS_QUARTER              (1 << 8)    // Free Buffer Threshold is 1/4 of RDLEN
#define RTCL_RDMTS_EIGHTH               (2 << 8)    // Free Buffer Threshold is 1/8 of RDLEN
#define RCTL_MO_36                      (0 << 12)   // Multicast Offset - bits 47:36
#define RCTL_MO_35                      (1 << 12)   // Multicast Offset - bits 46:35
#define RCTL_MO_34                      (2 << 12)   // Multicast Offset - bits 45:34
#define RCTL_MO_32                      (3 << 12)   // Multicast Offset - bits 43:32
#define RCTL_BAM                        (1 << 15)   // Broadcast Accept Mode
#define RCTL_VFE                        (1 << 18)   // VLAN Filter Enable
#define RCTL_CFIEN                      (1 << 19)   // Canonical Form Indicator Enable
#define RCTL_CFI                        (1 << 20)   // Canonical Form Indicator Bit Value
#define RCTL_DPF                        (1 << 22)   // Discard Pause Frames
#define RCTL_PMCF                       (1 << 23)   // Pass MAC Control Frames
#define RCTL_SECRC                      (1 << 26)   // Strip Ethernet CRC

// Buffer Sizes
#define RCTL_BSIZE_256                  (3 << 16)
#define RCTL_BSIZE_512                  (2 << 16)
#define RCTL_BSIZE_1024                 (1 << 16)
#define RCTL_BSIZE_2048                 (0 << 16)
#define RCTL_BSIZE_4096                 ((3 << 16) | (1 << 25))
#define RCTL_BSIZE_8192                 ((2 << 16) | (1 << 25))
#define RCTL_BSIZE_16384                ((1 << 16) | (1 << 25))


// Transmit Command

#define CMD_EOP                         (1 << 0)    // End of Packet
#define CMD_IFCS                        (1 << 1)    // Insert FCS
#define CMD_IC                          (1 << 2)    // Insert Checksum
#define CMD_RS                          (1 << 3)    // Report Status
#define CMD_RPS                         (1 << 4)    // Report Packet Sent
#define CMD_VLE                         (1 << 6)    // VLAN Packet Enable
#define CMD_IDE                         (1 << 7)    // Interrupt Delay Enable


// TCTL Register

#define TCTL_EN                         (1 << 1)    // Transmit Enable
#define TCTL_PSP                        (1 << 3)    // Pad Short Packets
#define TCTL_CT_SHIFT                   4           // Collision Threshold
#define TCTL_COLD_SHIFT                 12          // Collision Distance
#define TCTL_SWXOFF                     (1 << 22)   // Software XOFF Transmission
#define TCTL_RTLC                       (1 << 24)   // Re-transmit on Late Collision

#define TSTA_DD                         (1 << 0)    // Descriptor Done
#define TSTA_EC                         (1 << 1)    // Excess Collisions
#define TSTA_LC                         (1 << 2)    // Late Collision
#define LSTA_TU                         (1 << 3)    // Transmit Underrun
</source>

Now lets define the data structures for the transmit and receive buffers

<source lang="c">

#define E1000_NUM_RX_DESC 32
#define E1000_NUM_TX_DESC 8

struct e1000_rx_desc {
        volatile uint64_t addr;
        volatile uint16_t length;
        volatile uint16_t checksum;
        volatile uint8_t status;
        volatile uint8_t errors;
        volatile uint16_t special;
} __attribute__((packed));

struct e1000_tx_desc {
        volatile uint64_t addr;
        volatile uint16_t length;
        volatile uint8_t cso;
        volatile uint8_t cmd;
        volatile uint8_t status;
        volatile uint8_t css;
        volatile uint16_t special;
} __attribute__((packed));

</source>

And finally some helper static methods for MMIO read/write operations and Ports I/O

<source lang="c">

class MMIOUtils
{
    public:
        static uint8_t read8 (uint64_t p_address);
        static uint16_t read16 (uint64_t p_address);
        static uint32_t read32 (uint64_t p_address);
        static uint64_t read64 (uint64_t p_address);
        static void write8 (uint64_t p_address,uint8_t p_value);
        static void write16 (uint64_t p_address,uint16_t p_value);
        static void write32 (uint64_t p_address,uint32_t p_value);
        static void write64 (uint64_t p_address,uint64_t p_value);
};


uint8_t MMIOUtils::read8 (uint64_t p_address)
{
    return *((volatile uint8_t*)(p_address));
}
uint16_t MMIOUtils::read16 (uint64_t p_address)
{
    return *((volatile uint16_t*)(p_address));
    
}
uint32_t MMIOUtils::read32 (uint64_t p_address)
{
    return *((volatile uint32_t*)(p_address));
    
}
uint64_t MMIOUtils::read64 (uint64_t p_address)
{
    return *((volatile uint64_t*)(p_address));    
}
void MMIOUtils::write8 (uint64_t p_address,uint8_t p_value)
{
    (*((volatile uint8_t*)(p_address)))=(p_value);
}
void MMIOUtils::write16 (uint64_t p_address,uint16_t p_value)
{
    (*((volatile uint16_t*)(p_address)))=(p_value);    
}
void MMIOUtils::write32 (uint64_t p_address,uint32_t p_value)
{
    (*((volatile uint32_t*)(p_address)))=(p_value);
    
}
void MMIOUtils::write64 (uint64_t p_address,uint64_t p_value)
{
    (*((volatile uint64_t*)(p_address)))=(p_value);    
}

</source>

<source lang="c">
#ifndef PORTS_H_
#define PORTS_H_


class Ports
{
    private:
    public:
        static void outportb (uint16_t p_port,uint8_t data);
        static void outportw (uint16_t p_port,uint16_t data);
        static void outportl (uint16_t p_port,uint32_t data);
        static uint8_t inportb( uint16_t p_port);
        static uint16_t inportw( uint16_t p_port);
        static uint32_t inportl( uint16_t p_port);
};

#endif /* PORTS_H_ */


/* void Ports::outportb (uint16_t p_port,uint8_t p_data)
 * 
 * This method outputs a byte to a hardware port.
 * It uses an inline asm with the volatile keyword
 * to disable compiler optimization.
 * 
 *  p_port: the port number to output the byte p_data to.
 *  p_data: the byte to to output to the port p_port.
 * 
 * Notice the input constraint
 *      "dN" (port) : indicates using the DX register to store the 
 *                  value of port in it
 *      "a"  (data) : store the value of data into 
 * 
 * The above constraint will instruct the compiler to generate assembly
 * code that looks like that
 *      mov    %edi,%edx
 *      mov    %esi,%eax
 *      out    %eax,(%dx)
 * 
 * According the ABI, the edi will have the value of p_port and esi will have
 * the value of the p_data
 * 
 */
void Ports::outportb (uint16_t p_port,uint8_t p_data)
{
    asm volatile ("outb %1, %0" : : "dN" (p_port), "a" (p_data));
}

/* void Ports::outportw (uint16_t p_port,uint16_t p_data)
 * 
 * This method outputs a word to a hardware port.
 * 
 *  p_port: the port number to output the byte p_data to.
 *  p_data: the byte to to output to the port p_port.
 * 
 */


void Ports::outportw (uint16_t p_port,uint16_t p_data)
{
    asm volatile ("outw %1, %0" : : "dN" (p_port), "a" (p_data));
}

/* void Ports::outportl (uint16_t p_port,uint32_t p_data)
 * 
 * This method outputs a double word to a hardware port.
 * 
 *  p_port: the port number to output the byte p_data to.
 *  p_data: the byte to to output to the port p_port.
 * 
 */


void Ports::outportl (uint16_t p_port,uint32_t p_data)
{
    asm volatile ("outl %1, %0" : : "dN" (p_port), "a" (p_data));
}

/* uint8_t Ports::inportb( uint16_t p_port)
 * 
 * This method reads a byte from a hardware port.
 * 
 *  p_port: the port number to read the byte from.
 *  return value : a byte read from the port p_port.
 * 
 * Notice the output constraint "=a", this tells the compiler 
 * to expect the save the value of register AX into the variable l_ret
 * The register AX should contain the result of the inb instruction.
 * 
 * 
 */

uint8_t Ports::inportb( uint16_t p_port)
{
    uint8_t l_ret;
    asm volatile("inb %1, %0" : "=a" (l_ret) : "dN" (p_port));
    return l_ret;
}

/* uint16_t Ports::inportw( uint16_t p_port)
 * 
 * This method reads a word from a hardware port.
 * 
 *  p_port: the port number to read the word from.
 *  return value : a word read from the port p_port.
 * 
 */


uint16_t Ports::inportw( uint16_t p_port)
{
    uint16_t l_ret;
    asm volatile ("inw %1, %0" : "=a" (l_ret) : "dN" (p_port));
    return l_ret;
}


/* uint16_t Ports::inportl( uint16_t p_port)
 * 
 * This method reads a double word from a hardware port.
 * 
 *  p_port: the port number to read the double word from.
 *  return value : a double word read from the port p_port.
 * 
 */

uint32_t Ports::inportl( uint16_t p_port)
{
    uint32_t l_ret;
    asm volatile ("inl %1, %0" : "=a" (l_ret) : "dN" (p_port));
    return l_ret;
}


</source>

== The Driver Class Header (Class Definition)==


<source lang="c">
class E1000 : public NetworkDriver
{
    private:
        
        uint8_t bar_type;     // Type of BOR0
        uint16_t io_base;     // IO Base Address
        uint64_t  mem_base;   // MMIO Base Address
        bool eerprom_exists;  // A flag indicating if eeprom exists
        uint8_t mac [6];      // A buffer for storing the mack address
        struct e1000_rx_desc *rx_descs[E1000_NUM_RX_DESC]; // Receive Descriptor Buffers
        struct e1000_tx_desc *tx_descs[E1000_NUM_TX_DESC]; // Transmit Descriptor Buffers
        uint16_t rx_cur;      // Current Receive Descriptor Buffer
        uint16_t tx_cur;      // Current Transmit Descriptor Buffer
        
        
        // Send Commands and read results From NICs either using MMIO or IO Ports
        void writeCommand( uint16_t p_address, uint32_t p_value);
        uint32_t readCommand(uint16_t p_address);


        bool detectEEProm(); // Return true if EEProm exist, else it returns false and set the eerprom_existsdata member
        uint32_t eepromRead( uint8_t addr); // Read 4 bytes from a specific EEProm Address
        bool readMACAddress();       // Read MAC Address
        void startLink ();           // Start up the network
        void rxinit();               // Initialize receive descriptors an buffers
        void txinit();               // Initialize transmit descriptors an buffers
        void enableInterrupt();      // Enable Interrupts
        void handleReceive();        // Handle a packet reception.
    public:

        E1000(PCIConfigHeader * _pciConfigHeader); // Constructor. takes as a parameter a pointer to an object that encapsulate all he PCI configuration data of the device
        void start ();                             // perform initialization tasks and starts the driver
        void fire (InterruptContext * p_interruptContext);  // This method should be called by the interrupt handler 
        uint8_t * getMacAddress ();                         // Returns the MAC address
        int sendPacket(const void * p_data, uint16_t p_len);  // Send a packet
        ~E1000();                                             // Default Destructor
};
</source>

== How the Gears Move (Class methods implementation) ==

First of all we need to be able to send commands and read results from the NIC. It is important to detect the type of BAR0 and based on that the correct communication mechanism should be adopted. The following two methods encapsulate the read/write commands and uses MMIO or IO ports based on the value in BAR0 which is reflected in bar_type data member flag.

<source lang="c">
void E1000::writeCommand( uint16_t p_address, uint32_t p_value)
{
    if ( bar_type == 0 )
    {
        MMIOUtils::write32(mem_base+p_address,p_value);
    }
    else
    {
        Ports::outportl(io_base, p_address);
        Ports::outportl(io_base + 4, p_value);
    }
}
uint32_t E1000::readCommand( uint16_t p_address)
{
    if ( bar_type == 0 )
    {
        return MMIOUtils::read32(mem_base+p_address);
    }
    else
    {
        Ports::outportl(io_base, p_address);
        return Ports::inportl(io_base + 4);
    }
} 
</source>


Now we need to detect if the card has an EEPROM or not. The Qemu and Bochs emulate EEPROM, but the I217 and 82577LM do not. The following first method tries to read the status field of the EEPROM, the status field should contain the value 0x10, and based on the result the internal data member eerprom_exists. The second method performs a 2-bytes read operation from the EEPROM

<source lang="c">
bool E1000::detectEEProm()
{
    uint32_t val = 0;
    writeCommand(REG_EEPROM, 0x1); 

    for(int i = 0; i < 1000 && ! eerprom_exists; i++)
    {
            val = readCommand( REG_EEPROM);
            if(val & 0x10)
                    eerprom_exists = true;
            else
                    eerprom_exists = false;
    }
    return eerprom_exists;
}

uint32_t E1000::eepromRead( uint8_t addr)
{
	uint16_t data = 0;
	uint32_t tmp = 0;
        if ( eerprom_exists)
        {
            	writeCommand( REG_EEPROM, (1) | ((uint32_t)(addr) << 8) );
        	while( !((tmp = readCommand(REG_EEPROM)) & (1 << 4)) );
        }
        else
        {
            writeCommand( REG_EEPROM, (1) | ((uint32_t)(addr) << 2) );
            while( !((tmp = readCommand(REG_EEPROM)) & (1 << 1)) );
        }
	data = (uint16_t)((tmp >> 16) & 0xFFFF);
	return data;
}

</source>



The first thing you will need to do after detecting the BAR0 type and the existence of the EEPROM is to read the hardware MAC address of the NIC. The following method reads the hardware mac address based. If an EEPROM exists it will read it from the EEPROM else it will read it from address 0x5400 where it should be located in that case. It is very important to detect if an EEPROM exists or not prior to reading the MAC address.

<source lang="c">

bool E1000::readMACAddress()
{
    if ( eerprom_exists)
    {
        uint32_t temp;
        temp = eepromRead( 0);
        mac[0] = temp &0xff;
        mac[1] = temp >> 8;
        temp = eepromRead( 1);
        mac[2] = temp &0xff;
        mac[3] = temp >> 8;
        temp = eepromRead( 2);
        mac[4] = temp &0xff;
        mac[5] = temp >> 8;
    }
    else
    {
        uint8_t * mem_base_mac_8 = (uint8_t *) (mem_base+0x5400);
        uint32_t * mem_base_mac_32 = (uint32_t *) (mem_base+0x5400);
        if ( mem_base_mac_32[0] != 0 )
        {
            for(int i = 0; i < 6; i++)
            {
                mac[i] = mem_base_mac_8[i];
            }
        }
        else return false;
    }
    return true;
}
</source>


Now, we need to configure the transmit and receive descriptor buffers, here are the implementation of the corresponding methods. The rxinit method is identical to the one I use for my e1000 driver. The difference is in txinit

<source lang="c">

void E1000::rxinit()
{
    uint8_t * ptr;
    struct e1000_rx_desc *descs;

    // Allocate buffer for receive descriptors. For simplicity, in my case khmalloc returns a virtual address that is identical to it physical mapped address.
    // In your case you should handle virtual and physical addresses as the addresses passed to the NIC should be physical ones
 
    ptr = (uint8_t *)(kmalloc_ptr->khmalloc(sizeof(struct e1000_rx_desc)*E1000_NUM_RX_DESC + 16));

    descs = (struct e1000_rx_desc *)ptr;
    for(int i = 0; i < E1000_NUM_RX_DESC; i++)
    {
        rx_descs[i] = (struct e1000_rx_desc *)((uint8_t *)descs + i*16);
        rx_descs[i]->addr = (uint64_t)(uint8_t *)(kmalloc_ptr->khmalloc(8192 + 16));
        rx_descs[i]->status = 0;
    }

    writeCommand(REG_TXDESCLO, (uint32_t)((uint64_t)ptr >> 32) );
    writeCommand(REG_TXDESCHI, (uint32_t)((uint64_t)ptr & 0xFFFFFFFF));

    writeCommand(REG_RXDESCLO, (uint64_t)ptr);
    writeCommand(REG_RXDESCHI, 0);

    writeCommand(REG_RXDESCLEN, E1000_NUM_RX_DESC * 16);

    writeCommand(REG_RXDESCHEAD, 0);
    writeCommand(REG_RXDESCTAIL, E1000_NUM_RX_DESC-1);
    rx_cur = 0;
    writeCommand(REG_RCTRL, RCTL_EN| RCTL_SBP| RCTL_UPE | RCTL_MPE | RCTL_LBM_NONE | RTCL_RDMTS_HALF | RCTL_BAM | RCTL_SECRC  | RCTL_BSIZE_8192);
    
}


void E1000::txinit()
{    
    uint8_t *  ptr;
    struct e1000_tx_desc *descs;
    // Allocate buffer for receive descriptors. For simplicity, in my case khmalloc returns a virtual address that is identical to it physical mapped address.
    // In your case you should handle virtual and physical addresses as the addresses passed to the NIC should be physical ones
    ptr = (uint8_t *)(kmalloc_ptr->khmalloc(sizeof(struct e1000_tx_desc)*E1000_NUM_TX_DESC + 16));

    descs = (struct e1000_tx_desc *)ptr;
    for(int i = 0; i < E1000_NUM_TX_DESC; i++)
    {
        tx_descs[i] = (struct e1000_tx_desc *)((uint8_t*)descs + i*16);
        tx_descs[i]->addr = 0;
        tx_descs[i]->cmd = 0;
        tx_descs[i]->status = TSTA_DD;
    }

    writeCommand(REG_TXDESCHI, (uint32_t)((uint64_t)ptr >> 32) );
    writeCommand(REG_TXDESCLO, (uint32_t)((uint64_t)ptr & 0xFFFFFFFF));


    //now setup total length of descriptors
    writeCommand(REG_TXDESCLEN, E1000_NUM_TX_DESC * 16);


    //setup numbers
    writeCommand( REG_TXDESCHEAD, 0);
    writeCommand( REG_TXDESCTAIL, 0);
    tx_cur = 0;
    writeCommand(REG_TCTRL,  TCTL_EN
        | TCTL_PSP
        | (15 << TCTL_CT_SHIFT)
        | (64 << TCTL_COLD_SHIFT)
        | TCTL_RTLC);

    // This line of code overrides the one before it but I left both to highlight that the previous one works with e1000 cards, but for the e1000e cards 
    // you should set the TCTRL register as follows. For detailed description of each bit, please refer to the Intel Manual.
    // In the case of I217 and 82577LM packets will not be sent if the TCTRL is not configured using the following bits.
    writeCommand(REG_TCTRL,  0b0110000000000111111000011111010);
    writeCommand(REG_TIPG,  0x0060200A);

}


</source>


To enable interrupts 
<source lang="c">
void E1000::enableInterrupt()
{
    writeCommand(REG_IMASK ,0x1F6DC);
    writeCommand(REG_IMASK ,0xff & ~4);
    readCommand(0xc0);

}
</source>

As we have defined most of the building blocks and the helper methods lets define the main methods of the class.


The constructor is responsible for fetching PCI related data and initialize the object internal state

<source lang="c">
E1000::E1000(PCIConfigHeader * p_pciConfigHeader) : NetworkDriver(p_pciConfigHeader)
{
    // Get BAR0 type, io_base address and MMIO base address
    bar_type = pciConfigHeader->getPCIBarType(0);
    io_base = pciConfigHeader->getPCIBar(PCI_BAR_IO) & ~1;
    mem_base = pciConfigHeader->getPCIBar( PCI_BAR_MEM) & ~3;    
   
    // Off course you will need here to map the memory address into you page tables and use corresponding virtual addresses

    // Enable bus mastering
    pciConfigHeader->enablePCIBusMastering();
    eerprom_exists = false;
}
</source>

The start method basically detects the EEPROM, reads the MAC addresses, setup rx and tx buffers, register the interrupt handler, and enable NIC interrupts


<source lang="c">

bool E1000::start ()
{
    detectEEProm ();
    if (! readMACAddress()) return false;
    printMac();
    startLink();
    
    for(int i = 0; i < 0x80; i++)
        writeCommand(0x5200 + i*4, 0);
    if ( interruptManager->registerInterrupt(IRQ0+pciConfigHeader->getIntLine(),this))
    {
        enableInterrupt();
        rxinit();
        txinit();        
        video.putString("E1000 card started\n",COLOR_RED,COLOR_WHITE);
        return true;
    }
    else return false;
}


</source>


Your interrupt handler should eventually call the fire method which handles the NIC's events
<source lang="c">

void E1000::fire (InterruptContext * p_interruptContext)
{
    if ( p_interruptContext->getInteruptNumber() == pciConfigHeader->getIntLine()+IRQ0)
    {        
        /* This might be needed here if your handler doesn't clear interrupts from each device and must be done before EOI if using the PIC.
           Without this, the card will spam interrupts as the int-line will stay high. */
        writeCommand(REG_IMASK, 0x1);
       
        uint32_t status = readCommand(0xc0);
        if(status & 0x04)
        {
            startLink();
        }
        else if(status & 0x10)
        {
           // good threshold
        }
        else if(status & 0x80)
        {
            handleReceive();
        }
    }
}

void E1000::handleReceive()
{
    uint16_t old_cur;
    bool got_packet = false;
 
    while((rx_descs[rx_cur]->status & 0x1))
    {
            got_packet = true;
            uint8_t *buf = (uint8_t *)rx_descs[rx_cur]->addr;
            uint16_t len = rx_descs[rx_cur]->length;

            // Here you should inject the received packet into your network stack


            rx_descs[rx_cur]->status = 0;
            old_cur = rx_cur;
            rx_cur = (rx_cur + 1) % E1000_NUM_RX_DESC;
            writeCommand(REG_RXDESCTAIL, old_cur );
    }    
}

</source>

Finally we define the sendPacket method as follows

<source lang="c">

int E1000::sendPacket(const void * p_data, uint16_t p_len)
{    
    tx_descs[tx_cur]->addr = (uint64_t)p_data;
    tx_descs[tx_cur]->length = p_len;
    tx_descs[tx_cur]->cmd = CMD_EOP | CMD_IFCS | CMD_RS;
    tx_descs[tx_cur]->status = 0;
    uint8_t old_cur = tx_cur;
    tx_cur = (tx_cur + 1) % E1000_NUM_TX_DESC;
    writeCommand(REG_TXDESCTAIL, tx_cur);   
    while(!(tx_descs[old_cur]->status & 0xff));    
    return 0;
}

</source>


This is an example of how to instantiate an object of this class and startup you driver. I assume that you have scanned your PCI buses and loaded the found devices parameters into some data structures; in our example this is done by the PCIConfigManager class, which is outside the scope of this tutorial

<source lang="c">

        pciConfigHeaderManager.initialize(); // Initialize the PCIConfigHeaderManager Object and scan PCI devices
        if ( e1000PCIConfigHeader == NULL ) e1000PCIConfigHeader = pciConfigHeaderManager.getPCIDevice(INTEL_VEND,E1000_I217);
        if ( e1000PCIConfigHeader == NULL ) e1000PCIConfigHeader = pciConfigHeaderManager.getPCIDevice(INTEL_VEND,E1000_82577LM);
        if ( e1000PCIConfigHeader != NULL )
        {
            E1000 * e1000 = new E1000(e1000PCIConfigHeader);
            if (!e1000->start())
            {
               // Error starting the NIC
            }
        }
        else
        {
           // Intel cards not found
        }

</source>

== Summary and Wrap Up ==

I have presented in this Wiki the steps I followed to make an e1000 driver work with the two e1000e NICs Intel I217 and 82577LM. The wiki does not show how to utilize all the features of the NICs, but basically primitive setup and send/receive packets. Three important issues that I faced:

* You need to detect the BAR0 type as some cards uses IO ports and others uses MMIO and you need to communincate with the NIC with the method appropriate to each.
* You need to check if the card supports an EEPROM and read the MAC address from the EEPROM if the card supports it or read it from address 0x5400 if it does not support an EEPROM
* You need to make sure to setup the card TCTRL register (Transmission Control Register) with the value 0b0110000000000111111000011111010. For more details reference the Intel manual for the meaning of the different bits of the register

== Manuals ==
These are the full Intel manuals:
[http://www.intel.com/content/www/us/en/ethernet-controllers/ethernet-controller-i217-spec-update.html Intel Ethernet i217 V]
[http://www.intel.com/content/www/us/en/ethernet-controllers/82577-gbe-phy-datasheet.html Intel 82577 Gigabit Ethernet PHY]

(Looks like the former manual was moved [http://www.intel.com/content/www/us/en/embedded/products/networking/i217-ethernet-controller-datasheet.html here]).

[[Category:Network Hardware]]
