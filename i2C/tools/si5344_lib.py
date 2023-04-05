from smbus import SMBus


def read_config(config_filename):
    '''
    reg_file_name --> ouput of the Clock Builder C header file
    returns list of addresses and values including the preamble and the postamble
    '''
    encd = "latin1" #encoding
    TOTAL_REG_NUM = 0 #total number of registers actions
    addresses = []
    values = []
    with open(config_filename, encoding=encd) as f:
        for i in f.readlines():
            if ("NUM_REGS" in i) and ("#define" in i):
                TOTAL_REG_NUM = int(i.split("\t")[-1])
            if ("{" in i) & ("}" in i) & ("0x" in i):
                addr,val = i.split(" ")[1:3]
                addresses.append(addr.split(",")[0])
                values.append(val.split(",")[0])
                    # print(addr,val)
                # break
            # else:
            #     print(i)

    print("Total Registers Detected: ",TOTAL_REG_NUM)
    print("Total Registers Read    : ", len(addresses))
    return addresses,values

SET_ADDR  = 0b00000000; #Command to set address
READ_CMD  = 0b10000000; #Command to Read Data
WRITE_CMD = 0b01000000; #Command to Write Data
PAGE_ADDR = 0b00000001; #Page Address

def si5344_comm(spi, addr, val=0,write=0):
    '''
    spi --> Spi object
    addr --> address
    val --> value to write to address
    write:
      1: Will write the value
      0: Will read the value
    '''


    page_byte = (int(addr,base=16)>>8)&0xFF
    addr_byte = (int(addr,base=16))&0xFF
    val_byte  = int(val,base=16)&0xFF

    #Set Correct Page
    spi.xfer([PAGE_ADDR, SET_ADDR])
    # spi.xfer([PAGE_ADDR, SET_ADDR][::-1])

    spi.xfer([page_byte, WRITE_CMD])
    # spi.xfer([page_byte, WRITE_CMD][::-1])


    if write==1:
        #set addr
        spi.xfer([addr_byte, SET_ADDR])#Setting the address
        #write addr
        return spi.xfer([val_byte, WRITE_CMD])
    elif write == 0:
        spi.xfer([addr_byte, SET_ADDR]) #Setting the address
        return spi.xfer([val_byte, READ_CMD])

class si5344_i2c():
    def __init__(self,i2cbus,i2caddress):
        self.i2cbus = i2cbus
        self.i2caddress = i2caddress
        self.PAGE_ADDR = 0x1; #Page Address
        
    def write_i2c (self,address,data):
        self.i2cbus.write_byte_data(self.i2caddress,address,data)
        # print(bin(i2caddress))

    def read_i2c(self,address):
        self.i2cbus.write_byte(self.i2caddress,address)
        return self.i2cbus.read_byte(self.i2caddress)
    
    def set_page(self,page):
        self.write_i2c(self.PAGE_ADDR,page)
        #Check if the page is correct...
        if self.read_i2c(self.PAGE_ADDR) != page:
            print("Unsuccessful Page Settings!")

    def transfer(self,addr,val):
        page_byte = (int(addr,base=16)>>8)&0xFF
        addr_byte = (int(addr,base=16))&0xFF
        val_byte  = int(val,base=16)&0xFF

        self.set_page(page_byte)
        self.write_i2c(addr_byte,val_byte)
        reading  = self.read_i2c(addr_byte)
        if val_byte != reading:
            print(hex(val_byte), hex(reading))
