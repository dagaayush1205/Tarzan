#include <stdio.h>
#include <stdint.h>

// start and end byte values for s bus
const uint16_t start = 00001111; // 0x0F
const uint16_t end = 00000000; // 0x00

// variable to store channel 
uint16_t channel[16];

// creating 11 bit channel
void parse_buffer(uint16_t buff[]) { 
		
	// masking byte shiftings bits (value in hexa '0x07FF')
	uint32_t mask = 0000011111111111;

	// creating channels 
        channel[0]  = ((buff[1] | buff[2]<<8)                 & mask);
        channel[1]  = ((buff[2]>>3 | buff[3]<<5)              & mask);
        channel[2]  = ((buff[3]>>6 | buff[4]<<2 | buff[5]<<10) & mask);
        channel[3]  = ((buff[5]>>1 | buff[6]<<7)              & mask);
        channel[4]  = ((buff[6]>>4 | buff[7]<<4)              & mask);
        channel[5]  = ((buff[7]>>7 | buff[8]<<1 | buff[9]<<9)  & mask);
        channel[6]  = ((buff[9]>>2 | buff[10]<<6)             & mask);
        channel[7]  = ((buff[10]>>5| buff[11]<<3)             & mask);
        channel[8]  = ((buff[12]   | buff[13]<<8)             & mask);
        channel[9]  = ((buff[13]>>3| buff[14]<<5)             & mask);
        channel[10] = ((buff[14]>>6| buff[15]<<2| buff[16]<<10)& mask);
        channel[11] = ((buff[16]>>1| buff[17]<<7)             & mask);
        channel[12] = ((buff[17]>>4| buff[18]<<4)             & mask);
        channel[13] = ((buff[18]>>7| buff[19]<<1| buff[20]<<9) & mask);
        channel[14] = ((buff[20]>>2| buff[21]<<6)             & mask);
        channel[15] = ((buff[21]>>5| buff[22]<<3)             & mask);
}

// to check parity byte
int parity_checker(int parity)
{
	uint8_t failsafe, frame_lost, bit0, bit1, ct;

	// extracting first two bit of parity byte
	bit0 = parity>>7;
	bit1 = (parity>>6)<<1;

	if (bit0 == 0 && bit1 == 0)
		return 0;

	else
		return 1;
}

int main() {

	// array to store sbus packet  
	uint16_t buffer[25];

	// flag to check parity byte
	int flag = 0;

	// pointer to file 
	FILE* data;

	// opening file 
	data = fopen("sbus_data", "rb");

	while(1) 
	{

		// taking buffer input
		for(int i=0; i<26; i++)
		{
			fread(buffer, sizeof(uint16_t), 1, data);
		
		}

		if (buffer[0] == start) 
		{	
			flag = parity_checker(buffer[24]);

			if(flag == 0)
				parse_buffer(buffer);

			if (buffer[25] == end)
				continue;
		}
	}
	
	return 0;

}
