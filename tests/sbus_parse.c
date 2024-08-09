#include <stdio.h>

// start and end byte values for s bus
const int start[] = 00001111; // 0x0F
const int end[] = 00000000; // 0x00

// creating 11 bit channel
int *parse_bufffer(int buff[]) { 
		
	# to store resultant channel binary value 
	int channel[16];

	# masking byte shiftings bits (value in hexa '0x07FF')
	int mask = 0000011111111111;

	// creating channels 
        channel[0]  = ((buff[1] | buff[2]<<8)                 & mask);
        channel[1]  = ((buff[2]>>3 | buff[3]<<5)              & mask);
        channel[2]  = ((buff[3]>>6 | buff[4]<<2 | buf[5]<<10) & mask);
        channel[3]  = ((buff[5]>>1 | buff[6]<<7)              & mask);
        channel[4]  = ((buff[6]>>4 | buff[7]<<4)              & mask);
        channel[5]  = ((buff[7]>>7 | buff[8]<<1 | buf[9]<<9)  & mask);
        channel[6]  = ((buff[9]>>2 | buff[10]<<6)             & mask);
        channel[7]  = ((buff[10]>>5| buff[11]<<3)             & mask);
        channel[8]  = ((buff[12]   | buff[13]<<8)             & mask);
        channel[9]  = ((buff[13]>>3| buff[14]<<5)             & mask);
        channel[10] = ((buff[14]>>6| buff[15]<<2| buf[16]<<10)& mask);
        channel[11] = ((buff[16]>>1| buff[17]<<7)             & mask);
        channel[12] = ((buff[17]>>4| buff[18]<<4)             & mask);
        channel[13] = ((buff[18]>>7| buff[19]<<1| buf[20]<<9) & mask);
        channel[14] = ((buff[20]>>2| buff[21]<<6)             & mask);
        channel[15] = ((buff[21]>>5| buff[22]<<3)             & mask);

	return channel;
}

// to check parity byte
int parity_checker(int parity)
{
	int failsafe, frame_lost, bit0, bit1, ct;

	// extracting first two bit of parity byte
	bin0 = parity>>7;
	bin1 = (parity>>6)<<1;

	if (bin0 == 0 && bin1 == 0)
	{
		return 0;
	}

	else (bin0 == 1)
		return 1;
}

int main() {

	# array to store sbus packet in hexa-decimal  
	int bufffer[25];

	# to store channels
	int *ch;

	# flag to check parity byte
	int flag = 0;

	# taking bufffer input
	for(int i=0; i<26; i++)
	{
		cin>> bufffer[i];
	}

	// reading buffer
	while(True) 
	{
		if (buffer[0] == start) 
		{	
			flag = parity_checker(buffer[24]);

			if(flag == 0)
				ch = parse_buffer(buffer[])

			if (buffer[25] == end)
				continue;
		}
	}
	
	return 0;
}
