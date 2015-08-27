/////////////////////////////////////////////////////////////////////////////////////////
//	Functions for GPS Operation
//	Coordinate Data Format : 1Byte(Degree) and 2Bytes(Minute)
/////////////////////////////////////////////////////////////////////////////////////////

/* Headers */
#include "80c32x2.h"
#include "type.h"
#include "uart.h"                             
#include "gps.h"
#include "usb.h"
#include "sl11.h"
#include "sbit.h"
#include "flash.h"
#include "ioport.h"

/* Debug Information */
#define DEBUG_ALL		FALSE
#define DEBUG_GPS		DEBUG_ALL
#define DEBUG_PARSE_POSITION	DEBUG_ALL
#define DEBUG_INTERVAL		DEBUG_ALL
#define DEBUG_IS_VALID_RANGE	DEBUG_ALL
#define DEBUG_BIN_SEARCH	DEBUG_ALL
#define DEBUG_TABLE_SEARCH	DEBUG_ALL
#define DEBUG_EDIT_OLD		DEBUG_ALL
#define DEBUG_DISTANCE		FALSE
#define DEBUG_GPS_VALID		FALSE

/* Position from '$' */
#define HEADER_ORDER		(EP2Buf)
#define DATE_ORDER		(EP2Buf + 7)	//53
#define TIME_ORDER		(EP2Buf + 14)	//46
#define NORTH_ORDER             (EP2Buf + 21)
#define NORTH_MOD_ORDER         (EP2Buf + 26)
#define EAST_ORDER		(EP2Buf + 32)
#define EAST_MOD_ORDER		(EP2Buf + 38)
#define DILUTION_ORDER          (EP2Buf + 49)

/* Length of Serial Stream */
#define HEADER_LEN		5
#define DATE_LEN		6
#define TIME_LEN		6
#define N_LEN			4
#define N_MOD_LEN		3
#define E_LEN			5
#define E_MOD_LEN		3
#define DILUTION_LEN		3
#define DEG_SIZE		2

/* The Others : Definition and it`s dependant global variable */
#define MINI_BUFFER_SIZE	18		//16+2 : for use LCD module buffer
#define METER_PER_MILLIMINUTE	1		//unused:for definition
#define MILLIMINUTE_500M	500             //MILLIMINUTE_500M = METER_PER_MILLIMINUTE * 500M
#define SCALE			4               //unused:for definition
#define SCALING_FACTOR		2		//SCALING_FACTOR = SCALE / 2
BYTE global_control_flag=0;
BYTE mini_buffer[MINI_BUFFER_SIZE];
//BYTE GPSdate[3];				//binary hex 3
BYTE GPStime[3];          			//binary hex 3

BYTE binPos[6];					//binary hex 6 : byte-word-byte-word
#define N_POS		0x00			//binary hex format
#define N_MOD_POS	0x01
#define E_POS		0x03
#define E_MOD_POS	0x04

BYTE hor_dilution;
WORD top=0;
WORD bottom=0;
#define VALID_RANGE_KEY_MAX	4
WORD validRangeKey[VALID_RANGE_KEY_MAX];
BYTE distance=0;

/* Structures */
typedef struct structFtbHeader_tag{
	BYTE header[4];
	WORD unit_number;
	WORD unit_size;
	WORD full_size;
}structFtbHeader;

typedef struct structFlashTable_tag{
	BYTE north_deg;
	WORD north_min;
	BYTE east_deg;
	WORD east_min;
}structFlashTable;

structFlashTable xdata *FlashTable = (structFlashTable *)pFTBDataBase;	//sorted array data
structFlashTable xdata *UserTable = (structFlashTable *)pUserDataBase;  //user added data

/* Contant */
code const BYTE header_order=HEADER_ORDER;
code const BYTE date_order=DATE_ORDER;
code const BYTE time_order=TIME_ORDER;
code const BYTE north_order=NORTH_ORDER;
code const BYTE north_mod_order=NORTH_MOD_ORDER;
code const BYTE east_order=EAST_ORDER;
code const BYTE east_mod_order=EAST_MOD_ORDER;
code const BYTE dilution_order=DILUTION_ORDER;
code const WORD DecSquareTable[5]={1,10,100,1000,10000};



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	<Functions Section>
//
//	"MISCELLANEOUS" 
//	Purpose : Miscellaneous Functions for Stream and Data Type
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                       
				//ASCII to bin nibble
BYTE Asc2HexNib(char ch)        
{
	if(ch >= 'A' && ch <= 'F')	ch = ch-'A'+10;
	else if(ch >= 'a' && ch <= 'f') ch = ch-'a'+10;
	else if(ch >= '0' && ch <= '9')	ch = ch-'0';
	else ch=0;
	
return ch;
}
				//ASCII to bin WORD
WORD DecAsc2BinWord(BYTE *asc, BYTE asc_length)
{
	BYTE i;
	WORD bin_result=0;

	asc_length--;

	for(i=0;i<=asc_length;i++){
		bin_result += (WORD)( (DecSquareTable[i])*Asc2HexNib(asc[asc_length-i]) );
	}

return bin_result;
}
				//Hex ASCII to bin WORD
WORD HexAsc2BinWord(BYTE *asc, BYTE asc_length)
{
	BYTE i;
	WORD bin_result=0;

	for(i=0;i<asc_length;i++){
		bin_result <<= 4;
		bin_result = (WORD)(bin_result | Asc2HexNib(asc[i]));
	}

return bin_result;
}
				//bin to ASCII
BYTE *DecWord2DecAscStr(WORD number, BYTE *dest, BYTE asc_length)
{
	BYTE i=asc_length;

	dest[i]=0;
	while(i--){
		dest[asc_length-i-1] = ( HexAscTable[number/(DecSquareTable[i])] );
		number %= DecSquareTable[i];
	}

return dest;
}
				//Compare string
BYTE isSame(BYTE *str1, BYTE *str2, BYTE len)
{
	while(len--){
		if(str1[len] != str2[len])	return FALSE;
	}
return TRUE;
}
				//memcopy
BYTE *copyBuffer(BYTE *src, BYTE *dest, BYTE len)
{
	while(len--){
		dest[len] = src[len];
	}

return dest;
}

void Delay(int value)
{
	int i, j;

	for( i=0; i<value; i++)
	{ 
		for( j=0; j<625; j++);
	}
}

void Delay_s(int value)
{
	while(value--);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	<Functions Section>
//
//	"GPS Current Coordinate and Information" 
//	Purpose : Parsing stream information from GPS module
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#define LED1ON		0
#define LED1OFF		1
				//Extracting Binary Coordinate from ASCII Stream
void parsePosition(void)
{       
	if(SL11Read(HEADER_ORDER) != '$'){		//Verify Valid Data Set
		clear(GPS_VALID)
		return;
	}
      	EA=0;
							//3 Bytes for GPS Date
/*	SL11BufRead(date_order, mini_buffer, 2);
	GPSdate[0] = (BYTE)DecAsc2BinWord(mini_buffer, 2);
	SL11BufRead(date_order+2, mini_buffer, 2);
	GPSdate[1] = (BYTE)DecAsc2BinWord(mini_buffer, 2);
	SL11BufRead(date_order+4, mini_buffer, 2);
	GPSdate[2] = (BYTE)DecAsc2BinWord(mini_buffer, 2);
*/
							//3 Bytes for GPS Time + 9hour
	SL11BufRead(time_order, mini_buffer, 2);
	GPStime[0] = (BYTE)DecAsc2BinWord(mini_buffer, 2);
	SL11BufRead(time_order+2, mini_buffer, 2);
	GPStime[1] = (BYTE)DecAsc2BinWord(mini_buffer, 2);
	SL11BufRead(time_order+4, mini_buffer, 2);
	GPStime[2] = (BYTE)DecAsc2BinWord(mini_buffer, 2);
	
	GPStime[0] += 9;
	if(GPStime[0] >= 24){
		GPStime[0] = 24-GPStime[1];
	}
							//One Byte for North theta Degree
	SL11BufRead(north_order, mini_buffer, N_LEN-DEG_SIZE);
	bIO(binPos+N_POS) = (BYTE)DecAsc2BinWord(mini_buffer, N_LEN-DEG_SIZE);

							//Two Byte for North theta Minute
	SL11BufRead(north_mod_order-3, mini_buffer, DEG_SIZE);
	SL11BufRead(north_mod_order, mini_buffer+DEG_SIZE, N_MOD_LEN);
	wIO(binPos+N_MOD_POS) = DecAsc2BinWord(mini_buffer, N_MOD_LEN+DEG_SIZE);

							//One Byte for East pi Degree
	SL11BufRead(east_order, mini_buffer, E_LEN-DEG_SIZE);
	bIO(binPos+E_POS) = (BYTE)DecAsc2BinWord(mini_buffer, E_LEN-DEG_SIZE);
                                                               
							//Two Byte for East pi Minute
	SL11BufRead(east_mod_order-3, mini_buffer, DEG_SIZE);
	SL11BufRead(east_mod_order, mini_buffer+DEG_SIZE, E_MOD_LEN);
	wIO(binPos+E_MOD_POS) = DecAsc2BinWord(mini_buffer, E_MOD_LEN+DEG_SIZE);

							//One Byte for Horizontal Dilution
	SL11BufRead(dilution_order, mini_buffer, DILUTION_LEN);
	hor_dilution = (BYTE)DecAsc2BinWord(mini_buffer, DILUTION_LEN);

       	EA=1;

	#if DEBUG_PARSE_POSITION
	puts("\r\n North : ");
	DecWord2DecAscStr((WORD)bIO(binPos+N_POS), mini_buffer, N_LEN-DEG_SIZE);
	puts(mini_buffer);putch('.');
	DecWord2DecAscStr(wIO(binPos+N_MOD_POS), mini_buffer, N_MOD_LEN+DEG_SIZE);
	puts(mini_buffer);
	puts("     East : ");
	DecWord2DecAscStr((WORD)bIO(binPos+E_POS), mini_buffer, E_LEN-DEG_SIZE);
	puts(mini_buffer);putch('.');
	DecWord2DecAscStr(wIO(binPos+E_MOD_POS), mini_buffer, E_MOD_LEN+DEG_SIZE);
	puts(mini_buffer);
	#endif

#if DEBUG_GPS_VALID
	hor_dilution = 10;
#endif
	if(hor_dilution >= 99){
		sbitLED1 = LED1OFF;
		clear(GPS_VALID)				//Invalid GPS Signal Detected.
	}
	else{
		sbitLED1 = LED1ON;
		set(GPS_VALID)					//Valid GPS signal
	}
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	<Functions Section>
//
//	"Flash Main Table" 
//	Purpose : Access Flash Camera Table for Search Nearest Entry
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void parseFlashTable(void)
{
	if(isSame(&cFTBHeader, "*FTB", 4)){
		#if DEBUG_GPS
		puts("\r\nFTB Header Scanned");
		#endif
		if( ((cFTBUnitNumber*cFTBUnitSize)+FTB_HEADER_SIZE) == cFTBFullSize ){
			#if DEBUG_GPS
			puts("\r\nValid Flash Table Scanned..");
			#endif
			set(FTB_VALID)                          //Flash Table Valid
			return;
		}
	}
	puts("\r\nError : Invalid Flash Table Format!!");
	clear(FTB_VALID)                                        //Invalid Flash Table
}

#define	FIRST		1
#define SECOND		2
#define SAME		0

#define BASE_POS	0
#define MOD_POS		1
								//Compare Coordinate distance
BYTE whatIsBigger(BYTE *tablePos, BYTE *curPos)
{
		if(bIO(tablePos+BASE_POS) > bIO(curPos+BASE_POS))		return FIRST;
		else if(bIO(tablePos+BASE_POS) < bIO(curPos+BASE_POS))		return SECOND;
		else{
			if(wIO(tablePos+MOD_POS) > wIO(curPos+MOD_POS))		return FIRST;
			else if(wIO(tablePos+MOD_POS) < wIO(curPos+MOD_POS))  	return SECOND;
		}		
return SAME;
}
								//Get Coordinate Distance
void intervalBetween(BYTE *pos1, BYTE *pos2, BYTE *result)
{                                                                       
	if(bIO(pos1+BASE_POS) > bIO(pos2+BASE_POS)){
		if(bIO(pos1+MOD_POS) >= bIO(pos2+MOD_POS)){
			*(BYTE *)(result+BASE_POS) = bIO(pos1+BASE_POS) - bIO(pos2+BASE_POS);
			*(WORD *)(result+MOD_POS) = bIO(pos1+MOD_POS) - bIO(pos2+MOD_POS);
		}
		else{
			*(BYTE *)(result+BASE_POS) = bIO(pos1+BASE_POS) - bIO(pos2+BASE_POS) - 1;
			*(WORD *)(result+MOD_POS) = 60000 - (bIO(pos2+MOD_POS) - bIO(pos1+MOD_POS));
		}		
	}
	else if(bIO(pos1+BASE_POS) < bIO(pos2+BASE_POS)){
		if(bIO(pos1+MOD_POS) <= bIO(pos2+MOD_POS)){
			*(BYTE *)(result+BASE_POS) = bIO(pos2+BASE_POS) - bIO(pos1+BASE_POS);
			*(WORD *)(result+MOD_POS) = bIO(pos2+MOD_POS) - bIO(pos1+MOD_POS);
		}
		else{
			*(BYTE *)(result+BASE_POS) = bIO(pos2+BASE_POS) - bIO(pos1+BASE_POS) - 1;
			*(WORD *)(result+MOD_POS) = 60000 - (bIO(pos1+MOD_POS) - bIO(pos2+MOD_POS));
		}		
	}
	else{
		*(BYTE *)(result+BASE_POS) = 0;
		if(wIO(pos1+MOD_POS) >= wIO(pos2+MOD_POS)){
			*(WORD *)(result+MOD_POS) = (wIO(pos1+MOD_POS) - wIO(pos2+MOD_POS));
		}
		else{
			*(WORD *)(result+MOD_POS) = (wIO(pos2+MOD_POS) - wIO(pos1+MOD_POS));
		}		
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
//	Binary Search
//      Author : Lee,Yongho	
//      Purpose : Searching for the nearest two data from target coordinate
///////////////////////////////////////////////////////////////////////////////////////////////////////
WORD binarySearch(void)        			// Binary Searching to Find nearest Key from Current position
{
	WORD center=0;
	BYTE result_bottom[3];
	BYTE result_top[3];

	top=0;
	bottom=cFTBUnitNumber-1;						
	#if DEBUG_BIN_SEARCH
	puts("\r\nBinary Searching:");
	#endif
	while(1){								
		#if DEBUG_BIN_SEARCH
		putch('>');
		#endif
		center=(top+bottom)/2;
		switch(whatIsBigger((BYTE *)&FlashTable[center], binPos)){
			case FIRST  :
				bottom = center;				
				break;
			case SECOND :
				top = center;
				break;
			case SAME   :
				return center;
				break;
			default  :
				break;
		}
		if((bottom-top) <= 1){				//For the last two data
			intervalBetween((BYTE *)&FlashTable[top], binPos, result_top);		//Get direct distance between two coordinate
			intervalBetween((BYTE *)&FlashTable[bottom], binPos, result_bottom);

			switch(whatIsBigger(result_top,result_bottom)){
				case FIRST :     		//if top is far from target than bottom
					center = bottom;
					break;
				case SECOND :			//if bottom is far from target than top
					center = top;
					break;
				case SAME :
					center = top;
					break;
				default :
					break;
			}
			break;
		}
	
	}
	#if DEBUG_BIN_SEARCH
	puts("\r\nU:");puthexw(center);puts("-NDeg:");puthex(FlashTable[center].north_deg);
	puts("-NMin:");puthexw(FlashTable[center].north_min);
	puts("-EDeg:");puthex(FlashTable[center].east_deg);
	puts("-EMin:");puthexw(FlashTable[center].east_min);
	#endif
return center;
}
								//Assure Valid Range
BYTE IsValidRange(WORD key, BYTE target, structFlashTable *TargetTable)
{
	BYTE interval[3];
	BYTE boundary[3];

	bIO(boundary+BASE_POS) = 0;
	wIO(boundary+MOD_POS) = MILLIMINUTE_500M;

	if(target)
		intervalBetween((BYTE *)(&(TargetTable[key].east_deg)), binPos+target, interval);	
	else
		intervalBetween((BYTE *)(&TargetTable[key]), binPos, interval);

	switch(whatIsBigger(boundary, interval)){
				case FIRST :     		//Nearest Key is in Valid range
					#if DEBUG_IS_VALID_RANGE
					puts("\r\n0x");
					puthexw(key);
					puts(" In Range-Interval:");
					puthexw(wIO(interval+MOD_POS));
					#endif
					return TRUE;
					break;
				case SECOND :			//Out of Range
				case SAME :			//Out of Range
					#if DEBUG_IS_VALID_RANGE
					puts("\r\nOut of Range-Interval:");
					puthexw(wIO(interval+MOD_POS));
					#endif
					return FALSE;
					break;
				default :
					break;
	}
return FALSE;
}
								//Extracting North valid Range top and bottom
void ScrollSearch(WORD key)
{
	WORD count=key;
	
	while(1){
		count++;                        						
		if(!IsValidRange(count, N_POS, FlashTable) || (count >= cFTBUnitNumber)){
			bottom = count-1;	
			break;
		}
	}

	count=key;
	while(1){
		count--;                                                                    	
		if(!IsValidRange(count, N_POS, FlashTable) || (count >= cFTBUnitNumber)){
			top = count+1;
			break;
		}
	}
}
								//Searching Valid East Range
BYTE EastMinuteSearch(void)
{
	WORD count;
	BYTE finalCount=0;
	for(count=top;count<=bottom;count++){
	    	if(IsValidRange(count, E_POS, FlashTable)){
			if(finalCount>=VALID_RANGE_KEY_MAX){
				#if DEBUG_GPS
				puts("\r\nValid Data Flushed!!");
				#endif
				return finalCount;
			}
			validRangeKey[finalCount++]=count;
		}
	}
return finalCount;
}

////////////////////////////////////////////////////////////////////////////////////
//	Insertion Sort
//      Author : Lee,Yongho	
//      Purpose : 53726->55726->35726->35776->35576->33576->23576->23577->23567
//		First, Sorting small pieces(EX. 2 keys) from front of table.
//		then, Insert Next Key into there position in sorted small pieces.
//		These Small pieces Bigger and Bigger in sorting loop.
//		While Inserting, Each Sorted key move to its around position 
//		until the insert key find its position.
////////////////////////////////////////////////////////////////////////////////////

void insertion(WORD *a, BYTE N) 
{ 

	BYTE i, j;
	WORD v, key; 
	for (i=1; i<N; i++){ 
		v = a[i];
		key = validRangeKey[i];
		j = i; 
		while(a[j-1] > v){
			a[j] = a[j-1];
			validRangeKey[j] = validRangeKey[j-1];
			if(--j <= 0) break; 
		} 

		a[j]=v;
		validRangeKey[j]=key;
	}
} 

#define SCALED_LENGTH_500M	15625	// = sqr{(500 / 4[scale]}
#define SCALED_LENGTH_300M	5625    // = sqr{(300 / 4[scale]}
#define SCALED_LENGTH_100M	500	// 625= sqr{(100 / 4[scale]}

WORD LabelDistance(WORD DirectLength)
{
	if(SCALED_LENGTH_100M > DirectLength){
		#if DEBUG_TABLE_SEARCH
		puts("in 100M");
		#endif
		return DISTANCE_100M;
	}
	else if(SCALED_LENGTH_300M > DirectLength){
		#if DEBUG_TABLE_SEARCH
		puts("in 300M");
		#endif
		return DISTANCE_300M;
	}
	else if(SCALED_LENGTH_500M > DirectLength){
		#if DEBUG_TABLE_SEARCH
		puts("in 500M");
		#endif
		return DISTANCE_500M;
	}
	else{
		#if DEBUG_TABLE_SEARCH
		puts("\r\nExceptional Range!!");
		#endif
	 	return 0;
	}
}

////////////////////////////////////////////////////////////////////////////////////
//	Function GetDirectLength
//	Purpose : Extract Target table Entry and 
//		  compute Scaled Direct Length from current Coordinate
//		  In tri-angle closed loop has one 90degree corner 
//		  Sqr(Z) = Sqr(X) + Sqr(Y)
//			Z, X, Y is each side of loop, Sqr() is Square 2
////////////////////////////////////////////////////////////////////////////////////
WORD GetDirectLength(WORD key, structFlashTable *TargetTable)
{
	BYTE TempAbsCoordinate[6];

	intervalBetween((BYTE *)(&TargetTable[key]), binPos, TempAbsCoordinate);			//North
	wIO(TempAbsCoordinate+N_MOD_POS) >>= SCALING_FACTOR;						//Reduce Scale North : Y
	wIO(TempAbsCoordinate+N_MOD_POS) *= wIO(TempAbsCoordinate+N_MOD_POS);				//Square Y 
	intervalBetween((BYTE *)(&(TargetTable[key].east_deg)), binPos+E_POS, TempAbsCoordinate+E_POS);	//East
	wIO(TempAbsCoordinate+E_MOD_POS) >>= SCALING_FACTOR;						//Reduce Scale East  : X
	wIO(TempAbsCoordinate+E_MOD_POS) *= wIO(TempAbsCoordinate+E_MOD_POS);		    		//Square X
	return (wIO(TempAbsCoordinate+N_MOD_POS)+wIO(TempAbsCoordinate+E_MOD_POS));			//length  = Sqr(X)+Sqr(Y)
}

////////////////////////////////////////////////////////////////////////////////////
//	Function UserSpaceSearch
//	Purpose : Contiguous Search User Space Coordinate Entry
////////////////////////////////////////////////////////////////////////////////////
BYTE UserSpaceSearch(WORD *DirectLength)
{
	WORD count;
	
	if(!test(USER_SPACE_VALID))	return FALSE;
	for(count=0;count<cUserUnitNumber;count++){
		if(IsValidRange(count, N_POS, UserTable)){
			if(IsValidRange(count, E_POS, UserTable)){
				*DirectLength = GetDirectLength(count, UserTable);
				return TRUE;
			}
		}
	}

return FALSE;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	Table Search Routine
//	Author : Lee,Yongho	
//	Purpose : Search Coordinate Key in Flash Table and direct path length that have Valid detection range 
//		Step1 : Search for Nearest Coordinate key and verify whether the path length from current 
//			coordinate is in valid range
//		Step2 : Search another key that have valid range Coordinate scrolling up and down from current
//			coordinate key
//		Step3 : Filtering coordinate key with current East-wide valid range
//		Step4 : Searching Key that have Shortest Direct path from current coordinate 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TableSearch(void)
{
	WORD bin_search_result_key=0;
	WORD number_matched_unit=0;		
	WORD DirectLength[VALID_RANGE_KEY_MAX];
	BYTE i;

	bin_search_result_key = binarySearch();			// Step1
	#if DEBUG_TABLE_SEARCH
	puts("\r\nNear:");
	puthexw(bin_search_result_key);
	#endif
	
	if(IsValidRange(bin_search_result_key, N_POS, FlashTable)){                
		ScrollSearch(bin_search_result_key);            // Step2
		number_matched_unit = bottom - top + 1;		// Key roundnumber have only valid Range in North
		#if DEBUG_TABLE_SEARCH
		puts("\r\nNorthMatch:");
		puthexw(number_matched_unit);
		puts(" top:");puthexw(top);
		puts(" bottom:");puthexw(bottom);
		#endif
		number_matched_unit = EastMinuteSearch();	// Step3 : Key roundnumber have valid range both N & E
		if(number_matched_unit){
			#if DEBUG_TABLE_SEARCH
			puts("\r\nEastMatch:");puthexw(number_matched_unit);
			for(i=0;i<number_matched_unit;i++){
				putch(':');puthexw(validRangeKey[i]);
			}
			#endif
								// Step4
			for(i=0;i<number_matched_unit;i++){	           		//Extract each Direct Path Length from current coordinate 
				DirectLength[i] = GetDirectLength(validRangeKey[i], FlashTable);
			}
			if(number_matched_unit > 1){
				insertion(DirectLength, number_matched_unit);			//Finding Key Shortest Direct Path Length
			}
			// The Shortest Coordinate : FlashTable[validRangeKey[0]] ,Direct path length : DirectLength[0]
			#if DEBUG_TABLE_SEARCH
			puts("\r\nThe Shortest Key:");
			for(i=0;i<number_matched_unit;i++){
				puthexw(validRangeKey[i]);putch(':');
				puthexw(DirectLength[i]);putch('-');
			}
			#endif
			//USER SPACE SEARCH ROUTINE
			if(UserSpaceSearch(&DirectLength[1])){	//Read Direct length from UserSpace Valid Coord. to current Coord.
				if(DirectLength[0] > DirectLength[1]){	         	//Compare User Coord. with Cam. Coord.
#if !DEBUG_DISTANCE
					distance = LabelDistance(DirectLength[1]);
#endif
					set2(TARGET_IS_USERDB);
				}
				else{
#if !DEBUG_DISTANCE
					distance = LabelDistance(DirectLength[0]);
#endif
					clear2(TARGET_IS_USERDB);
				}
			}
			else{
#if !DEBUG_DISTANCE
				distance = LabelDistance(DirectLength[0]);
#endif
				clear2(TARGET_IS_USERDB);
			}

#if DEBUG_DISTANCE
			puts("\r\nIn");puthex(distance);
			if(test2(TARGET_IS_PASSED))	puts("\r\nPassed");
			else 				puts("\r\nBefore");
#endif
			return;
		}
		else{
			#if DEBUG_TABLE_SEARCH
			puts("\r\nNo valid East key:");puthexw(number_matched_unit);
			#endif
		}

	}
	
	if(UserSpaceSearch(&DirectLength[0])){
#if !DEBUG_DISTANCE
		distance = LabelDistance(DirectLength[0]);
#endif
		set2(TARGET_IS_USERDB);
	}
	else distance=0;

#if DEBUG_DISTANCE
	puts("\r\nOut");puthex(distance);
	if(test2(TARGET_IS_PASSED))	puts("\r\nPassed");
	else 				puts("\r\nBefore");
#endif
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	<Functions Section>
//
//	"User Table Access"
//	Purpose : Access User Table Entry. Write and Read to modify
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void parseUserSpace(void)
{
	if(isSame(&cUserHeader, "*FTB", 4)){
		#if DEBUG_GPS
		puts("\r\nUser Space Header Scanning");
		#endif
		if( ((cUserUnitNumber*cUserUnitSize)+FTB_HEADER_SIZE) == cUserFullSize ){
			#if DEBUG_GPS
			puts("\r\nValid User Space Table Scanned..");
			#endif
			set(USER_SPACE_VALID)                          //Flash Table Valid
			return;
		}
	}
	#if DEBUG_GPS
	puts("\r\nError : Invalid USER SPACE Flash Table Format!!");
	#endif
	clear(USER_SPACE_VALID)                                        //Invalid Flash Table
}

////////////////////////////////////////////////////////////////////////////////////
//	Function CreateNewUserTable & Function EditOldUserTable
//	Purpose : Write Current Coordinate Entry To User Space of Flash Memory
////////////////////////////////////////////////////////////////////////////////////
void CreateNewUserTable(void)
{
	BYTE *pStr = mini_buffer;

	puts("\r\nCreate New User Table");

	clear(FLASH_SECTOR)
	copyBuffer("*FTB", pStr, 4);			//Header Mark "*FTB" insert
	pStr+=4;
	*pStr++ = 0x00; *pStr++ = 0x01;
	*pStr++ = 0x00; *pStr++ = 0x06;
	*pStr++ = 0x00; *pStr++ = 0x10;
	copyBuffer(binPos, pStr, 6);
	pStr+=6;
	SL11BufWrite(EP1Buf, mini_buffer, pStr-mini_buffer);
	flashProgram(&cUserBase, pStr-mini_buffer);	
}

//Edit Old User Table Methodology
//#1. Read and mapping old user-ftb to structFtbHeader
//#2. Copy first 64Byte of old user-ftb to SL11 buffer and modifiy the buffer.
//#3. Copy modified user-ftb to temp-user-ftb from SL11 buffer with direct flash programming : 64Bytes copy, non protection flags
//#4. Copy all the user-ftb-data from user-ftb to SL11 buffer 
//    and add new coordinate to mini_buffer and each time backup from buffer to temp-user-ftb
//#5. Copy user-ftb data from temp-user-ftb space to user-ftb-space with protect enable programming
//I.E.	once{ user-ftb -> SL11buffer(modify) -> temp-user-ftb } 
//	repeat{ user-ftb -> temp-user-ftb } 
//	once{ new-coordinate -> mini_buffer -> temp-user-ftb } 
//	repeat{ temp-user-ftb -> user-ftb }

#define MAX_ITEM_ENTRY		100
#define MAX_BYTES_DOUBLE	128
#define FULL			2

void EditOldUserTable(void)
{
	BYTE *pStr = mini_buffer;				
	//#1 operation : mapping
	structFtbHeader *psHeader = &cUserBase;			//start of user space table header
	structFtbHeader *psTempHeader = &cFTempSpace;
	BYTE *pUserFile = &cUserBase;			//start of user space data 
	BYTE *pTempSpace = &cFTempSpace;
	if(psHeader->unit_number >= MAX_ITEM_ENTRY){		//Is User Item Full
		puts("\r\nUnable Add User Coordinate");
		WaitaMinute(1000);
		return;
	}
	#if DEBUG_EDIT_OLD
	puts("\r\nEdit Old User Table");
	#endif
	//#2 operation : modify header
	SL11BufWrite(EP1Buf, pUserFile, MAX_BYTES_DOUBLE);	
	copyBuffer("*FTB", pStr, 4);pStr+=4;			//Header Mark "*FTB" insert
    wIO(pStr) = psHeader->unit_number + 1;pStr+=2;		//modify unit_number
	wIO(pStr) = psHeader->unit_size; pStr+=2;               //modify unit_size
	wIO(pStr) = (psHeader->full_size)+(psHeader->unit_size);//modify full_size ; non increment pointer
	SL11BufWrite(EP1Buf, mini_buffer, 10);
	pStr=mini_buffer;
	wIO(pStr+FULL) = psHeader->full_size;
	//#3 operation : copy header and first 54Bytes to temp-user-ftb
	//#4 operation : repeating copy data space to temp-user-ftb
	clear(FLASH_SECTOR)					//init flash sector
	pUserFile+=MAX_BYTES_DOUBLE;
        while((pUserFile-(&cUserBase)) < wIO(pStr+FULL)){
		#if DEBUG_EDIT_OLD
		puts("\r\n1.Repeat");
		#endif
		flashProgram128(pTempSpace, MAX_BYTES_DOUBLE);
		SL11BufWrite(EP1Buf, pUserFile, MAX_BYTES_DOUBLE);//precopy next page
		pUserFile+=MAX_BYTES_DOUBLE;
		pTempSpace += MAX_BYTES_DOUBLE;
	}
	//if last data page(64Bytes:0.5Sector)
	if( ((pUserFile-(&cUserBase)) - wIO(pStr+FULL)) >= 6 ){//assure minimum space for coordinate
		#if DEBUG_EDIT_OLD
		puts("\r\n2.Normal");
		#endif
		SL11BufWrite(EP1Buf+((wIO(pStr+FULL))%MAX_BYTES_DOUBLE), binPos, 6);
		flashProgram128(pTempSpace, MAX_BYTES_DOUBLE);
		pTempSpace += MAX_BYTES_DOUBLE;      	//destination ptr increament
	}
	else{						//if last 6Bytes spreads on each boundary
		#if DEBUG_EDIT_OLD
		puts("\r\n4.Spread");
		#endif
		wIO(pStr) = (pUserFile-(&cUserBase))-(wIO(pStr+FULL));		//size of first part of spreaded 6Byte
		SL11BufWrite( EP1Buf+(wIO(pStr+FULL)%MAX_BYTES_DOUBLE), binPos, (BYTE)(wIO(pStr)) );
		flashProgram128(pTempSpace, MAX_BYTES_DOUBLE);
		pTempSpace += MAX_BYTES_DOUBLE;      	//Move the real last page data less than 6 Bytes to temp
		pUserFile += MAX_BYTES_DOUBLE;

		SL11BufWrite( EP1Buf, binPos+(BYTE)(wIO(pStr)), 6-(BYTE)(wIO(pStr)) );
		fillSL11Ep1Buf( EP1Buf+(6-(wIO(pStr))), '*', MAX_BYTES_DOUBLE-( 6-(BYTE)(wIO(pStr)) ) );
		flashProgram128(pTempSpace, MAX_BYTES_DOUBLE);
	}
	puthexw(wIO(pStr+FULL));
}


