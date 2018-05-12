/**
 * This file contains the list of functions defined in parsing.cpp
 */

bool us_getData(int sock,char* data,unsigned int data_ln);
int us_get_word(char* data, int pos, char* word);
// get the availabe data type and message body pointer
int us_get_type(char* data, char** ppBody);
// get the first data segment starts with name from the position pos.
int us_get_segmentByName(char* data, int pos, char* name, char* segment);
// get the name value pair in a segment.
int us_get_value(char* segment, char* name, char* value);
// get the first name value pair in a message.
int us_get_value2(char* data, char* name, char* value);

