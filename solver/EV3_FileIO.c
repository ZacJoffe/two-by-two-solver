/**********************************************************************
Filename: EV3_FileIO.c
Date: November 7, 2016
File Version: 1.0

This file selects the EV3 version of the ASCII conversion functions 
defined in PC_FileIO.c so that files read or written on a PC.
Whitespace (space, tab, enter) is needed between values.

The functions available are:
bool openWritePC(TFileHandle & fout, char* name);
bool openWritePC(TFileHandle & fout, char* name, word fileSize); // file size can be specified for NXT
bool openReadPC(TFileHandle & fin, char* name);
bool closeFilePC(TFileHandle & fileHandle);
bool writeCharPC(TFileHandle & fout, byte charmsg);
bool writeEndlPC(TFileHandle & fout);
bool writeTextPC(TFileHandle & fout, string const & textmsg);
bool writeLongPC(TFileHandle & fout, long number);
bool writeFloatPC(TFileHandle & fout, string const & numFormat, float number);
bool writeFloatPC(TFileHandle & fout, float number);
bool readCharPC(TFileHandle & fin, char & charmsg);
bool readTextPC(TFileHandle & fin, string & result); // a maximum of 20 characters can be read
bool readIntPC(TFileHandle & fin, int & number);
bool readFloatPC(TFileHandle & fin, float & number);

History
Ver  Date       Comment
1.0  Nov  7/16  original release

**********************************************************************/
#pragma SystemFile

// select EV3 set of functions
#define _EV3FILEIO 1

#include "PC_FileIO.c"
/**********************************************************************
Copyright(c) 2015-2016 C.C.W. Hulls, P.Eng.
Students, staff, and faculty members at the University of Waterloo
are granted a non-exclusive right to copy, modify, or use this
software for non-commercial teaching, learning, and research purposes
provided the author(s) are acknowledged except for as noted below.
**********************************************************************/
