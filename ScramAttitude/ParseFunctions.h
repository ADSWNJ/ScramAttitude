// ==============================================================
//
//	Parse Functions
//	===============
//
//	Copyright (C) 2014-2017	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See ParseFunctions.cpp
//
// ==============================================================

#ifndef __ParseFunctins
#define __ParseFunctins
#include <stdio.h>
#include <exception>
#include <stdarg.h>

class ParseException : public std::exception {
public:
  ParseException();
  ParseException(const char *msg, ...);
  const char *getMsg();
private:
  char msg[256];
};

//
// ParseQuotedString:
// Pull out a quoted string from the buffer pointer to by bp into ret
// After the function, ret points to the quoted string (quotes stripped), and bp points to the rest of the string
// Note 1 ... the buffer is modified (nuls added to terminate strings). 
// Note 2 ... escaped characters are not handled (e.g. \" terminates the string and leaves a \). 
// Return status reflects whether the task was successful. 
//
  bool ParseQuotedString(char **bp, char **ret, bool throwPlease=false);
//
// ParseString:
// Same as ParseQuotedString except does not want quotes and stops at the first whitespace
//
  bool ParseString(char **bp, char **ret, bool throwPlease = false);
// 
// ParseDouble:
// Same as ParseString except it pulls out a double 
//
  bool ParseDouble(char **bp, double *ret, bool throwPlease = false);
// 
// ParseDoubleArray:
// Same as ParseDouble except it parses out 'count' numbers
//
  bool ParseDoubleArray(char **bp, double *ret, int count, bool throwPlease = false);

// 
// ParseJustDoubleArray:
// Same as ParseDoubleArray except it insists on no trailing data
//
  bool ParseJustDoubleArray(char **bp, double *ret, int count, bool throwPlease = false);

  // 
// ParseInt:
// Same as ParseString except it pulls out an integer
//
  bool ParseInt(char **bp, int *ret, bool throwPlease = false);

// 
// ParseBool:
// Same as ParseString except it pulls out a true or false
//
  bool ParseBool(char **bp, bool *ret, bool throwPlease = false);
//
// ParseWhiteSpace:
// Skips leading space, tabs or comments (; found in whitespace parse)
// Returns false if all whitespace
//
  bool ParseWhiteSpace(char **bp);

  //
  // ParseStringFromList:
  // Parses a string, and looks for it on a list of tokens. If found, returns the token position, elase returns false (or throws exception if requested). 
  //
  bool ParseStringFomList(char **bp, int *ret, const char **tokList, const int tokCount, const bool throwPlease = false);
  #endif // __ParseFunctions

