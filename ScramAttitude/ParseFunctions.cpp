//
// Parse Helper Functions
//
// Purpose ... tokenize strings, pull out doubles, deal with quoted strings, etc
//
// (c) Andrew Stokes (ADSWNJ) 2012-2017
//
// All rights reserved
//
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <exception>


#include "ParseFunctions.h"


ParseException::ParseException() {
  strcpy_s(msg, 256, "Other Parse Exception");
}

ParseException::ParseException(const char *msg_, ...) {
  va_list args;
  va_start(args, msg_);
  vsprintf_s(msg, msg_, args);
};
const char *ParseException::getMsg() { return msg; }

//
// ParseQuotedString:
// Pull out a quoted string from the buffer pointer to by bp into ret
// After the function, ret points to the quoted string (quotes stripped), and bp points to the rest of the string
// Note 1 ... the buffer is modified (nuls added to terminate strings). 
// Note 2 ... escaped characters are not handled (e.g. \" terminates the string and leaves a \). 
// Return status reflects whether the task was successful. 
//
bool ParseQuotedString(char **bp, char **ret, bool throwPlease) {
	char *b = *bp;
	while ((*b==' ')||(*b=='\t')) b++;
  if (*b != '\"') {
    if (throwPlease) throw ParseException("missing start quote on string");
    return false;
  }
	b++;
	*ret = b;
	while ((*b!='\"')&&(*b!='\0')&&(*b!='\n')) b++;
	if (*b!='\"') {
    if (throwPlease) throw ParseException("missing end quote on string");
    return false;
  }
	*b = '\0';
	*bp = b+1;
	return true;
}

//
// ParseString:
// Same as ParseQuotedString except does not want quotes and stops at the first whitespace
//
bool ParseString(char **bp, char **ret, bool throwPlease) {

	char *b = *bp;
	while ((*b==' ')||(*b=='\t')) b++;
  if (*b == ';' || *b == '#' || *b == '\0' || *b == '\n') {
    if (throwPlease) throw ParseException("no string found");
    return false;
  }
	*ret = b;
	while ((*b!='\"')&&(*b!='\0')&&(*b!='\n')&&(*b!=' ')&&(*b!='\'')&&(*b!='\t')) b++;
	if ((*b=='\"')||(*b=='\'')) { // No quotes allowed in string
    if (throwPlease) throw ParseException("quote found in string");
    return false;
  }
	if (*b!='\0') {
	*b = '\0';
	*bp=b+1;
	} else {
	*bp = b;	// if we hit EOL, point to EOL not one beyond
	}
	return true;
}

// 
// ParseDouble:
// Same as ParseString except it pulls out a decimal number
//
bool ParseDouble(char **bp, double *ret, bool throwPlease) {
	char *b = *bp;
	char *t;
	int i;

	while ((*b==' ')||(*b=='\t')) b++;
  if (*b == ';' ||  *b == '#' || *b == '\0' || *b == '\n') { 
    if (throwPlease) throw ParseException("no number found");
    return false;
  }

  t = b;
	i = strspn(t, "+-0123456789e.");
	b = t+i;
	if ((*b!=' ')&&(*b!='\t')&&(*b!='\n')&&(*b!='\0')) {
    if (throwPlease) throw ParseException("non-numeric found in number: (%c)", *b);
    return false;
  } // End of parse must be whitespace
	if (*b!='\0') {
	*b = '\0';
	*bp=b+1;
	} else {
	*bp = b;	// if we hit EOL, point to EOL not one beyond
	}
	if (i==0) {
    if (throwPlease) throw ParseException("no number found");
    return false;
  }
	*ret = atof(t);
	return true;
}

// 
// ParseDoubleArray:
// Same as ParseDouble except it parses out 'count' numbers
//
bool ParseDoubleArray(char **bp, double *ret, int count, bool throwPlease) {
  char *b;
  char *t;
  int i;
  double *retA = ret;
  for (int elem = 0; elem < count; elem++) {
    b = *bp;
    i = 0;
    while ((*b == ' ') || (*b == '\t')) b++;
    if (*b == ';' || *b == '#' || *b == '\0' || *b == '\n') {
      if (throwPlease) throw ParseException("no number found");
      return false;
    }

    t = b;
    i = strspn(t, "+-0123456789e.");
    b = t + i;
    if ((*b != ' ') && (*b != '\t') && (*b != '\n') && (*b != '\0')) {
      if (throwPlease) throw ParseException("non-numeric found in number: (%c)", *b);
      return false;
    } // End of parse must be whitespace
    if (*b != '\0') {
      *b = '\0';
      *bp = b + 1;
    } else {
      *bp = b;	// if we hit EOL, point to EOL not one beyond
    }
    if (i == 0) {
      if (throwPlease) throw ParseException("no number found");
      return false;
    }
    *retA++ = atof(t);
  }
  return true;
}

// 
// ParseJustDoubleArray:
// Same as ParseDoubleArray except it insists on no trailing data
//
bool ParseJustDoubleArray(char **bp, double *ret, int count, bool throwPlease) {
  if (!ParseDoubleArray(bp, ret, count, throwPlease)) return false;
  if (ParseWhiteSpace(bp)) {
    if (throwPlease) throw ParseException("extra data found on end of line: (%s)", *bp);
    return false;
  }
  return true;
}


// 
// ParseInt:
// Same as ParseString except it pulls out an integer
//
bool ParseInt(char **bp, int *ret, bool throwPlease) {
	double f=0.0;
	if (!ParseDouble(bp,&f, throwPlease)) {
    return false;
  }
	*ret=int(f);
	return true;
}

// 
// ParseBool:
// Same as ParseString except it pulls out a true or false
//
bool ParseBool(char **bp, bool *ret, bool throwPlease) {
	char *bufp;
	if (!ParseString(bp,&bufp, throwPlease)) {
    return false;
  }
	if (_stricmp(bufp,"FALSE")==0) {
		*ret = false;
		return true;
	}
	if (_stricmp(bufp,"TRUE")==0) {
		*ret = true;
		return true;
	}

  if (throwPlease) throw ParseException("expected FALSE or TRUE, not %s", bufp);
  return false;
}

//
// ParseWhiteSpace:
// Skips leading space, tabs or comments (; found in whitespace parse)
// Returns false if all whitespace
//
bool ParseWhiteSpace(char **bp) {
	char *b = *bp;
	while ((*b==' ')||(*b=='\t')) b++;
  *bp = b;
  if (*b == ';' || *b == '#' || *b == '\0' || *b == '\n') return false;

  return true;
}

//
// ParseStringFromList:
// Parses a string, and looks for it on a list of tokens. If found, returns the token position, elase returns false (or throws exception if requested). 
//
bool ParseStringFomList(char **bp, int *ret, const char **tokList, const int tokCount, const bool throwPlease) {
  char *tok;
  const char * cmp;
  if (!ParseString(bp, &tok, throwPlease)) return false;
  for (int i = 0; i < tokCount; i++) {
    cmp = tokList[i];
    if (!_stricmp(tok, cmp)) {
      *ret = i;
      return true;
    }
  }
  if (throwPlease) throw ParseException("invalid command name found: %s", tok);
  *ret = -1;
  return false;
}
