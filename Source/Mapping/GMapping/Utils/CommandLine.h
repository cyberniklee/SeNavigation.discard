#ifndef _COMMANDLINE_H_
#define _COMMANDLINE_H_

#define parseFlag(name,value)\
if (!strcmp(argv[c],name)){\
	value=true;\
	cout << name << " on"<< endl;\
	recognized=true;\
}\

#define parseString(name,value)\
if (!strcmp(argv[c],name) && c<argc-1){\
	c++;\
	value=argv[c];\
	cout << name << "=" << value << endl;\
	recognized=true;\
}\


#define parseDouble(name,value)\
if (!strcmp(argv[c],name) && c<argc-1){\
	c++;\
	value=atof(argv[c]);\
	cout << name << "=" << value << endl;\
	recognized=true;\
}\

#define parseInt(name,value)\
if (!strcmp(argv[c],name) && c<argc-1){\
	c++;\
	value=atoi(argv[c]);\
	cout << name << "=" << value << endl;\
	recognized=true;\
}\

#define CMD_PARSE_BEGIN(i, count)\
{\
	int c=i;\
	while (c<count){\
		bool recognized=false;

#define CMD_PARSE_END\
		if (!recognized)\
			cout << "COMMAND LINE: parameter " << argv[c] << " not recognized" << endl;\
		c++;\
	}\
}

#define CMD_PARSE_BEGIN_SILENT(i, count)\
{\
	int c=i;\
	while (c<count){\
		bool recognized=false;

#define CMD_PARSE_END_SILENT\
		c++;\
	}\
}

#define parseFlagSilent(name,value)\
if (!strcmp(argv[c],name)){\
	value=true;\
	recognized=true;\
}\

#define parseStringSilent(name,value)\
if (!strcmp(argv[c],name) && c<argc-1){\
	c++;\
	value=argv[c];\
	recognized=true;\
}\


#define parseDoubleSilent(name,value)\
if (!strcmp(argv[c],name) && c<argc-1){\
	c++;\
	value=atof(argv[c]);\
	recognized=true;\
}\

#define parseIntSilent(name,value)\
if (!strcmp(argv[c],name) && c<argc-1){\
	c++;\
	value=atoi(argv[c]);\
	recognized=true;\
}\


#endif

