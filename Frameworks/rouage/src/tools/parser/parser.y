/* Rouage xml parser */

/* declarations */
%{
#define YYSTYPE int
%}

%token NAME
%token NUM
%token TEXT
%token OPEN_TAG_XML  /* <?xml */
%token CLOSE_TAG_XML  /* ?> */
%token OPEN_TAG_DOC  /* <!DOCTYPE */
%token CLOSE_TAG     /* > */
%token OPEN_TAG_ROBOT    /* <robot */
%token CLOSE_TAG_ROBOT    /* </robot */
%token OPEN_TAG_MODULE   /* <module */
%token CLOSE_TAG_MODULE  /* </module> */
%token CLOSE_TAG_EMPTY   /* /> */
%token SYSTEM_TAG      /*  "SYSTEM" */

%start input

%% /* Grammar */
input: xmlversion dtdfile robot   /* Empty file is forbidden. Only a robot by file, following xml standard header*/
;

xmlversion: OPEN_TAG_XML parameter CLOSE_TAG_XML { /* check xml version validity */ }
;

dtdfile: OPEN_TAG_DOC NAME SYSTEM_TAG value CLOSE_TAG  {/* check dtd spec */}
;

robot: OPEN_TAG_ROBOT parameter CLOSE_TAG module CLOSE_TAG_ROBOT {/* main module. check Robot class*/}
;

parameter: /*empty*/  
| NAME '=' value;         /* classic parameter form A=B */

value : '"' value '"'     /* value can be between "" */
|  NUM '.' NUM            /* float parameter */
|  NUM                    /* integer parameter*/
|  TEXT                   /* text parameter*/
;



module: /*empty*/                              /* No module */
|  OPEN_TAG_MODULE parameter CLOSE_TAG module CLOSE_TAG_MODULE  /* Modules into modules */
|  OPEN_TAG_MODULE parameter CLOSE_TAG_EMPTY                    /* Empty module */
;


%%
