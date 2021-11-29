/*-----------------------------------------------------------------------------    
* EcVersion.h                                                                      
* Copyright                acontis technologies GmbH, D-88250 Weingarten, Germany  
* Description              EC-Master version information.                          
*---------------------------------------------------------------------------*/     
#define ATECAT_VERS_MAJ             3   /* major version */             
#define ATECAT_VERS_MIN             0   /* minor version */             
#define ATECAT_VERS_SERVICEPACK     3   /* service pack */           
#define ATECAT_VERS_BUILD           14   /* build number */              
#define ATECAT_VERSION (  (ATECAT_VERS_MAJ << 3*8)              \
                        | (ATECAT_VERS_MIN << 2*8)              \
                        | (ATECAT_VERS_SERVICEPACK << 1*8)      \
                        | (ATECAT_VERS_BUILD << 0*8)            \
                       )
#define ATECAT_FILEVERSION      3,0,3,14 
#if (defined EVAL_VERSION) 
#define ATECAT_FILEVERSIONSTR   "3.0.3.14 (Protected)\0" 
#else 
#define ATECAT_FILEVERSIONSTR   "3.0.3.14\0" 
#endif   
#define ATECAT_PRODVERSION      ATECAT_FILEVERSION 
#define ATECAT_PRODVERSIONSTR   ATECAT_FILEVERSIONSTR 
#define ATECAT_COPYRIGHT        "Copyright acontis technologies GmbH @ 2020\0" 
