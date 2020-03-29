#ifndef _LOG_H_
#define _LOG_H_

#include "config.h"

#ifndef LOG_WITH_TIMESTAMP
#define TRACE(...)  do {\
                            //printf("TRACE: %3d %-10s:", __LINE__, __func__);\
                            //printf(__VA_ARGS__);\
                    } while (0)

#define pr_warn(...)  do {\
                            printf("WARN : ");\
                            printf(__VA_ARGS__);\
                    } while (0)

#define pr_err(...)  do {\
                            printf("ERROR: ");\
                            printf(__VA_ARGS__);\
                    } while (0)

#define pr_info(...)  do {\
                            printf("INFO : ");\
                            printf(__VA_ARGS__);\
                    } while (0)

#define pr_debug(...)  do {\
                            printf("DEBUG: ");\
                            printf(__VA_ARGS__);\
                    } while (0)

#else

#define TRACE(...)  //do {\
                            //printf("[%lu] TRACE: %3d %-10s:", millis(), __LINE__, __func__);\
                            //printf(__VA_ARGS__);\
                    //} while (0)

#define pr_warn(...)  do {\
                            printf("[%lu] WARN : ", millis());\
                            printf(__VA_ARGS__);\
                    } while (0)

#define pr_err(...)  do {\
                            printf("[%lu] ERROR: ", millis());\
                            printf(__VA_ARGS__);\
                    } while (0)

#define pr_info(...)  do {\
                            printf("[%lu] INFO : ", millis());\
                            printf(__VA_ARGS__);\
                    } while (0)

#define pr_debug(...)  do {\
                            printf("[%lu] DEBUG: ", millis());\
                            printf(__VA_ARGS__);\
                    } while (0)

#endif


#define pr_raw(...)     printf(__VA_ARGS__)

int serial_putc(char c, struct __file *);
void log_init(void);

#endif